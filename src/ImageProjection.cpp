#include "faster_lio_sam/common.h"

using namespace std;

// 缓存队列长度
const int queueLength = 2000;

class ImageProjection : public parameter
{
private:
    // 线程锁
    std::mutex imuLock;
    std::mutex odoLock;
    std::mutex biasLock;

    ros::Subscriber subLaserCloud;
    ros::Subscriber subImuBias;

    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubOriginalCloud;
    ros::Publisher pubExtractedCloudRGB;

    // 保存imu
    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    // 保存里程计
    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    // 保存点云
    std::deque<faster_lio_sam::merge_cloud> cloudQueue;
    faster_lio_sam::merge_cloud currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    PointCloudXYZIT::Ptr laserCloudIn;
    PointCloudXYZIT::Ptr   fullCloud; // 一帧雷达的所有数据

    int deskewFlag;
    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    faster_lio_sam::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    bool imuAvailable = false;

    Eigen::Vector3d accBias;
    Eigen::Vector3d gyrBias;

public:
    ImageProjection():deskewFlag(0)
    {
        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<faster_lio_sam::merge_cloud>("/livox_merge_pcl0", 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        subImuBias    = nh.subscribe<std_msgs::Float64MultiArray>("lio_sam/imu/bias", 5, &ImageProjection::imuBiasHandler, this, ros::TransportHints().tcpNoDelay());

        pubOriginalCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_original", 1);
        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed", 1);
        pubExtractedCloudRGB = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed_rgb", 1);
        pubLaserCloudInfo = nh.advertise<faster_lio_sam::cloud_info> ("lio_sam/deskew/cloud_info", 1);

        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){
        laserCloudIn.reset(new PointCloudXYZIT);
        fullCloud.reset(new PointCloudXYZIT);

        fullCloud->resize(N_SCAN_0*Horizon_SCAN_0);

        accBias.setZero();
        gyrBias.setZero();

        resetParameters();

    }

    void resetParameters(){
        laserCloudIn->clear();

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        // 将原始IMU数据通过外参变换转到雷达坐标系下
        sensor_msgs::Imu thisImu = *imuMsg;

        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(biasLock);

        thisImu.linear_acceleration.x -= accBias(0);
        thisImu.linear_acceleration.y -= accBias(1);
        thisImu.linear_acceleration.z -= accBias(2);
        thisImu.angular_velocity.x -= gyrBias(0);
        thisImu.angular_velocity.y -= gyrBias(1);
        thisImu.angular_velocity.z -= gyrBias(2);
        imuQueue.push_back(thisImu);
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void imuBiasHandler(const std_msgs::Float64MultiArray::ConstPtr& imuBiasMsg)
    {
        std::lock_guard<std::mutex> lock1(biasLock);
        for(int i=0; i<3; i++)
        {
            accBias(i) = imuBiasMsg->data[1+i];
            gyrBias(i) = imuBiasMsg->data[3+i];
        }
    }

    void cloudHandler(const faster_lio_sam::merge_cloud::ConstPtr& mergeLaserCloudMsg)
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // @TODO: 将点云转到IMU坐标系下

        if (!cachePointCloud(mergeLaserCloudMsg))
            return;

        // 获取当前点云持续时间的旋转平移
        // 旋转的角速度由imu积分得到，平移量由里程计得到
        if (!deskewInfo())
            return;

        // 点云整理索引、矫正畸变
        projectPointCloud();

        // 发布预处理后的点云
        publishClouds();

        resetParameters();

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        ROS_DEBUG("solve time cost = %f seconds.", time_used.count());
    }

    // 缓存点云消息，转换成pcl类型点云，记录时间戳，去除无效点
    bool cachePointCloud(const faster_lio_sam::merge_cloud::ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if(cloudQueue.size()<=1)
            return false;
        
        // convert cloud
        currentCloudMsg = cloudQueue.front();
        cloudQueue.pop_front();
        pcl::fromROSMsg(currentCloudMsg.merge_cloud, *laserCloudIn);

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        // 该点云帧第一个点的时间戳
        timeScanCur = cloudHeader.stamp.toSec();
        // 该点云帧最后一个点的时间戳
        timeScanEnd = currentCloudMsg.scanEndTime;

        // cout<<laserCloudIn->size()<<endl;

        return true;
    }

    // 根据IMU pose和IMU odometry，提前计算整帧点云的去畸变参数，在projectPointCloud会根据时间偏移量校正每个点
    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        // 确保IMU队列的时间跨度大于一帧雷达
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        // 修剪imu的数据队列，直到imu的时间处于这帧点云的时间内
        // 之后对当前队列中的imu进行积分，角加速度乘时间，求得旋转的角速度
        imuDeskewInfo();

        // 获取这帧点云时间内的　第一个和最后一个里程计信息，将第一个odom当成初始值放入cloudInfo
        // 求得第一个和最后一个里程计间的平移量
        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        imuAvailable = false;
        // 修剪imu的数据队列，直到imu的时间处于这帧点云的时间前
        sensor_msgs::Imu curImu;
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur)
            {
                curImu = imuQueue.front();
                imuQueue.pop_front();
            }
            else
                break;
        }
        imuQueue.push_front(curImu);

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        // cout<<imuQueue.size()<<endl;
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            // 获取首个IMU消息
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            // 时间戳
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            // imu的时间合适，应用ros的tf库将四元数转变成 rpy 作为点云的初始姿态
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            // 到达点云帧末，停止提取IMU
            if (currentImuTime > timeScanEnd)
                break;

            // 记录本帧雷达扫描内IMU的旋转量
            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            // 提取IMU的角速度
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            // 角速度积分
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        // 判断是否积分有效
        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        // 修剪odom的数据队列
        nav_msgs::Odometry startOdomMsg;
        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur)
            {
                startOdomMsg = odomQueue.front();
                odomQueue.pop_front();
            }
            else
                break;
        }
        odomQueue.push_front(startOdomMsg);

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd-0.01)
                continue;
            else
                break;
        }

        // Create a transformation from the given translation and Euler angles (XYZ-convention)
        // 起始的位姿-->齐次变换矩阵
        Eigen::Affine3f transBegin = Eigen::Affine3f::Identity();
        transBegin.pretranslate(Eigen::Vector3f(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z));
        transBegin.rotate(Eigen::Quaternionf(startOdomMsg.pose.pose.orientation.w, startOdomMsg.pose.pose.orientation.x, startOdomMsg.pose.pose.orientation.y, startOdomMsg.pose.pose.orientation.z));

        // quaterion --> eular --> 齐次变换
        // 末端的位姿
        Eigen::Affine3f transEnd = Eigen::Affine3f::Identity();
        transEnd.pretranslate(Eigen::Vector3f(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z));
        transEnd.rotate(Eigen::Quaternionf(endOdomMsg.pose.pose.orientation.w, endOdomMsg.pose.pose.orientation.x, endOdomMsg.pose.pose.orientation.y, endOdomMsg.pose.pose.orientation.z));

        // 计算帧间变换
        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        // 获取变换增量
        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    // 根据IMU预积分得到的相对旋转，得到点的相对旋转进行矫正畸变
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || imuAvailable == false)
            return *point;

        // 点的绝对时间
        double pointTime = timeScanCur + relTime*10.0*(timeScanEnd-timeScanCur);

        // 通过线性差值，找到pointTime对应的旋转
        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        // 位移畸变很小，忽视
        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;
        newPoint.time = point->time;

        return newPoint;
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // fullCloud->resize(cloudSize);
        fullCloud->clear();
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint = laserCloudIn->points[i];

            // 去除过近或过远的点
            float range = pointDistance(thisPoint);
            //               1.0                    200.0
            if (range < lidarMinRange_0 || range > lidarMaxRange_0)
                continue;

            // 对点云中的每个点进行　畸变校正
            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            // fullCloud->points[i] = thisPoint;
            fullCloud->push_back(thisPoint);
        }
    }

    void RGBpointAssociateToMap(PointType const *const pi,
                            pcl::PointXYZRGB *const po) {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        po->x = point_curr.x();
        po->y = point_curr.y();
        po->z = point_curr.z();
        int reflection_map = (int)pi->intensity;
        if (reflection_map < 30) {
            int green = (reflection_map * 255 / 30);
            po->r = 0;
            po->g = green & 0xff;
            po->b = 0xff;
        } else if (reflection_map < 90) {
            int blue = (((90 - reflection_map) * 255) / 60);
            po->r = 0x0;
            po->g = 0xff;
            po->b = blue & 0xff;
        } else if (reflection_map < 150) {
            int red = ((reflection_map - 90) * 255 / 60);
            po->r = red & 0xff;
            po->g = 0xff;
            po->b = 0x0;
        } else {
            int green = (((255 - reflection_map) * 255) / (255 - 150));
            po->r = 0xff;
            po->g = green & 0xff;
            po->b = 0;
        }
    }

    // 将点云封装成cloud_info并发布
    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, fullCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
        if(pubOriginalCloud.getNumSubscribers()!=0)
        {
            publishCloud(&pubOriginalCloud, laserCloudIn, cloudHeader.stamp, lidarFrame);
        }

        if(pubExtractedCloudRGB.getNumSubscribers()!=0)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZRGB>());
            laserCloudFullResColor->resize(fullCloud->size());
            for(int i=0; i<fullCloud->size(); i++)
            {
                RGBpointAssociateToMap(&fullCloud->points[i], &laserCloudFullResColor->points[i]);
            }
            sensor_msgs::PointCloud2 laserCloudFullRGB;
            pcl::toROSMsg(*laserCloudFullResColor, laserCloudFullRGB);
            laserCloudFullRGB.header.stamp = cloudHeader.stamp;
            laserCloudFullRGB.header.frame_id = lidarFrame;
            pubExtractedCloudRGB.publish(laserCloudFullRGB);
        }
    }
    ~ImageProjection(){};
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "ImageProjection");

    ROS_INFO("\033[1;32m----> start ImageProjection. \033[0m");

    ImageProjection _ImageProjection;

    // 使用了3个线程，这3个线程是ros自己管理的，灵活调用，有可能一个函数同时占用2个线程，并不是为每个回调分配1个线程
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    return 0;
}
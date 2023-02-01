#include "faster_lio_sam/common.h"
#include "faster_lio_sam/FilterState.h"

// #define USE_IKD_TREE

#ifndef USE_IKD_TREE
#include "ivox3d/ivox3d.h"
using namespace faster_lio;
#else
#include "ikd-Tree/ikd_Tree.h"
#endif

using namespace std;

#define POS_ 0
#define ROT_ 3
#define VEL_ 6
#define BIA_ 9
#define BIG_ 12
#define GW_ 15

class mapOptimization : public parameter
{
public:
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using VV4F = std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>;
#ifndef USE_IKD_TREE
    #ifdef IVOX_NODE_TYPE_PHC
        using IVoxType = IVox<3, IVoxNodeType::PHC, PointType>;
    #else
        using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
    #endif
#else
#endif
    enum SYS_STATUS{FIRST_SCAN, OTHER_SCAN};

private:
    SYS_STATUS sysStatus = FIRST_SCAN;
    // 记录世界坐标系下的pvq
    FilterState filterState;
    // 线程锁
    std::mutex imuLock;
    std::mutex biasLock;

    // ros
    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubPath;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubCloudUndisorted;
    ros::Subscriber subCloud;
    ros::Subscriber subImuBias;
    faster_lio_sam::cloud_info cloudInfo;

    // 保存imu
    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;
    std::vector<sensor_msgs::Imu> imuBucket;
    std::vector<sensor_msgs::Imu> imuBucketForCorrect;
    std_msgs::Header laserInfoHeader;
    double timeLaserInfoCur;
    double timeLastProcessing = -1;
    nav_msgs::Path globalPath;
    Eigen::Vector3d accBias;
    Eigen::Vector3d gyrBias;

    PointCloudXYZIT::Ptr laserCloudFullLast;
    PointCloudXYZIT::Ptr laserCloudFullLastDS;
    PointCloudXYZIT::Ptr laserCloudFullLastDS_w;
    PointCloudXYZIT::Ptr coeffSel;
    pcl::VoxelGrid<PointType> downSizeFilterScan;
    #ifdef USE_IKD_TREE
    PointCloudXYZIT::Ptr _featsArray;
    #endif

    std::vector<PointVector> nearestPoints;         // nearest points of current scan
    std::vector<bool> pointSelectedSurf;
    VV4F planeCoef;                         // plane coeffs
    int laserCloudFullLastDSNum = 0;

    #ifndef USE_IKD_TREE
    // ivox
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
    #else
    KD_TREE<PointType> ikdtree;
    vector<vector<int>>  pointSearchInd_surf;
    vector<BoxPointType> cub_needrm;
    vector<PointVector>  Nearest_Points;
    int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
    const float MOV_THRESHOLD = 1.5f;
    const float DET_RANGE = 250.0f;
    #endif

    float transformTobeMapped[18]; // [p_w, q_w, v_w, ba, bq, g_w]
    float transformTobeMappedLast[18]; // [p_w, q_w, v_w, ba, bq, g_w]

    tf::StampedTransform aftMappedTrans;
    tf::TransformBroadcaster tfBroadcaster;

    bool systemInitial = false;
    bool isDegenerate;

    // prediction
    Eigen::Matrix<float, 18, 18> F_t;
    Eigen::Matrix<float, 18, 12> G_t;
    Eigen::Matrix<float, 18, 18> P_t;
    Eigen::Matrix<float, 18, 18> P_t_inv;
    Eigen::Matrix<float, 12, 12> noise_;
    // measurement
    Eigen::Matrix<float, Eigen::Dynamic, 1> residual_;
    Eigen::Matrix<float, Eigen::Dynamic, 18> H_k;
    Eigen::Matrix<float, 18, Eigen::Dynamic> K_k;
    Eigen::Matrix<float, 18, 1> updateVec_;
    Eigen::Matrix<float, 18, 1> errState;
    Eigen::Matrix<float, 18, 18> HRH;
    Eigen::Matrix<float, 18, 18> HRH_inv;

    Eigen::Matrix<float, 1, 6> matE;
    Eigen::Matrix<float, 6, 6> matV;
    Eigen::Matrix<float, 6, 6> matV2;
    Eigen::Matrix<float, 6, 6> matP;

public:
    mapOptimization()
    {
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);
        pubCloudUndisorted = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_undisorted", 1);

        subCloud = nh.subscribe<faster_lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subImu   = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &mapOptimization::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subImuBias    = nh.subscribe<std_msgs::Float64MultiArray>("lio_sam/imu/bias", 5, &mapOptimization::imuBiasHandler, this, ros::TransportHints().tcpNoDelay());
        downSizeFilterScan.setLeafSize(filter_size_surf, filter_size_surf, filter_size_surf);

        aftMappedTrans.frame_id_ = odometryFrame;
        // aftMappedTrans.child_frame_id_ = lidarFrame;
        aftMappedTrans.child_frame_id_ = lidarFrame;

        allocateMemory();
    }
    ~mapOptimization(){}

    void allocateMemory()
    {
        #ifndef USE_IKD_TREE
        ivox_options_.resolution_ = ivox_grid_resolution;
        if (ivox_nearby_type == 0) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
        } else if (ivox_nearby_type == 6) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
        } else if (ivox_nearby_type == 18) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
        } else if (ivox_nearby_type == 26) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
        } else {
            ROS_WARN("unknown ivox_nearby_type, use NEARBY18");
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
        }
        ivox_options_.capacity_ = ivox_capacity;
        // localmap init (after LoadParams)
        ivox_ = std::make_shared<IVoxType>(ivox_options_);
        #else
        _featsArray.reset(new PointCloudXYZIT());
        #endif

        laserCloudFullLast.reset(new PointCloudXYZIT());
        laserCloudFullLastDS.reset(new PointCloudXYZIT());
        laserCloudFullLastDS_w.reset(new PointCloudXYZIT());
        coeffSel.reset(new PointCloudXYZIT());

        for (int i = 0; i < 18; ++i){
            transformTobeMapped[i] = 0;
            transformTobeMappedLast[i] = 0;
        }
        transformTobeMapped[GW_+2] = -imuGravity;
        transformTobeMappedLast[GW_+2] = -imuGravity;
        filterState.gn_ = Eigen::Vector3f(0.0, 0.0, -imuGravity);

        isDegenerate = false;

        F_t.setZero();
        G_t.setZero();
        P_t.setZero();
        noise_.setZero();
        // asDiagonal()指将向量作为对角线构建对角矩阵
        noise_.block<3, 3>(0, 0) = Eigen::Vector3f(imuAccNoise, imuAccNoise, imuAccNoise).asDiagonal();
        noise_.block<3, 3>(3, 3) = Eigen::Vector3f(imuGyrNoise, imuGyrNoise, imuGyrNoise).asDiagonal();
        noise_.block<3, 3>(6, 6) = Eigen::Vector3f(imuAccBiasN, imuAccBiasN, imuAccBiasN).asDiagonal();
        noise_.block<3, 3>(9, 9) = Eigen::Vector3f(imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).asDiagonal();

        errState.setZero();
        HRH.setZero();
        HRH_inv.setZero();

        accBias.setZero();
        gyrBias.setZero();
        updateVec_.setZero();

        matE.setZero();
        matV.setZero();
        matV2.setZero();
        matP.setZero();
        
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        // 将原始IMU数据通过外参变换转到雷达坐标系下
        sensor_msgs::Imu thisImu = *imuMsg;

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    void imuBiasHandler(const std_msgs::Float64MultiArray::ConstPtr& imuBiasMsg)
    {
        std::lock_guard<std::mutex> lock1(biasLock);
        for(int i=0; i<3; i++)
        {
            accBias(i) = imuBiasMsg->data[1+i];
            gyrBias(i) = imuBiasMsg->data[4+i];
        }
    }

    void laserCloudInfoHandler(const faster_lio_sam::cloud_infoConstPtr& msgIn)
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        laserInfoHeader = msgIn->header;
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        // extract info
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_deskewed, *laserCloudFullLast);

        if(timeLaserInfoCur-timeLastProcessing >= 0.05)
        {
            switch (sysStatus)
            {
                case FIRST_SCAN:
                {
                    updateInitialPose();
                    removeOldImu();
                    handleFirstScan();
                    sysStatus = OTHER_SCAN;
                    break;
                }
                case OTHER_SCAN:
                {
                    predictByFilter();
                    downsampleCurrentScan();
                    updateTransformationByFilter();
                    updateFilterState(timeLaserInfoCur-timeLastProcessing);
                    mapIncremental();
                    updatePath(transformTobeMapped);
                    publishOdometry();
                    publishFrames();
                    break;
                }
            }
            for(int i=0; i<18; i++)
            {
                transformTobeMappedLast[i] = transformTobeMapped[i];
            }
            timeLastProcessing = timeLaserInfoCur;
        }
        
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        ROS_DEBUG("solve time cost = %f seconds.", time_used.count());
    }

    void transformPoint(const PointType& pi, PointType& po, const Eigen::Affine3f& transIn)
    {
        po = pi;
        po.x = transIn(0,0) * pi.x + transIn(0,1) * pi.y + transIn(0,2) * pi.z + transIn(0,3);
        po.y = transIn(1,0) * pi.x + transIn(1,1) * pi.y + transIn(1,2) * pi.z + transIn(1,3);
        po.z = transIn(2,0) * pi.x + transIn(2,1) * pi.y + transIn(2,2) * pi.z + transIn(2,3);
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, float* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = trans2Affine3f(transformIn);

        vector<int> index(cloudSize, 0);
        for(int i=0; i<cloudSize; i++)
            index[i] = i;
        // 采用引用捕获
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&cloudIn, &cloudOut, &transCur](int i)
        {
            PointType& pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
            cloudOut->points[i].time = pointFrom.time;
        });
        return cloudOut;
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[POS_+0], transformIn[POS_+1], transformIn[POS_+2], transformIn[ROT_+0], transformIn[ROT_+1], transformIn[ROT_+2]);
    }

    void updateInitialPose()
    {
        // 根据第一帧点云的IMU姿态初始化
        transformTobeMapped[ROT_+0] = cloudInfo.imuRollInit;
        transformTobeMapped[ROT_+1] = cloudInfo.imuPitchInit;
        transformTobeMapped[ROT_+2] = cloudInfo.imuYawInit;

        // 使用绝对航向角还是设为0？
        if(!useImuHeadingInitialization)
            transformTobeMapped[ROT_+2] = 0;
    }

    void removeOldImu()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        sensor_msgs::Imu frontImu;
        while(!imuQueue.empty())
        {
            if(imuQueue.front().header.stamp.toSec()<timeLaserInfoCur)
            {
                frontImu = imuQueue.front();
                imuQueue.pop_front();
            }
            else
                break;
        }
        imuQueue.push_front(frontImu); // Imu coverage lidar
    }

    void handleFirstScan()
    {
        // 1. 根据初始估计，将点云转移到世界坐标系下
        PointCloudXYZIT::Ptr laserCloudFullLastInMap(new PointCloudXYZIT());
        laserCloudFullLastInMap = transformPointCloud(laserCloudFullLast, transformTobeMapped);
        #ifndef USE_IKD_TREE
        // 2. 将转换后的点云注册到ivox中
        ivox_->AddPoints(laserCloudFullLastInMap->points);
        #else
        /*** initialize the map kdtree ***/
        // 构建kd树
        // 设置ikd tree的降采样参数
        ikdtree.set_downsample_param(filter_size_map);
        // 组织ikd tree
        ikdtree.Build(laserCloudFullLastInMap->points);
        // 获取ikd tree中的有效节点数，无效点就是被打了deleted标签的点
        int featsFromMapNum = ikdtree.validnum();
        // 获取Ikd tree中的节点数
        kdtree_size_st = ikdtree.size();
        #endif
    }

    void getImuBucket()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        imuBucket.clear();
        sensor_msgs::Imu thisImu;
        //imu    last|---------------|---------------|cur
        //lidar   last|***************|***************|cur
        while(!imuQueue.empty())
        {
            if(imuQueue.front().header.stamp.toSec() < timeLaserInfoCur)
            {
                thisImu = imuQueue.front();
                imuBucket.emplace_back(thisImu);
                imuQueue.pop_front();
            }
            else
                break;
        }
        imuQueue.push_front(thisImu);
    }

    void downsampleCurrentScan()
    {
        laserCloudFullLastDS->clear();
        for(int i=0; i<laserCloudFullLast->size(); i++)
            if(i%point_filter_num==0)
                laserCloudFullLastDS->points.emplace_back(laserCloudFullLast->points[i]);

        // Downsample cloud from current scan
        downSizeFilterScan.setInputCloud(laserCloudFullLastDS);
        downSizeFilterScan.filter(*laserCloudFullLastDS);
        laserCloudFullLastDSNum = laserCloudFullLastDS->size();
    }

    void featureMatching(int iterCount)
    {
        Eigen::Affine3f transCur = trans2Affine3f(transformTobeMapped);
        laserCloudFullLastDS_w->resize(laserCloudFullLastDSNum);
        nearestPoints.resize(laserCloudFullLastDSNum, PointVector());
        pointSelectedSurf.resize(laserCloudFullLastDSNum, false);
        planeCoef.resize(laserCloudFullLastDSNum, Eigen::Vector4f::Zero());
        coeffSel->resize(laserCloudFullLastDSNum);

        int cnt_pts = laserCloudFullLastDSNum;
        vector<size_t> index(cnt_pts);
        for(int i=0; i<cnt_pts; i++)
            index[i] = i;
        std::mutex m;
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&, this](int i){
            PointType& pointBody = laserCloudFullLastDS->points[i];
            PointType& pointWorld = laserCloudFullLastDS_w->points[i];
            auto &pointsNear = nearestPoints[i];
            transformPoint(pointBody, pointWorld, transCur);
            #ifndef USE_IKD_TREE
            ivox_->GetClosestPoint(pointWorld, pointsNear, NUM_MATCH_POINTS);
            pointSelectedSurf[i] = pointsNear.size()>=MIN_NUM_MATCH_POINTS;
            #else
            vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
            ikdtree.Nearest_Search(pointWorld, NUM_MATCH_POINTS, pointsNear, pointSearchSqDis);
            pointSelectedSurf[i] = pointsNear.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 1.5 ? false : true;
            #endif
            if(pointSelectedSurf[i])
            {
                pointSelectedSurf[i] = esti_plane(planeCoef[i], pointsNear, esti_plane_threshold);
            }
            if(pointSelectedSurf[i])
            {
                // 记录合适的法向量、点(base)、点面距离
                auto temp = pointWorld.getVector4fMap();
                temp[3] = 1.0;
                float pd2 = planeCoef[i].dot(temp);
                float s;
                // s = 1- 0.9 * fabs(pd2)/ sqrt(sqrt(pointBody.x * pointBody.x
                //             + pointBody.y * pointBody.y + pointBody.z * pointBody.z));
                s = sqrt(pointBody.x * pointBody.x
                    + pointBody.y * pointBody.y + pointBody.z * pointBody.z)
                    - 81 * pd2 * pd2;
                if(s > 0.0)
                {
                    pointSelectedSurf[i] = true;
                    // 法向量：V4f planeCoef[i]
                    coeffSel->points[i].x = planeCoef[i][0]; // 恢复未归一化的法向量
                    coeffSel->points[i].y = planeCoef[i][1];
                    coeffSel->points[i].z = planeCoef[i][2];
                    // 原始点：PointType laserCloudFullLastDS->points[i]
                    // 残差
                    coeffSel->points[i].intensity = pd2;
                }
            }
        }
        );
    }

    void predictByFilter()
    {
        biasLock.lock();
        filterState.ba_ = accBias.cast<float>();
        filterState.bw_ = gyrBias.cast<float>();
        biasLock.unlock();
        //imu    last|---------------|---------------|cur
        //lidar   last|***************|***************|cur
        getImuBucket();
        if(imuBucket.size()<=3)
            return;
        // 将transformTobeMapped转成矩阵形式
        Eigen::Affine3f T_transformTobeMapped = trans2Affine3f(transformTobeMapped);
        // rotation, position, velocity
        Eigen::Quaternionf R_transformTobeMapped(T_transformTobeMapped.rotation());
        Eigen::Vector3f P_transformTobeMapped = T_transformTobeMapped.matrix().block(0, 3, 3, 1);
        Eigen::Vector3f V_transformTobeMapped = Eigen::Vector3f(transformTobeMapped[VEL_+0],
            transformTobeMapped[VEL_+1], transformTobeMapped[VEL_+2]);
        
        Eigen::Vector3f un_acc_last, un_gyr_last, un_acc_next, un_gyr_next;
        imuData thisImu(imuBucket[0]);
        imuData lastImu = thisImu;
        double imuTime_last = thisImu.timestamp;
        double dt = 0.0;
        // convert to w frame (remove the grarity)
        un_acc_last = R_transformTobeMapped*(thisImu.acc-filterState.ba_)+filterState.gn_;
        un_gyr_last = thisImu.gyr-filterState.bw_;
        for(int i=1; i<imuBucket.size(); i++)
        {
            thisImu.set(imuBucket[i]);
            dt = thisImu.timestamp - imuTime_last;
            un_acc_next = R_transformTobeMapped*(thisImu.acc-filterState.ba_)+filterState.gn_;
            un_gyr_next = thisImu.gyr-filterState.bw_;
            Eigen::Vector3f un_acc = 0.5*(un_acc_last + un_acc_next); // world frame
            Eigen::Vector3f un_gyr = 0.5*(un_gyr_last + un_gyr_next);
            // 求角度变化量，再转化成李群
            Eigen::Quaternionf dq = axis2Quat(un_gyr*dt);
            // 更新PVQ
            R_transformTobeMapped = (R_transformTobeMapped*dq).normalized(); // base坐标系的变换，右乘
            V_transformTobeMapped = V_transformTobeMapped+un_acc*dt;
            P_transformTobeMapped = P_transformTobeMapped+V_transformTobeMapped*dt+un_acc*dt*dt;

            // predict relative transformation
            filterState.rn_ = P_transformTobeMapped;
            filterState.vn_ = V_transformTobeMapped;
            filterState.qbn_ = R_transformTobeMapped;
            // filterState.ba_?
            // filterState.bw_? 后续看一下怎么更新比较好，参考预积分的线性化
            // filterState.gn_?

            F_t.setIdentity();
            Eigen::Vector3f midAcc = 0.5*(thisImu.acc + lastImu.acc);
            Eigen::Vector3f midGyr = 0.5*(thisImu.gyr + lastImu.gyr);
            F_t.block<3, 3>(POS_, VEL_) = dt*Eigen::Matrix3f::Identity();
            F_t.block<3, 3>(ROT_, ROT_) = Eigen::Matrix3f::Identity() + anti_symmetric<float>(-dt*(midGyr-filterState.bw_));
            F_t.block<3, 3>(ROT_, BIG_) = -dt*Eigen::Matrix3f::Identity();
            F_t.block<3, 3>(VEL_, ROT_) = -dt*filterState.qbn_.toRotationMatrix()*anti_symmetric<float>(midAcc-filterState.ba_);
            F_t.block<3, 3>(VEL_, BIA_) = -dt*filterState.qbn_.toRotationMatrix();
            F_t.block<3, 3>(VEL_, GW_)  = dt*Eigen::Matrix3f::Identity();

            G_t.setZero();
            G_t.block<3, 3>(VEL_, 0) = -filterState.qbn_.toRotationMatrix();
            G_t.block<3, 3>(ROT_, 3) = -Eigen::Matrix<float, 3, 3>::Identity();
            G_t.block<3, 3>(BIA_, 6) = Eigen::Matrix<float, 3, 3>::Identity();
            G_t.block<3, 3>(BIG_, 9) = Eigen::Matrix<float, 3, 3>::Identity();

            P_t = F_t * P_t * F_t.transpose() + (dt*G_t) * noise_ * (dt*G_t).transpose();

            imuTime_last = thisImu.timestamp;
            un_acc_last = un_acc_next;
            un_gyr_last = un_gyr_next;
            lastImu = thisImu;
        }

        P_t_inv = P_t.colPivHouseholderQr().inverse();

        // remap to transformTobeMapped
        T_transformTobeMapped.setIdentity();
        T_transformTobeMapped.pretranslate(P_transformTobeMapped);
        T_transformTobeMapped.rotate(R_transformTobeMapped);
        pcl::getTranslationAndEulerAngles(T_transformTobeMapped, 
            transformTobeMapped[POS_+0], transformTobeMapped[POS_+1], transformTobeMapped[POS_+2], 
            transformTobeMapped[ROT_+0], transformTobeMapped[ROT_+1], transformTobeMapped[ROT_+2]);
        // 更新速度
        for(int i=0; i<3; i++)
            transformTobeMapped[VEL_+i] = V_transformTobeMapped(i, 0);
    }

    bool updateTransformationIESKF(int iterCount)
    {
        double residualNorm = 1e6;
        bool hasConverged = false;
        bool hasDiverged = false;
        // lidar base
        float srx = sin(transformTobeMapped[ROT_+0]);
        float crx = cos(transformTobeMapped[ROT_+0]);
        float sry = sin(transformTobeMapped[ROT_+1]);
        float cry = cos(transformTobeMapped[ROT_+1]);
        float srz = sin(transformTobeMapped[ROT_+2]);
        float crz = cos(transformTobeMapped[ROT_+2]);

        int laserCloudSelNum = laserCloudFullLastDSNum;
        int VaildCount = 0;
        for(int i=0; i<laserCloudFullLastDSNum; i++)
        {
            if(pointSelectedSurf[i]) VaildCount++;
        }
        if (VaildCount < 10) {
            return true; // 直接跳出
        }

        residual_ = Eigen::Matrix<float, Eigen::Dynamic, 1>::Zero(laserCloudSelNum, 1);
        H_k       = Eigen::Matrix<float, Eigen::Dynamic, 18>::Zero(laserCloudSelNum, 18);
        // R_K为测量噪声协方差矩阵的逆
        // R_k       = LIDAR_STD*Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Identity(laserCloudSelNum, laserCloudSelNum);
        // R_k_inv   = (1/LIDAR_STD)*Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Identity(laserCloudSelNum, laserCloudSelNum);
        K_k       = Eigen::Matrix<float, 18, Eigen::Dynamic>::Zero(18, laserCloudSelNum);

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            if(!pointSelectedSurf[i])
                continue;

            pointOri.x = laserCloudFullLastDS->points[i].x;
            pointOri.y = laserCloudFullLastDS->points[i].y;
            pointOri.z = laserCloudFullLastDS->points[i].z;

            coeff.x = coeffSel->points[i].x;
            coeff.y = coeffSel->points[i].y;
            coeff.z = coeffSel->points[i].z;
            coeff.intensity = coeffSel->points[i].intensity;
            float& px = pointOri.x; float& py = pointOri.y; float& pz = pointOri.z;
            float& nx = coeff.x; float& ny = coeff.y; float& nz = coeff.z;
            float arx = py*(nx*(srx*srz + crx*crz*sry) - ny*(crz*srx - crx*sry*srz) +
                        nz*crx*cry) - pz*(ny*(crx*crz + srx*sry*srz) - nx*(crx*srz - crz*srx*sry) + nz*cry*srx);
            float ary = pz*(nx*crx*cry*crz - nz*crx*sry + ny*crx*cry*srz) + py*(nx*cry*crz*srx -
                        nz*srx*sry + ny*cry*srx*srz) - px*(nz*cry + nx*crz*sry + ny*sry*srz);
            float arz = px*(ny*cry*crz - nx*cry*srz) - py*(nx*(crx*crz + srx*sry*srz) +
                        ny*(crx*srz - crz*srx*sry)) + pz*(nx*(crz*srx - crx*sry*srz) +
                        ny*(srx*srz + crx*crz*sry));

            H_k(i, ROT_+0) = arx;
            H_k(i, ROT_+1) = ary;
            H_k(i, ROT_+2) = arz;
            H_k(i, POS_+0) = coeff.x;
            H_k(i, POS_+1) = coeff.y;
            H_k(i, POS_+2) = coeff.z;
            residual_(i, 0) = optimizationStep * coeff.intensity;
        }

        // 原始
        // HRH_P = (H_k.transpose()*R_k_inv*H_k+P_t_inv).block(0, 0, 18, 18);
        // HRH_P_inv = HRH_P.colPivHouseholderQr().inverse();
        // K_k = HRH_P_inv*H_k.transpose()*R_k_inv;

        // 改进， 省略了大矩阵R的计算，速度会快一个数量级
        Eigen::Matrix<float, 18, 18> P_tmp = LIDAR_STD*P_t_inv;
        HRH = H_k.transpose()*H_k+P_tmp;
        HRH_inv = HRH.colPivHouseholderQr().inverse();
        K_k = HRH_inv * H_k.transpose();
        
        updateVec_ = K_k*(H_k*errState - residual_) - errState;

        // cout<<K_k<<endl<<endl;

        // Divergence determination
        // 迭代发散判断
        bool hasNaN = false;
        for (int i = 0; i < updateVec_.size(); i++) {
            if (isnan(updateVec_[i])) {
                updateVec_[i] = 0;
                hasNaN = true;
            }
        }
        if (hasNaN == true) {
            ROS_WARN("System diverges Because of NaN...");
            hasDiverged = true;
            return true;
        }
        // Check whether the filter converges
        // 检查滤波器是否迭代收敛
        if (residual_.norm() > residualNorm * 10) {
            ROS_WARN("System diverges...");
            hasDiverged = true;
            return true;
        }

        if(iterCount==0)
        {
            matE.setZero();
            matV.setZero();
            matV2.setZero();

            Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 6, 6> > esolver(HRH.block<6, 6>(0, 0));
            matE = esolver.eigenvalues().real();
            matV = esolver.eigenvectors().real();

            matV2 = matV.transpose();

            isDegenerate = false;
            float eignThre[6] = { 30, 30, 30, 30, 30, 30 };
            for (int i = 0; i < 6; i++)
            {
               if (matE(0, i) < eignThre[i])
               {
                  for (int j = 0; j < 6; j++)
                  {
                     matV2(i, j) = 0;
                  }
                  isDegenerate = true;
                  ROS_WARN(" Feature degradation !!! ");
               }
               else
               {
                  break;
               }
            }
            //需要进行转置
            matP = matV.transpose().inverse() * matV2;
        }

        if(isDegenerate)
        {
            Eigen::Matrix<float, 6, 1> matX2 = updateVec_.block<6, 1>(0, 0);
            updateVec_.block<6, 1>(0, 0) = matP * matX2;
        }


        errState += updateVec_;

        transformTobeMapped[ROT_+0] += updateVec_(ROT_+0, 0);
        transformTobeMapped[ROT_+1] += updateVec_(ROT_+1, 0);
        transformTobeMapped[ROT_+2] += updateVec_(ROT_+2, 0);
        transformTobeMapped[POS_+0] += updateVec_(POS_+0, 0);
        transformTobeMapped[POS_+1] += updateVec_(POS_+1, 0);
        transformTobeMapped[POS_+2] += updateVec_(POS_+2, 0);
        if(!useUniformMotionForUpdateVel)
        {
            transformTobeMapped[VEL_+0] += updateVec_(VEL_+0, 0);
            transformTobeMapped[VEL_+1] += updateVec_(VEL_+1, 0);
            transformTobeMapped[VEL_+2] += updateVec_(VEL_+2, 0);
        }

        bool coverage = false;
        float deltaR = sqrt(
                            pow(pcl::rad2deg(updateVec_(ROT_+0, 0)), 2) +
                            pow(pcl::rad2deg(updateVec_(ROT_+1, 0)), 2) +
                            pow(pcl::rad2deg(updateVec_(ROT_+2, 0)), 2));
        float deltaT = sqrt(
                            pow(updateVec_(POS_+0, 0) * 100, 2) +
                            pow(updateVec_(POS_+1, 0) * 100, 2) +
                            pow(updateVec_(POS_+2, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            coverage = true;
        }
        return coverage;
    }

    void updateTransformationByFilter()
    {
        for(int iter=0; iter<30; iter++)
        {
            if(iter%3==0) featureMatching(iter);
            if(updateTransformationIESKF(iter))
                break;
        }
        Eigen::Matrix<float, 18, 18> I_ = Eigen::Matrix<float, 18, 18>::Identity();
        // P_t = (I_-K_k*H_k)*P_t*(I_-K_k*H_k).transpose()+K_k*R_k*K_k.transpose();
        P_t = (I_-K_k*H_k)*P_t;
        errState.setZero();
    }

    void updateFilterState(double dt)
    {
        for(int i=0; i<3; i++)
        {
            if(useUniformMotionForUpdateVel)
                transformTobeMapped[VEL_+i] = (transformTobeMapped[POS_+i]-transformTobeMappedLast[POS_+i])/dt;
            filterState.vn_(i) = transformTobeMapped[VEL_+i];

            // 看作匀加速运动
            // transformTobeMapped[VEL_+i] = 2.0*(transformTobeMapped[POS_+i]-transformTobeMappedLast[POS_+i])/dt-transformTobeMappedLast[VEL_+i];
            // filterState.vn_(i) = transformTobeMapped[VEL_+i];
        }
        // filterState.vn_ = Eigen::Vector3f(transformTobeMapped[VEL_+0], transformTobeMapped[VEL_+1], transformTobeMapped[VEL_+2]);
        // 将transformTobeMapped转成矩阵形式
        Eigen::Affine3f T_transformTobeMapped = trans2Affine3f(transformTobeMapped);
        filterState.rn_ = T_transformTobeMapped.translation();
        filterState.qbn_ = T_transformTobeMapped.rotation();
    }

    #ifdef USE_IKD_TREE
    void points_cache_collect()
    {
        PointVector points_history;
        ikdtree.acquire_removed_points(points_history);
        for (int i = 0; i < points_history.size(); i++) 
            _featsArray->push_back(points_history[i]);
    }
    // 动态调整地图区域，防止地图过大而内存溢出，类似LOAM中提取局部地图的方法
    BoxPointType LocalMap_Points; // ikd-tree中,局部地图的包围盒角点
    bool Localmap_Initialized = false;
    void lasermap_fov_segment()
    {
        cub_needrm.clear(); // 清空需要移除的区域
        kdtree_delete_counter = 0;
        // X轴分界点转换到w系下
        // pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
        // global系下lidar位置
        Eigen::Vector3f pos_LiD = Eigen::Vector3f(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
        //初始化局部地图包围盒角点，以为w系下lidar位置为中心
        if (!Localmap_Initialized){ // 系统起始需要初始化局部地图的大小和位置
            for (int i = 0; i < 3; i++){
                LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_side_length / 2.0;
                LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_side_length / 2.0;
            }
            Localmap_Initialized = true;
            return;
        }
        //各个方向上Lidar与局部地图边界的距离，或者说是lidar与立方体盒子六个面的距离
        float dist_to_map_edge[3][2];
        bool need_move = false;
        for (int i = 0; i < 3; i++){
            dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
            //与某个方向上的边界距离（例如1.5*300m）太小，标记需要移除need_move，参考论文Fig3
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
        }
        // 没有超出地图则返回
        if (!need_move) return;
        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        // 新的局部地图盒子边界点
        New_LocalMap_Points = LocalMap_Points;
        float mov_dist = max((cube_side_length - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
        for (int i = 0; i < 3; i++){
            tmp_boxpoints = LocalMap_Points;
            //与包围盒最小值边界点距离
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
                cub_needrm.push_back(tmp_boxpoints); // 移除较远包围盒
            } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            }
        }
        LocalMap_Points = New_LocalMap_Points;

        points_cache_collect();
        // 使用Boxs删除指定盒内的点
        if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    }
    #endif

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    void updatePath(const float pose_in[])
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = cloudInfo.header.stamp;
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in[0];
        pose_stamped.pose.position.y = pose_in[1];
        pose_stamped.pose.position.z = pose_in[2];
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in[3], pose_in[4], pose_in[5]);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void mapIncremental()
    {
        PointVector points_to_add;
        PointVector point_no_need_downsample;

        int cur_pts = laserCloudFullLastDS->size();
        points_to_add.reserve(cur_pts);
        point_no_need_downsample.reserve(cur_pts);

        std::vector<size_t> index(cur_pts);
        for (size_t i = 0; i < cur_pts; ++i) {
            index[i] = i;
        }
        /* transform to world frame */
        laserCloudFullLastDS_w = transformPointCloud(laserCloudFullLastDS, transformTobeMapped);
        std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i){
            PointType &point_world = laserCloudFullLastDS_w->points[i];

            if(!nearestPoints[i].empty())
            {
                const PointVector &points_near = nearestPoints[i];
                Eigen::Vector3f center =
                    ((point_world.getVector3fMap() / filter_size_map).array().floor() + 0.5) * filter_size_map;
                Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

                if (fabs(dis_2_center.x()) > 0.5 * filter_size_map &&
                    fabs(dis_2_center.y()) > 0.5 * filter_size_map &&
                    fabs(dis_2_center.z()) > 0.5 * filter_size_map) {
                    point_no_need_downsample.emplace_back(point_world);
                    return;
                }

                bool need_add = true;
                float dist = calc_dist(point_world.getVector3fMap(), center);
                if (points_near.size() >= NUM_MATCH_POINTS) {
                    for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                        if (calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                            need_add = false;
                            break;
                        }
                    }
                }
                if (need_add) {
                    points_to_add.emplace_back(point_world);
                }
            }
            else
            {
                points_to_add.emplace_back(point_world);
            }
        });
        #ifndef USE_IKD_TREE
        ivox_->AddPoints(points_to_add);
        ivox_->AddPoints(point_no_need_downsample);
        #else
        ikdtree.Add_Points(points_to_add, true);
        ikdtree.Add_Points(point_no_need_downsample, false);
        #endif
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = cloudInfo.header.stamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "/odom_mapping";
         laserOdometryROS.pose.pose.position.x = transformTobeMapped[POS_+0];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[POS_+1];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[POS_+2];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[ROT_+0], transformTobeMapped[ROT_+1], transformTobeMapped[ROT_+2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);

        aftMappedTrans.stamp_ = cloudInfo.header.stamp;
        aftMappedTrans.setRotation(tf::createQuaternionFromRPY(transformTobeMapped[ROT_+0], transformTobeMapped[ROT_+1], transformTobeMapped[ROT_+2]));
        aftMappedTrans.setOrigin(tf::Vector3(transformTobeMapped[POS_+0], transformTobeMapped[POS_+1], transformTobeMapped[POS_+2]));
        tfBroadcaster.sendTransform(aftMappedTrans);
    }

    void publishFrames()
    {
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = cloudInfo.header.stamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }

        if(pubCloudUndisorted.getNumSubscribers()!=0)
        {
            publishCloud(&pubCloudUndisorted, laserCloudFullLastDS, cloudInfo.header.stamp, lidarFrame);
        }
    }
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "FusionOptimization");

    ROS_INFO("\033[1;32m----> start FusionOptimization. \033[0m");

    mapOptimization mapOptimization_;

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}
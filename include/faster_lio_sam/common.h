#ifndef _COMMON_H_
#define _COMMON_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <faster_lio_sam/cloud_info.h>
#include <faster_lio_sam/merge_cloud.h>

#include <opencv2/opencv.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <execution>

using namespace std;

struct PointXYZIT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen的字段对齐
} EIGEN_ALIGN16;

// 将 PointXYZIRT 在pcl中注册，其包括的字段为 x,y,z,intensity,ring,time
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (float, time, time)
)

typedef PointXYZIT PointType;
typedef pcl::PointCloud<PointXYZIT> PointCloudXYZIT;

using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

constexpr int NUM_MATCH_POINTS = 5;      // required matched points in current
constexpr int MIN_NUM_MATCH_POINTS = 3;  // minimum matched points in current
constexpr float LIDAR_STD = 0.001;

inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }

inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

struct Transform{
    Eigen::Vector3f translate;
    Eigen::Quaternionf rotate;
    Transform(){
        translate.setZero();
        rotate.setIdentity();
    }
    Transform(Eigen::Vector3f t, Eigen::Quaternionf q):translate(t), rotate(q){}
};

struct imuData{
    double timestamp;
    Eigen::Vector3f acc;
    Eigen::Vector3f gyr;
    imuData(const sensor_msgs::Imu& msg)
    {
        set(msg);
    }
    void set(const sensor_msgs::Imu& msg)
    {
        timestamp = msg.header.stamp.toSec();
        acc = Eigen::Vector3f(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
        gyr = Eigen::Vector3f(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    }
};

class parameter
{
protected:
    ros::NodeHandle nh;

    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    Transform extImuToBase;
    Transform extLivoxToBase;

    int N_SCAN_0;
    int Horizon_SCAN_0;
    float lidarMinRange_0;
    float lidarMaxRange_0;

    // IMU
    int imuFrequence;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;

    float z_tollerance; 
    float rotation_tollerance;

    float ivox_grid_resolution;
    int ivox_nearby_type;
    int ivox_capacity;
    float esti_plane_threshold;
    int cube_side_length;
    float filter_size_surf;
    float filter_size_map;
    int point_filter_num;

    bool useImuHeadingInitialization;
    float optimizationStep;
    
public:
    parameter()
    {
        nh.param<string>("ros/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<string>("ros/imuTopic", imuTopic, "imu_correct");
        nh.param<string>("ros/odomTopic", odomTopic, "odometry");
        nh.param<string>("ros/lidarFrame", lidarFrame, "base_link");
        nh.param<string>("ros/baselinkFrame", baselinkFrame, "base_link");
        nh.param<string>("ros/odometryFrame", odometryFrame, "odom");
        nh.param<string>("ros/mapFrame", mapFrame, "map");

        vector<float> v_livoxExtrinsicTrans;
        vector<float> v_livoxExtrinsicRot;
        nh.param<vector<float>>("calibration/livoxExtrinsicTrans",  v_livoxExtrinsicTrans, vector<float>());
        nh.param<vector<float>>("calibration/livoxExtrinsicRot",  v_livoxExtrinsicRot, vector<float>());
        extLivoxToBase.translate = Eigen::Map<Eigen::Vector3f>(v_livoxExtrinsicTrans.data());
        extLivoxToBase.rotate = Eigen::Map<Eigen::Matrix3f>(v_livoxExtrinsicRot.data());

        vector<float> v_imuExtrinsicTrans;
        vector<float> v_imuExtrinsicRot;
        nh.param<vector<float>>("calibration/imuExtrinsicTrans",  v_imuExtrinsicTrans, vector<float>());
        nh.param<vector<float>>("calibration/imuExtrinsicRot",  v_imuExtrinsicRot, vector<float>());
        extImuToBase.translate = Eigen::Map<Eigen::Vector3f>(v_imuExtrinsicTrans.data());
        extImuToBase.rotate = Eigen::Map<Eigen::Matrix3f>(v_imuExtrinsicRot.data());

        nh.param<int>("lidar0/N_SCAN_0", N_SCAN_0, 1);
        nh.param<int>("lidar0/Horizon_SCAN_0", Horizon_SCAN_0, 10000);
        nh.param<float>("lidar0/lidarMinRange_0", lidarMinRange_0, 1.0);
        nh.param<float>("lidar0/lidarMaxRange_0", lidarMaxRange_0, 200.0);

        nh.param<float>("imu/imuAccNoise", imuAccNoise, 0.1);
        nh.param<float>("imu/imuGyrNoise", imuGyrNoise, 0.1);
        nh.param<float>("imu/imuAccBiasN", imuAccBiasN, 0.1);
        nh.param<float>("imu/imuGyrBiasN", imuGyrBiasN, 0.1);
        nh.param<float>("imu/imuGravity", imuGravity, 9.80511);
        nh.param<float>("imu/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<float>("imu/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("imu/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<float>("ivox/ivox_grid_resolution", ivox_grid_resolution, 0.5);
        nh.param<int>("ivox/ivox_nearby_type", ivox_nearby_type, 26);
        nh.param<int>("ivox/ivox_capacity", ivox_capacity, 1000000);
        nh.param<float>("ivox/esti_plane_threshold", esti_plane_threshold, 0.1);
        nh.param<int>("ivox/cube_side_length", cube_side_length, 1000);
        nh.param<float>("ivox/filter_size_surf", filter_size_surf, 0.5);
        nh.param<float>("ivox/filter_size_map", filter_size_map, 0.5);
        nh.param<int>("ivox/point_filter_num", point_filter_num, 3);

        nh.param<bool>("mapping/useImuHeadingInitialization", useImuHeadingInitialization, true);
        nh.param<float>("mapping/optimizationStep", optimizationStep, 0.1);

    }
    virtual ~parameter()
    {

    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        // 添加外参
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extImuToBase.rotate.cast<double>() * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extImuToBase.rotate.cast<double>() * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extImuToBase.rotate.cast<double>();
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }

    /**
     * estimate a plane
     * @tparam T
     * @param pca_result
     * @param point
     * @param threshold
     * @return
     */
    template <typename T>
    inline bool esti_plane(Eigen::Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold = 0.1f) {
        if (point.size() < MIN_NUM_MATCH_POINTS) {
            return false;
        }

        Eigen::Matrix<T, 3, 1> normvec;

        if (point.size() == NUM_MATCH_POINTS) {
            Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
            Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;

            A.setZero();
            b.setOnes();
            b *= -1.0f;

            for (int j = 0; j < NUM_MATCH_POINTS; j++) {
                A(j, 0) = point[j].x;
                A(j, 1) = point[j].y;
                A(j, 2) = point[j].z;
            }

            normvec = A.colPivHouseholderQr().solve(b);
        } else {
            Eigen::MatrixXd A(point.size(), 3);
            Eigen::VectorXd b(point.size(), 1);

            A.setZero();
            b.setOnes();
            b *= -1.0f;

            for (int j = 0; j < point.size(); j++) {
                A(j, 0) = point[j].x;
                A(j, 1) = point[j].y;
                A(j, 2) = point[j].z;
            }

            Eigen::MatrixXd n = A.colPivHouseholderQr().solve(b);
            normvec(0, 0) = n(0, 0);
            normvec(1, 0) = n(1, 0);
            normvec(2, 0) = n(2, 0);
        }

        T n = normvec.norm();
        pca_result(0) = normvec(0) / n;
        pca_result(1) = normvec(1) / n;
        pca_result(2) = normvec(2) / n;
        pca_result(3) = 1.0 / n;

        for (const auto &p : point) {
            Eigen::Matrix<T, 4, 1> temp = p.getVector4fMap();
            temp[3] = 1.0;
            if (fabs(pca_result.dot(temp)) > threshold) {
                return false;
            }
        }
        return true;
    }

    /**
     * squared distance
     * @param p1
     * @param p2
     * @return
     */
    inline float calc_dist(const PointType &p1, const PointType &p2) {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    }

    inline float calc_dist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2) { return (p1 - p2).squaredNorm(); }

};

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    if(thisImuMsg->orientation.x==0 &&
        thisImuMsg->orientation.y==0 &&
        thisImuMsg->orientation.z==0 &&
        thisImuMsg->orientation.w==0 )
    {
        *rosRoll = 0.0;
        *rosPitch = 0.0;
        *rosYaw = 0.0;
        return;
    }
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

template<typename T>
float pointDistance(T p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

template<typename T>
float pointDistance(T p1, T p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

template<typename T>
Eigen::Matrix<T, 3, 3> anti_symmetric(Eigen::Matrix<T, 3, 1> const &_v)
{
    Eigen::Matrix<T, 3, 3> _m;
    _m(0, 0) = 0.0;
    _m(0, 1) = -_v.z();
    _m(0, 2) = _v.y();
    _m(1, 0) = _v.z();
    _m(1, 1) = 0.0;
    _m(1, 2) = -_v.x();
    _m(2, 0) = -_v.y();
    _m(2, 1) = _v.x();
    _m(2, 2) = 0.0;
    return _m;
}

#endif
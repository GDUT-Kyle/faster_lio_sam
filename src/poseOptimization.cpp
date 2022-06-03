#include "faster_lio_sam/common.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "poseOptimization");
    ros::NodeHandle nh;

    ROS_INFO("start poseOptimization");
    ros::spin();
    return 0;
}
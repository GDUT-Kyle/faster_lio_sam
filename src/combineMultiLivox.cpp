#include "faster_lio_sam/common.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "combineMultiLivox");
    ros::NodeHandle nh;

    ROS_INFO("start combineMultiLivox");
    ros::spin();
    return 0;
}
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"
#include "faster_lio_sam/common.h"

ros::Publisher pub_pcl, pub_merge_pcl;
uint64_t TO_MERGE_CNT = 1; 
constexpr bool b_dbg_line = false;
// 创建一个循环队列用于存储雷达帧
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;
void LivoxMsgCbk1(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  livox_data.push_back(livox_msg_in);
  // 第一帧则跳过
  if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointXYZIT> pcl_in;

  // 遍历队列
  for (size_t j = 0; j < livox_data.size(); j++) {
    // 通过引用，方便操作每一帧
    auto& livox_msg = livox_data[j];
    // 获取该帧最后一个点的相对时间
    auto time_end = livox_msg->points.back().offset_time;
    // 重新组织成PCL的点云
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      if(isnan(livox_msg->points[i].x)||isnan(livox_msg->points[i].y)||isnan(livox_msg->points[i].z))
        continue;
      PointXYZIT pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      float s = livox_msg->points[i].offset_time / (float)time_end;
      // 线数——整数，时间偏移——小数
      pt.intensity = 1.0*livox_msg->points[i].reflectivity + s*0.1; // The integer part is line number and the decimal part is timestamp
      pt.time = s*0.1; // 每一点的偏移/秒
      pcl_in.push_back(pt);
    }
  }

  faster_lio_sam::merge_cloud merge_cloud_msg;
  merge_cloud_msg.header = livox_data[0]->header;
  merge_cloud_msg.header.frame_id = "/livox";

  unsigned long timeend_ns = livox_data[0]->header.stamp.toNSec() + livox_data[0]->points.back().offset_time;
  ros::Time timestamp;
  timestamp.fromNSec(timeend_ns);
  merge_cloud_msg.scanEndTime = timestamp.toSec();

  pcl::toROSMsg(pcl_in, merge_cloud_msg.merge_cloud);
  merge_cloud_msg.merge_cloud.header = merge_cloud_msg.header;
  pub_merge_pcl.publish(merge_cloud_msg);
  
  pub_pcl.publish(merge_cloud_msg.merge_cloud);

  livox_data.clear();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 100, LivoxMsgCbk1);

  pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/livox_pcl0", 100);
  
  pub_merge_pcl = nh.advertise<faster_lio_sam::merge_cloud>("/livox_merge_pcl0", 100);

  ros::spin();
}

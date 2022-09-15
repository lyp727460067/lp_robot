#include <ros/ros.h>
#include <signal.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// #include "dev_socket.h"
#include "geometry_msgs/Transform.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_listener.h"
#include "dev_seria.h"
#include "glog/logging.h"
#include "Eigen/Core"
#include <tf/transform_broadcaster.h>

tf::TransformBroadcaster* tf_broadcaster;
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
  auto odom_msg = *msg;
  geometry_msgs::TransformStamped tf_trans;
  tf_trans.header.stamp = ros::Time::now();
  tf_trans.header.frame_id = "map";
  tf_trans.child_frame_id = "base_link";
  tf_trans.transform.translation.x = odom_msg.pose.pose.position.x;
  tf_trans.transform.translation.y = odom_msg.pose.pose.position.y;
  tf_trans.transform.translation.z = 0;
  tf_trans.transform.rotation.x = odom_msg.pose.pose.orientation.x;
  tf_trans.transform.rotation.y = odom_msg.pose.pose.orientation.y;
  tf_trans.transform.rotation.z = odom_msg.pose.pose.orientation.z;
  tf_trans.transform.rotation.w = odom_msg.pose.pose.orientation.w;
  tf_broadcaster->sendTransform(tf_trans);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "pub_odom_tf");
  geometry_msgs::TransformStamped transform;
  nav_msgs::Odometry odom;
  tf_broadcaster = new tf::TransformBroadcaster;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = 1;      //是否打印到控制台
  FLAGS_alsologtostderr = 1;  //打印到日志同时是否打印到控制
  ros::NodeHandle ph;
  auto odom_sub = ph.subscribe<nav_msgs::Odometry>("/odom", 10, odom_cb);
  ros::spin();
  ros::shutdown();
  exit(0);
}

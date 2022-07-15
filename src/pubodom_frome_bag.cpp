#include "ros/ros.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include <sstream>
#include <istream>
#include <fstream>
#include <vector>
#include <tuple>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2_ros/transform_listener.h"

#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <nav_msgs/Path.h>
#include <stdio.h>
#include "nav_msgs/Odometry.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <random>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Path.h>
#include <thread>
DEFINE_string(ground_true_file,"","file of ground true to draw in rviz");
//DEFINE_string(color_of_path,"","file of ground true to draw in rviz");
struct PoseVector {
  double x;
  double y;
  ros::Time time;
};

std::map<std::string, std::vector<PoseVector>> ExtractData(
    std::vector<std::string> files_name) {
  std::map<std::string, std::vector<PoseVector>> result;
  for (auto file : files_name) {
    std::string inputstring;
    rosbag::Bag in_bag;
    in_bag.open(file, rosbag::bagmode::Read);
    rosbag::View view(in_bag);
    rosbag::View::const_iterator view_iterator = view.begin();
    ros::Time begin_time = view_iterator->getTime();
    
    auto &file_pose_vector =result[file];
    for (auto view_iterator = view.begin(); view_iterator != view.end();
         view_iterator++) {
      rosbag::MessageInstance msg = *view_iterator;
      if (msg.isType<nav_msgs::Odometry>()) {
        if (msg.getTopic() != "/odom") {
          geometry_msgs::Point point;
          auto msg_value = msg.instantiate<nav_msgs::Odometry>();
          file_pose_vector.push_back({msg_value->pose.pose.position.x,
                                      msg_value->pose.pose.position.y,
                                      msg_value->header.stamp});
        }
      }
    }
  }
  return result;
}

void PubPoseWithMark(
    ros::NodeHandle &nh,
    const std::map<std::string, std::vector<PoseVector>> &poses) {
  ros::Publisher markpub =
      nh.advertise<visualization_msgs::MarkerArray>("ground_paths", 10);
  visualization_msgs::MarkerArray marks;
  std::default_random_engine e;

  int mark_id = 0;
  for (const auto &pose_with_name : poses) {
    visualization_msgs::Marker mark;
    mark.header.frame_id = "map";

    mark.ns = pose_with_name.first.c_str();
    mark.header.stamp = ::ros::Time::now();
    mark.id = mark_id++;
    mark.action = visualization_msgs::Marker::ADD;
    mark.type = visualization_msgs::Marker::POINTS;
    // mark.type = visualization_msgs::Marker::ARROW;
    mark.lifetime = ros::Duration(0);
    mark.scale.x = 0.1;
    mark.scale.y = 0.1;
    mark.scale.z = 0.1;
    std::uniform_real_distribution<float> ran(0, 1);
    mark.color.r = 1;       // ran(e);//1.0;
    mark.color.a = 1;       // ran(e);
    mark.color.g = ran(e);  //(mark_id / sizeofils);
    mark.color.b = ran(e);  //(sizeofils- mark_id) / sizeofils;
    // LOG(INFO)<<mark.color.g<<mark.color.b;
    int cnt = 0;

    for (const auto &pose : pose_with_name.second) {
      geometry_msgs::Point point;
      point.x = pose.x;
      point.y = pose.y;
      point.z = 0;
      mark.points.push_back(point);
    }
    marks.markers.push_back(mark);
  }
  markpub.publish(marks);
}
  tf::TransformBroadcaster* odom_broadcaster;
void PubMapToOdomTf(ros::NodeHandle &nh) {
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "map";
  odom_trans.child_frame_id = "odom";
  odom_trans.transform.translation.x = 0;
  odom_trans.transform.translation.y = 0;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation.x = 0;
  odom_trans.transform.rotation.y = 0;
  odom_trans.transform.rotation.z = 0;
  odom_trans.transform.rotation.w = 1;
  odom_broadcaster->sendTransform(odom_trans);
}
void PubPoseWithPath(
    ros::NodeHandle &nh,
    const std::map<std::string, std::vector<PoseVector>> &poses) {
  std::map<std::string, ros::Publisher> path_pubs;
  std::map<std::string, nav_msgs::Path> paths;
  int index = 0;
  for (auto pose_with_name : poses) {

    std::cout<<pose_with_name.first<<std::endl;
    path_pubs.insert({pose_with_name.first,
                      nh.advertise<nav_msgs::Path>(
                          "odom_path_" + std::to_string(index), 1, true)});
    paths[pose_with_name.first];
    index++;
  }
  index = 0;
  for (const auto &pose_with_name : poses) {
    paths.at(pose_with_name.first).header.frame_id = "map";
    paths.at(pose_with_name.first).header.stamp  = ros::Time::now();
    for (const auto &pose : pose_with_name.second) {
      geometry_msgs::PoseStamped this_pose_stamped;
      this_pose_stamped.pose = geometry_msgs::Pose();
      // std::cout<<pose.x<<" "<<pose.y<<std::endl;
      this_pose_stamped.pose.position.x = pose.x;
      this_pose_stamped.pose.position.y = pose.y;
      this_pose_stamped.pose.position.z = 0;
      this_pose_stamped.pose.orientation.w =1;
      this_pose_stamped.header.seq = ++index;
      this_pose_stamped.header.stamp = ros::Time::now();
      this_pose_stamped.header.frame_id = "map";
      std::this_thread::sleep_for(std::chrono::microseconds(10));
      paths.at(pose_with_name.first).poses.push_back(this_pose_stamped);
      path_pubs[pose_with_name.first].publish(paths.at(pose_with_name.first));
    }
  }
}



int main(int argc, char **argv) {
  if (argc < 2) {
    return EXIT_FAILURE;
  }
  ros::init(argc, argv, "pub_odom_from_bag");
  odom_broadcaster = new   tf::TransformBroadcaster();
  ros::NodeHandle nh;
  std::vector<std::string> file_name;
  for (int i = 1; i < argc; i++) {
    std::cout << argv[i];
    file_name.emplace_back(argv[i]);
  }

  ros::start();
  // std::thread tf_thread([&]() {
  //   while (1) {
  //     PubMapToOdomTf(nh);
  //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
  //   }
  // });
  auto poses = ExtractData(file_name);
  PubPoseWithPath(nh, poses);
  ros::spin();
  ros::shutdown();
}

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <dirent.h>
#include <sys/types.h>
#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "image_transport/image_transport.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_listener.h"
#include "boost/filesystem.hpp"
namespace {

std::vector<std::string> topics = {"/imu", "/odom",
                                   "/usb_cam/image_raw/compressed",
                                   "/ydlidar/scanOriginal", "/tf"};

}

struct Pose2d {
  double x;
  double y;
  double theta;
};
std::istream& operator>>(std::istream& in, Pose2d& pose) {
  in >> pose.x >> pose.y >> pose.theta;
  return in;
}
bool ReadTf(std::istream& in, std::map<uint64_t, Pose2d>* poses) {
  Pose2d pose;
  while (in.good()) {
    uint64_t time_stamp;
    // std::cout<<"tf time_stamp"<<time_stamp/100<<std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    in >> time_stamp;
    in >> pose;
    (*poses)[time_stamp] = pose;
  }
  in >> std::ws;
  return true;
}
std::map<uint64_t, Pose2d> FindInTimeDru(
    std::map<uint64_t, Pose2d>& poses,
    std::pair<uint64_t, uint64_t> times_dui) {
  auto it_begin = poses.lower_bound(times_dui.first);
  if (it_begin == poses.end() || it_begin->first >= times_dui.second) {
    return {};
  }
  auto it_end = poses.lower_bound(times_dui.second);
  if (it_end == poses.end() || it_end->first > times_dui.second) {
    it_end = it_end--;
  }
  if (it_end == it_begin) {
    return {*it_end};
  }
  return {it_begin, it_end};
}
geometry_msgs::Transform ToPose2d2TransformStamped(const Pose2d& pose) {
  geometry_msgs::Transform transform;
  auto q = tf::Quaternion(0, 0, pose.theta);
  transform.rotation.x = q.getX();
  transform.rotation.y = q.getY();
  transform.rotation.z = q.getZ();
  transform.rotation.w = q.getW();
  transform.translation.x = pose.x;
  transform.translation.y = pose.y;
  transform.translation.z = 0;
  return transform;
}
geometry_msgs::TransformStamped PackToMsg(const Pose2d& pose, uint64_t time) {
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.stamp = ros::Time().fromNSec(time);
  // transform_stamped.header.seq
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "base_link";
  transform_stamped.transform = ToPose2d2TransformStamped(pose);
  return transform_stamped;
}
#define WRITE_2320_FRAME_NAME   
void WriteOriginalMsg(rosbag::Bag& out_bag,
                      const rosbag::MessageInstance& msg) {
  if (msg.isType<nav_msgs::Odometry>()) {
#ifdef WRITE_2320_FRAME_NAME
    if (msg.getTopic() != "/groundtruth_odom") {
      auto no_frame_msg = msg.instantiate<nav_msgs::Odometry>();
      no_frame_msg->header.frame_id = "map";
      no_frame_msg->child_frame_id = "base_link";
      out_bag.write("/odom", msg.getTime(), no_frame_msg);
    } else {
      out_bag.write(msg.getTopic(), msg.getTime(),
                    msg.instantiate<nav_msgs::Odometry>());
    }

#else
    out_bag.write(msg.getTopic(), msg.getTime(),
                  msg.instantiate<nav_msgs::Odometry>());
#endif
  }
  if (msg.isType<sensor_msgs::Imu>()) {
#ifdef WRITE_2320_FRAME_NAME
    auto no_frame_msg = msg.instantiate<sensor_msgs::Imu>();
    no_frame_msg->header.frame_id = "imu_link";
    out_bag.write("/imu", msg.getTime(), no_frame_msg);
#else
    out_bag.write(msg.getTopic(), msg.getTime(),
                  msg.instantiate<sensor_msgs::Imu>());
#endif
  }
  if (msg.isType<sensor_msgs::LaserScan>()) {
#ifdef WRITE_2320_FRAME_NAME
    out_bag.write("/scan", msg.getTime(),
                  msg.instantiate<sensor_msgs::LaserScan>());
#else
    out_bag.write(msg.getTopic(), msg.getTime(),
                  msg.instantiate<sensor_msgs::LaserScan>());
#endif
  }
  if (msg.isType<sensor_msgs::CompressedImage>()) {
    out_bag.write(msg.getTopic(), msg.getTime(),
                  msg.instantiate<sensor_msgs::CompressedImage>());
  }
}

void Run(rosbag::Bag& in_bag, rosbag::Bag& out_bag,const std::istream& in) {
  std::cout << "run start" << std::endl;
  rosbag::View view(in_bag);

  rosbag::View::const_iterator view_iterator = view.begin();
  ros::Time begin_time = view_iterator->getTime();
  std::map<uint64_t, Pose2d> poses;

  // if (!ReadTf(in, &poses)) {
  //   return;
  // }

  // std::cout << "tf.size()= " << poses.size() << std::endl;
  // std::this_thread::sleep_for(std::chrono::seconds(2));
  // std::cout << "start write tf to rosbag" << std::endl;
  // tf2_msgs::TFMessage tf_msg;

  // auto duri_time_tf = FindInTimeDru(poses, {0, begin_time.toNSec()});
  // if (!duri_time_tf.empty()) {
  //   for (auto pose : duri_time_tf) {
  //     tf_msg.transforms.push_back(PackToMsg(pose.second, pose.first));
  //     out_bag.write(topics.back(), ros::Time().fromNSec(pose.first), tf_msg);
  //   }
  // }
  // tf_msg.transforms.clear();
  int count = 0;
  for (auto view_iterator = view.begin(); view_iterator != view.end();
       view_iterator++) {
    rosbag::MessageInstance msg = *view_iterator;
    const ros::Time new_time = msg.getTime();

    // auto duri_time_tf =
    //     FindInTimeDru(poses, {begin_time.toNSec(), new_time.toNSec()});
    // if (!duri_time_tf.empty()) {
    //   for (auto pose : duri_time_tf) {
    //     std::cout << "pose tim :" << pose.first << std::endl;
    //     tf_msg.transforms.push_back(PackToMsg(pose.second, pose.first));
    //     out_bag.write(topics.back(), ros::Time().fromNSec(pose.first), tf_msg);
    //   }
    //   begin_time = new_time;
    // }
    // if(++count/100){
    // std::cout << "ros time :" << new_time.toNSec() << std::endl;
    // }
    WriteOriginalMsg(out_bag, msg);
  }
}

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cerr << "please input bag file and tf.txt";
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "write_tf_to_rosbag");
  ros::start();

  // // //打开指定目录opendir得到目录句柄
  // DIR* dir = opendir(argv[1]);

  // // struct dirent结构体变量，用来存储子目录项
  // struct dirent* entry;

  // //然后通过while循环不断readdir，获取目录中的内容
  // while ((entry = readdir(dir)) != 0) {
  //   //获取该结构体变量的成员函数d_name就得到了待扫描的文件，然后在使用sprintf函数加入文件绝对路径
  //   printf("%s\n", entry->d_name);
  //   char* newpath;
  //   sprintf(newpath, "%s%s", argv[1], entry->d_name);
  //   printf("%s\n", newpath);
  // }
//
  std::set<std::string> fp_set;
  std::string targetPath(argv[1]);
  boost::filesystem::path myPath(targetPath);
  boost::filesystem::directory_iterator endIter;
  for (boost::filesystem::directory_iterator iter(myPath); iter != endIter;
       iter++) {
    if (boost::filesystem::is_directory(*iter)) {
      std::cout << "thiere is dir" << std::endl;
      std::cout << iter->path().string() << std::endl;

    } else {
      std::cout << "is a file" << std::endl;
      std::cout << iter->path().string() << std::endl;
      fp_set.insert(iter->path().string());
    }
  }
  // //
  for (const auto fp : fp_set) {
    std::cout << fp<<std::endl;
  }
  rosbag::Bag out_bag;
  std::string out_bag_filename(argv[2]);
  try {
    out_bag.open(out_bag_filename, rosbag::bagmode::Write);
  } catch (...) {
    std::cerr << "open :" << out_bag_filename << "faied." << std::endl;
    return EXIT_FAILURE;
  }

  for (auto& bag_file : fp_set) {
    // std::string bag_file(argv[1]);
    rosbag::Bag in_bag;
    std::string in_bag_filename(bag_file);
    try {
      in_bag.open(in_bag_filename, rosbag::bagmode::Read);
    } catch (...) {
      std::cerr << "open :" << in_bag_filename << "faied." << std::endl;
      return EXIT_FAILURE;
    }
    Run(in_bag, out_bag, std::ifstream());
    in_bag.close();
  }

  // std::string tf_file(argv[3]);
  // std::ifstream infile(tf_file);
  // if (!infile) {
  //   std::cerr << "open :" << tf_file << "faied." << std::endl;
  //   return EXIT_FAILURE;
  // }


  out_bag.close();
  // infile.close();

  return EXIT_SUCCESS;
}
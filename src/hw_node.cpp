#include <ros/ros.h>
#include <signal.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "dev_socket.h"
#include "geometry_msgs/Transform.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_listener.h"
using namespace lprobot;
using namespace device;

namespace {
std::mutex SendMutex;
std::condition_variable SendCondition;
bool kill_thread = false;
bool SendFlag = false;
ros::Publisher odom_pub;
std::pair<std::string,int> host_ip{"127.0.0.1",8555};

std::atomic<bool> odom_recive(false);
std::ostream& operator<<(std::ostream& out, std::vector<uint8_t>& data) {
  out << "hex ";
  out << std::hex;
  for (auto& d : data) {
    out << "0x" << std::hex << (int)d << " ";
  }
  out << std::endl;
  return out;
}
void termin_out(int sig) {
  ros::shutdown();
  kill_thread = true;
}

}  // namespace
struct HwData {
  float x;
  float y;
  float theta;
  float v;
  float w;
};
struct HwCmd {
  float v;
  float w;
};

HwCmd velocity_msg;
bool velocity_recive_flag = false;
void VelocityCallBack(const geometry_msgs::TwistConstPtr& msg) {
  std::lock_guard<std::mutex> mtx(SendMutex);
  velocity_msg.v = static_cast<float>(msg->linear.x);
  velocity_msg.w = static_cast<float>(msg->angular.z);
  velocity_recive_flag = true;
  //SendCondition.notify_all();
}

void PubHwData(const HwData& hw_data) {
  nav_msgs::Odometry odom_msg;
  odom_msg.pose.pose.position.x = hw_data.x;
  odom_msg.pose.pose.position.y = hw_data.y;
  odom_msg.pose.pose.position.z = 0;
  auto q = tf::Quaternion(0, 0, hw_data.theta * TFSIMD_RADS_PER_DEG);
  odom_msg.pose.pose.orientation.x = q.getX();
  odom_msg.pose.pose.orientation.y = q.getY();
  odom_msg.pose.pose.orientation.z = q.getZ();
  odom_msg.pose.pose.orientation.w = q.getW();
  odom_msg.twist.twist.angular.x = hw_data.v;
  odom_msg.twist.twist.angular.y = hw_data.w;
  odom_msg.twist.twist.angular.x = 0;

  odom_msg.child_frame_id = "base_link";
  odom_msg.header.frame_id = "odom";
  odom_msg.header.stamp = ros::Time::now();
  odom_pub.publish(odom_msg);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "odometry_publisher");

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = 1;      //是否打印到控制台
  FLAGS_alsologtostderr = 1;  //打印到日志同时是否打印到控制
  LOG(INFO) << "lp_hw_main start";
  ros::NodeHandle ph;
  odom_pub = ph.advertise<nav_msgs::Odometry>("/odom", 10);
  ros::Subscriber vel_sub =
      ph.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &VelocityCallBack);

  std::vector<std::unique_ptr<DevInterface>> Device;

  signal(SIGINT, termin_out);
  // signal(SIGTERM, termin_out);
  HwData hw_data;
  try {
    Device.emplace_back(
        new internal::DevSocket(host_ip,[&hw_data](std::vector<uint8_t>&& d) {
          // std::copy(d.begin(), d.end(),
          //          std::ostream_iterator<uint8_t>(std::cout, " "));
          memcpy( (void*)&hw_data, (void*)d.data(),d.size());
          PubHwData(hw_data);
        }));
  } catch (const std::string s) {
    LOG(INFO) << "Devive creat err" << s;
    return EXIT_FAILURE;
  }

  while (!kill_thread) {
    //  std::unique_lock<std::mutex> mtx(SendMutex);
    //  SendCondition.wait(mtx,[](){ return velocity_recive_flag; });

    if (velocity_recive_flag) {
      velocity_recive_flag = false;
      for (auto& dev : Device) {
        std::vector<uint8_t> send_data(8,0);
       // LOG(INFO) << "start pub_recive cmd";
        memcpy((void*)send_data.data(),(void*)&velocity_msg,
               sizeof(velocity_msg));
        LOG(INFO)<<"send_data with v= "<<velocity_msg.v<<" and w = "<<velocity_msg.w;
        dev->tx(send_data);
        send_data.clear();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ros::spinOnce();
  }

  LOG(INFO) << " end hw_node";
  ros::shutdown();
  exit(0);
}

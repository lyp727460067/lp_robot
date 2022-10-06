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
using namespace lprobot;
using namespace device;

#define ENABLE_ODOM_TF


namespace {


std::mutex SendMutex;
std::condition_variable SendCondition;
bool kill_thread = false;
bool SendFlag = false;
ros::Publisher odom_pub;
std::pair<std::string,int> host_ip{"127.0.0.1",8555};

std::atomic<bool> odom_recive{false};
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
struct MowerData {
  uint16_t cmd;
  uint16_t state;
  float l_encode;
  float r_encode;
  float ultra_left_front;
  float ultra_right_front;
  float ultra_back;
};

struct MowerSendData {
  uint16_t cmd;
  float v;
  float w;
};

MowerSendData mower_send_data;
bool velocity_recive_flag = false;
void VelocityCallBack(const geometry_msgs::TwistConstPtr& msg) {
  std::lock_guard<std::mutex> mtx(SendMutex);
  mower_send_data.cmd = 0x01;
  mower_send_data.v = static_cast<float>(msg->linear.x);
  mower_send_data.w = static_cast<float>(msg->angular.z);
  velocity_recive_flag = true;
  // SendCondition.notify_all();
}
 
std::vector<uint8_t> ToUartData(const MowerSendData& data) {
  std::vector<uint8_t> result;
  std::vector<uint8_t> temp{100, 0};
  LOG(INFO)<<data.v;
  memcpy((void*)temp.data(), (void*)&data, sizeof(MowerSendData));
  result.push_back(0x55);
  result.push_back(0xaa);
  char lenth_temp[4];
  int lenth = 6 + sizeof(MowerSendData);
  LOG(INFO) << lenth;
  memcpy((void*)lenth_temp, (void*)&lenth, 4);
  for (int i = 0; i < 4; i++) {
    result.push_back(lenth_temp[i]);
  }
  std::cout<<std::hex;
  for (int i = 0; i < sizeof(MowerSendData); i++) {
    std::cout<<int(temp[i]);
    result.push_back(temp[i]);
  }
  std::cout<<std::endl;
  result.push_back(0x44);
  return result;
}

std::ostream &operator<<(std::ostream& out, const MowerData& data) {
  out << "mower_data: "
      << "\n"
      << "cmd : " << data.cmd << " state: " << data.state << "\n"
      << "encode : " << data.l_encode << " " << data.r_encode << "\n"
      << "utrl: " << data.ultra_left_front << " " << data.ultra_right_front
      << " " << data.ultra_back << std::endl;
  return out;
}


double odom_theta;
constexpr double kWheelLenth = 0.27;
Eigen::Vector2d odom_translation;

nav_msgs::Odometry ToOdomtry(const MowerData& data) {
  static MowerData old_data = data;
  const double l_encode = data.l_encode - old_data.l_encode;
  const double r_encode = data.r_encode - old_data.r_encode;
  old_data = data;
  LOG(INFO)<< l_encode<<" "<<r_encode;
  float odom_v = (r_encode + l_encode) * 0.5f;  // mm/s
  const double angle_delta = (r_encode - l_encode) / ( kWheelLenth);
  const double odom_deltax = odom_v * cos(odom_theta) ;
  const double odom_deltay = odom_v * sin(odom_theta) ;
  odom_theta += angle_delta;
  odom_translation.x() += odom_deltax;
  odom_translation.y() += odom_deltay;

  nav_msgs::Odometry odom_msg;
  odom_msg.pose.pose.position.x = odom_translation.x();
  odom_msg.pose.pose.position.y = odom_translation.y();
  odom_msg.pose.pose.position.z = 0;
  auto q = tf::Quaternion(0, 0, odom_theta);
  odom_msg.pose.pose.orientation.x = q.getX();
  odom_msg.pose.pose.orientation.y = q.getY();
  odom_msg.pose.pose.orientation.z = q.getZ();
  odom_msg.pose.pose.orientation.w = q.getW();
  odom_msg.twist.twist.angular.x = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.x = 0;
  return odom_msg;
}
tf::TransformBroadcaster* tf_broadcaster;
uint32_t mower_data_last_cmd;
void PubMowerData(const MowerData& mower_data) {

  uint32_t  cmd =  mower_data.cmd^mower_data_last_cmd;
  mower_data_last_cmd = mower_data.cmd;

  bool start_area = (cmd & 0x00000040) ? true : false;
  bool finish_area = (cmd & 0x00000080) ? true : false;
  bool save_area = (cmd & 0x00000100) ? true : false;
  bool start_mower = (cmd & 0x00000200) ? true : false;
  bool clean_area = (cmd & 0x00000400) ? true : false;
  bool set_doking = (cmd & 0x00000800) ? true : false;

  nav_msgs::Odometry odom_msg = ToOdomtry(mower_data);
  odom_msg.child_frame_id = "base_link";
  odom_msg.header.frame_id = "odom";
  odom_msg.header.stamp = ros::Time::now();
  odom_pub.publish(odom_msg);

#ifdef ENABLE_ODOM_TF
  geometry_msgs::TransformStamped tf_trans;
  tf_trans.header.stamp = ros::Time::now();
  tf_trans.header.frame_id = "map";
  tf_trans.child_frame_id = "base_link";
  tf_trans.transform.translation.x = odom_msg.pose.pose.position.x;
  tf_trans.transform.translation.y = odom_msg.pose.pose.position.y;
  tf_trans.transform.translation.z = odom_msg.pose.pose.position.z;
  tf_trans.transform.rotation.x = odom_msg.pose.pose.orientation.x;
  tf_trans.transform.rotation.y = odom_msg.pose.pose.orientation.y;
  tf_trans.transform.rotation.z = odom_msg.pose.pose.orientation.z;
  tf_trans.transform.rotation.w = odom_msg.pose.pose.orientation.w;
  tf_broadcaster->sendTransform(tf_trans);
#endif
}
struct RtkData {
  double lat;
  double log;
  double alt;
};
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "odometry_publisher");
   geometry_msgs::TransformStamped transform;
   nav_msgs::Odometry odom;
   odom.pose.pose.position.x  = transform.transform.translation.x;
   odom.pose.pose.position.y  = transform.transform.translation.y;
   odom.pose.pose.position.z  = transform.transform.translation.z;
   odom.pose.pose.orientation.w  = transform.transform.rotation.w;
   tf_broadcaster = new tf::TransformBroadcaster;
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
   MowerData mower_data;
   std::pair<std::string, int> port("/dev/ttyUSB1", 115200);
   try {
     Device.emplace_back(new internal::DevSeriWithDataProcess(
         port, [&mower_data](std::vector<uint8_t>&& d) {
          //  std::cout << std::hex;
           std::copy(d.begin(), d.end(),
                     std::ostream_iterator<int>(std::cout, " "));
           std::cout << std::endl;
           memcpy((void*)&mower_data, (void*)d.data(), d.size());
           PubMowerData(mower_data);
           LOG(INFO) << mower_data;
         },31));
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
        // mower_send_data.cmd = 0x01;
        // mower_send_data.v = 0;
        // mower_send_data.w = 0.3;
        auto send_data = ToUartData(mower_send_data);
        LOG(INFO) << "send_data with v= " << mower_send_data.v
                  << " and w = " << mower_send_data.w;
        dev->tx(send_data);
        // LOG(INFO)<<send_data.size();
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


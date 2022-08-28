#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Path.h"
#include "tf2_ros/transform_listener.h"
#include <thread>
constexpr char kGetGloblePath[] = "FetchGloblePath";
bool HandlegetPath(nav_msgs::GetPlan::Request& reques,
                   nav_msgs::GetPlan::Response& respose) {
  std::string str;
  std::vector<std::vector<double>> points;
  std::cout<<"please input point>>>>>>>>>>>>>>"<<std::endl;
  // if (std::getline(std::cin, str)) {
    // std::string::size_type idex = str.find_first_not_of('[');
    // std::cout<<str<<std::endl;
    // while (idex != std::string::npos && ros::ok()) {
    //   auto end_idex = str.find_first_of(']', idex);
    //   if(end_idex == std::string::npos){
    //     break;
    //   }
    //   std::stringstream ss(str.substr(idex, end_idex-1));
    //   double x, y, th;
    //   ss >> x >> y >> th;
    //   std::cout<<"x="<<x<<"y= "<<y<<"th= "<<th<<std::endl;
    //   idex = str.find_first_not_of('[', end_idex+1);
    //   points.push_back({x, y, th});
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100)) ;
    // }
  points = std::vector<std::vector<double>>{
      {1, 0, 0},          {1.5, 0, 0},      {1.5, 0, 3.14 / 2},
      {1.5, 1, 3.14 / 2}, {1.5, 0.5, 3.14}, {1.5, 0.0, 3.14}};
  for (auto point : points) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    pose.pose.position.z =0; 
    auto q =     tf::Quaternion(0,0,point[2]*TFSIMD_RADS_PER_DEG);
    pose.pose.orientation.x = q.getX();
    pose.pose.orientation.y = q.getY();
    pose.pose.orientation.z = q.getZ();
    pose.pose.orientation.w = q.getW();
    pose.header.frame_id= "base_link";
    pose.header.stamp = ros::Time::now();
    std::cout<<pose.pose.position.x<<pose.pose.position.y<<std::endl;
    respose.plan.poses.push_back(pose);
  }
  return true;
  // }
  // return false;

 }

int main(int argc,char **argv)
{
  ros::init(argc, argv, "navigation_map");
  ros::start();
  
  ros::NodeHandle node_handle("");
  ros::ServiceServer service_servers =
      node_handle.advertiseService(kGetGloblePath, HandlegetPath);

  ros::spin();
}


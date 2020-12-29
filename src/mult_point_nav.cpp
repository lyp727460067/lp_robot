#include <tf/transform_listener.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "actionlib/client/simple_action_client.h"
#include "glog/logging.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/RecoveryStatus.h"
#include "ros/ros.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
using MoveBaseActionClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ;
//
//template<typename Instance>
//class StateInterface
//{
//  public:
//   virtual void Enter(Instance& instance) = 0;
//   virtual void Excute(Instance& instance) = 0;
//   virtual void Eixt(Instance& instance) = 0;
//   ~StateInterface(){}
//};
//
//template<typename Instance>
//class CheckInputState:public StateInterface<Instance>
//{
// public:
//  virtual void Enter(Instance& instance) {
//    LOG(INFO) << "Enter Checkinput state";
//  }
//  virtual void Excute(Instance& instance) {
//    LOG(INFO) << "Enter Checkinput state";
//    std::vector<Eigen::Vector3d> targets;
//    std::string str;
//    LOG(INFO) << "please input target pose";
//    while (std::getline(std::cin, str))
//      ;
//    std::string::size_type start_pos = str.find_first_not_of('[');
//    while (start_pos != std::string::npos && ros::ok()) {
//      std::string::size_type end_pose = str.find_first_of(']');
//      if (end_pose == std::string::npos) {
//        break;
//      }
//      std::stringstream ss(str.substr(start_pos, end_pose - 1));
//      double x, y, z;
//      ss >> x >> y >> z;
//      targets.push_back({x, y, z});
//    }
//    instance.targets = targets;
//  }
//  virtual void Eixt(Instance& instance) {
//    LOG(INFO) << "Eixt Checkinput state";
//  }
//};
std::vector<Eigen::Vector3d> targets_;

void CheckInputTarget() {
  LOG(INFO) << "Enter Checkinput state";
  std::vector<Eigen::Vector3d> targets;
  std::string str;
  LOG(INFO) << "please input target pose";
  while (std::getline(std::cin, str))
    ;
  std::string::size_type start_pos = str.find_first_not_of('[');
  while (start_pos != std::string::npos && ros::ok()) {
    std::string::size_type end_pose = str.find_first_of(']');
    if (end_pose == std::string::npos) {
      break;
    }
    std::stringstream ss(str.substr(start_pos, end_pose - 1));
    double x, y, z;
    ss >> x >> y >> z;
    targets.push_back({x, y, z});
  }
  targets_ = targets;
}
MoveBaseActionClient ac("mult_pose_target",true) ;
void SetMutyTarget()
{
 
  move_base_msgs::MoveBaseGoal goal;
  for (int i = 0; i < targets_.size(); i++) {
    goal.target_pose.pose.position.x = targets_[i].x();
    goal.target_pose.pose.position.y = targets_[i].y();
    goal.target_pose.pose.position.z = 0;
    auto q = tf::Quaternion(0, 0, targets_[i].z() * TFSIMD_RADS_PER_DEG);
    goal.target_pose.pose.orientation.x = q.getX();
    goal.target_pose.pose.orientation.y = q.getY();
    goal.target_pose.pose.orientation.z = q.getZ();
    goal.target_pose.pose.orientation.w = q.getW();
    goal.target_pose.header.stamp = ros::Time::now();
    ROS_INFO("sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("target %d  MOVE  first target success", i);
    } else {
      ROS_INFO("target %dMOVE  first target failer", i);
    }
  }

  targets_.clear();
}

constexpr int CheckInputState = 0;
constexpr int SetMutyTargetstate = 1;
void run() {
  int states = 0;

  while (ros::ok()) {
    switch (states) {
      case CheckInputState:
        CheckInputTarget();
        states = SetMutyTargetstate;
        break;
      case SetMutyTargetstate:
        SetMutyTarget();
        states = CheckInputState;
        break;
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mult_pose_node");
  while (!ac.waitForResult(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  run();
  ros::shutdown();
  exit(0);
}
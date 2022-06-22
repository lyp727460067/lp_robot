#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/Odometry.h"

int main(int argc,char **argv)
{
  ros::init(argc, argv, "rasbag_node");
	ros::NodeHandle nh;
  ros::Publisher markpub =
      nh.advertise<visualization_msgs::MarkerArray>("odom_marks_", 10);

  visualization_msgs::MarkerArray marks;

  std::string inputbag_name(argv[1]);
  visualization_msgs::Marker mark;
  mark.header.frame_id = "map";
  std::string namesring = inputbag_name;

  mark.ns = namesring.c_str();
  mark.header.stamp = ::ros::Time::now();
  mark.id = 0;
  //mark.action = visualization_msgs::Marker::ADD;
  //mark.type = visualization_msgs::Marker::LINE_STRIP;
  mark.type = visualization_msgs::Marker::POINTS;
  // mark.type = visualization_msgs::Marker::ARROW;
  //mark.lifetime = ros::Duration(0);
  mark.scale.x = 0.5;
  mark.scale.y = 0.5;
  mark.scale.z = 0.5;
  //  std::uniform_real_distribution<float> ran(0, 1);
  mark.color.r = 1;   // 1.0;
  mark.color.a = 1;   // ran(e);
  mark.color.g = 0;   //(mark_id / sizeofils);
  mark.color.b = 0;  //(sizeofils- mark_id) / sizeofils;
  // LOG(INFO)<<mark.color.g<<mark.color.b;
  int cnt = 0;
  std::cout<<"read bag"<<std::endl;

	rosbag::Bag in_bag;

  in_bag.open(inputbag_name, rosbag::bagmode::Read);

  rosbag::View view(in_bag);
  rosbag::View::const_iterator view_iterator = view.begin();



  mark.pose.position.x = 0;
  mark.pose.position.y = 0;
  mark.pose.position.z = 0;
  mark.pose.orientation.x = 0.0;
  mark.pose.orientation.y = 0.0;
  mark.pose.orientation.z = 0.0;
  mark.pose.orientation.w = 1.0;
  for (auto view_iterator = view.begin(); view_iterator != view.end();
       view_iterator++) {
    rosbag::MessageInstance msg = *view_iterator;
    if (msg.isType<nav_msgs::Odometry>()) {
      if (msg.getTopic() == "/odometry/gps") {
        nav_msgs::OdometryConstPtr odom_ptr =
            msg.instantiate<nav_msgs::Odometry>();
        geometry_msgs::Point point;
				std::cout<<odom_ptr->pose.pose.orientation.w<<std::endl;
				std::cout<<odom_ptr->pose.pose.orientation.x<<std::endl;
				std::cout<<odom_ptr->pose.pose.orientation.y<<std::endl;
				std::cout<<odom_ptr->pose.pose.orientation.z<<std::endl;
				int x;
				// std::cin>>x;
        point.x = odom_ptr->pose.pose.position.x;
        point.y = odom_ptr->pose.pose.position.y;
        point.z = 0.0;
				if(mark.points.size()==16384){
			 		 marks.markers.push_back(mark);
					 ++mark.id;
					 mark.points.clear();
				}
        mark.points.push_back(point);
      }
    }
  }

  marks.markers.push_back(mark);
  ros::Rate rate(1);
  std::cout<<"pub bag"<<std::endl;

  while (ros::ok()) {
	markpub.publish(marks);

    rate.sleep();
    //ros::spinOnce();
  }
  ros::shutdown();
  return 0;
}


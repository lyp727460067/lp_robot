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
DEFINE_string(ground_true_file,"","file of ground true to draw in rviz");
//DEFINE_string(color_of_path,"","file of ground true to draw in rviz");
void run(std::vector<std::string> files_name)
{
  ros::NodeHandle nh("");
  ros::Publisher markpub =
      nh.advertise<visualization_msgs::MarkerArray>("ground_paths", 10);

  visualization_msgs::MarkerArray marks;

  // mark.color.r = 1;
  // mark.color.g = 0;
  // mark.color.b = 0;
  // mark.color.a = 1;

  // tf::TransformBroadcaster tf_broadcaster;
  tf2_ros::StaticTransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped tf_trans;
  tf_trans.header.stamp = ros::Time::now();
  tf_trans.header.frame_id = "map";
  tf_trans.child_frame_id = "base_link";
  tf_trans.transform.translation.x = 0;
  tf_trans.transform.translation.y = 0;
  tf_trans.transform.translation.z = 0.0;
  tf_trans.transform.rotation.x = 0;
  tf_trans.transform.rotation.y = 0;
  tf_trans.transform.rotation.z = 0;
  tf_trans.transform.rotation.w = 1;

  int sizeofils = files_name.size();
  std::cout<<"sizeof =  "<<sizeofils<<std::endl;
  int mark_id = 0;
  std::default_random_engine  e;
  for (auto file : files_name) {
    std::string inputstring;
    // std_msgs::ColorRGBA  color = {1,1,};
    std::ifstream input_stream(file, std::ios_base::in);
    if(!input_stream){
      std::cout<<"cannot open "<<file<<std::endl;
      continue;
    }

    visualization_msgs::Marker mark;
    mark.header.frame_id = "map";
    std::string namesring = file.substr(file.find_last_of("/")+1,file.find_first_of(".txt"));
 
    mark.ns = namesring.c_str();
    mark.header.stamp = ::ros::Time::now();
    mark.id = mark_id++;
    mark.action = visualization_msgs::Marker::ADD;
    mark.type = visualization_msgs::Marker::POINTS;
    // mark.type = visualization_msgs::Marker::ARROW;
    mark.lifetime = ros::Duration(0);
    mark.scale.x = 0.1;
    mark.scale.y = 0.1;
    mark.scale.z = 0.1;
    std::uniform_real_distribution<float> ran(0,1);
    mark.color.r = 1;// ran(e);//1.0;
    mark.color.a = 1;//ran(e);
    mark.color.g = ran(e);//(mark_id / sizeofils);
    mark.color.b = ran(e);//(sizeofils- mark_id) / sizeofils;
    //LOG(INFO)<<mark.color.g<<mark.color.b;
    int cnt = 0;
    geometry_msgs::Point point;
    std::string line;
    while (std::getline(input_stream, line)) {
      std::istringstream iss(line);
      std::string name;
      int index;

      cnt++;
      int64_t timestape;
      double x, y, z, qw, qx, qy, qz;
      int64_t tmp;
      if (!(iss >> name >> index >> x >> y >> z >> qw >> qx >> qy >> qz >>
            tmp)) {
        break;
      }  // error

      point.x = x;
      point.y = y;
      point.z = z;
      mark.points.push_back(point);

      // path.poses.push_back(pose); 
    }
    // if(cnt%2){
    //    mark.points.push_back(point);
    // }
    input_stream.clear();
    input_stream.close();
    marks.markers.push_back(mark);
  }

  markpub.publish(marks);
  tf_broadcaster.sendTransform(tf_trans);
  ros::Rate rate(1);
  std::cout<<"marks_pub"<<std::endl;
  while(ros::ok()){
    markpub.publish(marks);
    rate.sleep();
    ros::spinOnce();
  }
  //ros::spin();  // check for incoming messages
  ros::shutdown();
}

int main(int argc,char **argv)
{
     //google::InitGoogleLogging(argv[0]);
     //google::ParseCommandLineFlags(&argc,&argv,true);
    // if(FLAGS_ground_true_file.empty()){
    //     google::ShowUsageWithFlagsRestrict(argv[0],"view_ground");
    //     return EXIT_FAILURE;

    // }

    if(argc<2){
      return EXIT_FAILURE;
    }
    std::vector<std::string> file_name;
    for(int i =1;i<argc;i++){
      file_name.emplace_back(argv[i]);
    }
    ros::init(argc,argv,"mutipaht");
    ros::start();

    run(std::move(file_name));
}

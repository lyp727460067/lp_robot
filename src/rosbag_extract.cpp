#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

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
#include <set>
#include "tf2_ros/transform_listener.h"
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sys/stat.h> 
namespace {

}

std::set<std::string> GetTopics(rosbag::View& view)
{
  std::set<std::string> topics;
  for(const auto * connection : view.getConnections()){
   topics.insert(connection->topic) ;
   std::cout<<"input_bag has topic:\n "<<connection->topic<<"\n";
  }
  std::cout<<std::endl;
  return topics;
}

bool WriteOutPutImage(std::string outdir,const rosbag::MessageInstance &msg)
{
  cv_bridge::CvImagePtr image_ptr;
  if(msg.isType<sensor_msgs::CompressedImage>()){
    image_ptr =cv_bridge::toCvCopy(msg.instantiate<sensor_msgs::CompressedImage>());
    cv::Mat  image= image_ptr->image;
    std::string topic2dir =  msg.getTopic();
    for(auto  &c :topic2dir){
      if(c=='/'){
        c = '_';
      }
    }
    outdir += "/" ;
    outdir+=topic2dir.substr(1,topic2dir.size()-1);// msg.getTopic();
    outdir+= "/" ;
    mkdir(outdir.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    std::string outname = outdir +std::to_string(image_ptr->header.stamp.toNSec());
    outname += ".png";
    //outname += image_ptr->encoding;
    std::cout<<outname<<std::endl;
    cv::imwrite(outname,image);
  }
}


bool WriteOutPutImu(std::string outdir,const rosbag::MessageInstance &msg)
{
  cv_bridge::CvImagePtr image_ptr;
  if(msg.isType<sensor_msgs::Imu>()){

  }
}



void run(const rosbag::Bag& input_bag,const std::string& output_dir)
{
  std::cout<<"run start"<<std::endl;
  rosbag::View  view(input_bag);
  rosbag::View::const_iterator view_iterator = view.begin() ;

  double ration = 0;
  for (auto view_iterator = view.begin(); view_iterator != view.end();
       view_iterator++) {
    if (++ration < 30) continue;
    ration = 0;
    rosbag::MessageInstance msg = *view_iterator;
    WriteOutPutImage(output_dir, msg);
  }
}


int main(int argc,char*argv[])
{
  if(argc!=3){
    std::cerr<<"please input bag file and output dirction";
    return EXIT_FAILURE;
  }
  ros::init(argc,argv,"rosbag_extract_node");
  ros::start();

  rosbag::Bag input_bag;
  std::string input_bag_name(argv[1]);
  try{
    input_bag.open(input_bag_name,rosbag::bagmode::Read);
  }catch(...){
    std::cerr<<"open: "<<input_bag_name<<"faied."<<std::endl;
    return EXIT_FAILURE;
  }

  std::string output_dir(argv[2]);
  run(input_bag,output_dir);





}

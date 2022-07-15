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
constexpr int64_t kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64_t;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;
Time FromUniversal(const int64_t ticks) { return Time(Duration(ticks)); }

Time FromRos(const ::ros::Time& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0
  // +0000", exactly 719162 days before the Unix epoch.
  return FromUniversal(
      (time.sec + kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (time.nsec + 50) / 100);  // + 50 to get the rounding correct.
}
int64_t ToUniversal(const Time time) { return time.time_since_epoch().count(); }
std::ostream& operator<<(std::ostream& os, const Time time) {
  os << std::to_string(ToUniversal(time));
  return os;
}

bool WriteOutPutImu(std::string outdir, const rosbag::MessageInstance& msg) {
  if (msg.isType<sensor_msgs::Imu>()) {
    std::string topic2dir = msg.getTopic();
    std::fstream file(outdir, std::ios_base::out | std::ios_base::app);
    auto imu_msg = msg.instantiate<sensor_msgs::Imu>();
    file << FromRos(imu_msg->header.stamp) << " " << imu_msg->angular_velocity.x
         << " " << imu_msg->angular_velocity.y << " "
         << imu_msg->angular_velocity.z << " " << imu_msg->linear_acceleration.x
         << " " << imu_msg->linear_acceleration.y << " "
         << imu_msg->linear_acceleration.z << std::endl;
  }
  return true;
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
    WriteOutPutImu(output_dir + ".imu_data.txt", msg);
    // WriteOutPutImage(output_dir, msg);
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

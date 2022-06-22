#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_listener.h>
#include <sstream>
#include <fstream>
#include "glog/logging.h"
#include "gflags/gflags.h"
DEFINE_string(map_to_base_file,"","use to record carto map to base tf");

void run(std::string &file_name)
{
    ros::NodeHandle ph("");
    tf2_ros::Buffer tf_buffer{ ::ros::Duration(1) };
    tf2_ros::TransformListener listener(tf_buffer, ph, true);
    geometry_msgs::TransformStamped transform;
    std::ofstream outfile(file_name+".local_pos.txt", std::ios_base::out);
    std::ostringstream outstring;
    uint32_t cunt = 0;
    uint64_t oldtimestamp = 0;
    ros::Rate loop_rate(500);
    //tf::Transform map_to_base(tf::Quaternion(tf::Vector3(0,0,1),-TFSIMD_HALF_PI));
    while (ros::ok()) {
      try {
        
        transform = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
        uint64_t timestamp = transform.header.stamp.toNSec();
        if (oldtimestamp != timestamp) {
          oldtimestamp = timestamp;
          tf::Transform car_to_base;

          tf::transformMsgToTF(transform.transform, car_to_base);
          tf::Transform map_to_base(
          tf::Quaternion(tf::Vector3(0, 0, 1), -TFSIMD_HALF_PI));
          map_to_base *= car_to_base;
          double x, y;
          double roll, pitch, yaw;
          x = map_to_base.getOrigin().x();
          y = map_to_base.getOrigin().y();
          tf::Matrix3x3(map_to_base.getRotation()).getRPY(roll, pitch, yaw);
          outstring << timestamp << "," << x << "," << y << "," << yaw << "\n";
          outfile << outstring.str();
          outstring.str("");
          if (++cunt >= 1000) {
            cunt = 0;
            std::cout << timestamp << " " << x << " " << y << " " << yaw
                      << "\n";
          }
        }
        // std::cout<<timestamp<<" "<<x<<" "<<y<<" "<<yaw<<"\n";

      } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
      loop_rate.sleep();
      ros::spinOnce();  // check for incoming messages
    }

    outfile.close();

}

int main(int argc,char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc,&argv,true);
    if(FLAGS_map_to_base_file.empty()){
        google::ShowUsageWithFlagsRestrict(argv[0],"carto_map_to");
        return EXIT_FAILURE;
    }


    ros::init(argc,argv,"carto_map_to");
    ros::start();
    run(FLAGS_map_to_base_file);
    //return 0;
}



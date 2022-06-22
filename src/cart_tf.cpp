#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <nav_msgs/Path.h>
#include <stdio.h>
#include "nav_msgs/Odometry.h"

nav_msgs::Odometry odom_data;
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_data = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cart_tf");

    ros::NodeHandle ph;
    tf::TransformBroadcaster tf_broadcaster;
   // ros::Rate loop_rate(20);

    // tf_trans.header.stamp = ros::Time::now();
    // tf_trans.header.frame_id = "camera_link";
    // tf_trans.child_frame_id =  "camera_accel_frame";
    // tf_trans.transform.translation.x = 0;
    // tf_trans.transform.translation.y = 0;
    // tf_trans.transform.translation.z = 0.0;
    // tf_trans.transform.rotation.x = 0;
    // tf_trans.transform.rotation.y = 0;
    // tf_trans.transform.rotation.z = 0;
    // tf_trans.transform.rotation.w = 1;
    // tf_broadcaster.sendTransform(tf_trans);

    //  tf_trans.header.stamp = ros::Time::now();
    // tf_trans.header.frame_id = "camera_accel_frame";
    // tf_trans.child_frame_id =  "camera_gyro_frame";
    // tf_trans.transform.translation.x = 0;
    // tf_trans.transform.translation.y = 0;
    // tf_trans.transform.translation.z = 0.0;
    // tf_trans.transform.rotation.x = 0;
    // tf_trans.transform.rotation.y = 0;
    // tf_trans.transform.rotation.z = 0;
    // tf_trans.transform.rotation.w = 1;
    // tf_broadcaster.sendTransform(tf_trans);

    // nav_msgs::Path path;
    // path.header.stamp=ros::Time::now();
    // path.header.frame_id="/odom";

    //ros::Time current_time, last_time;
    while (ros::ok())
    {
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
        tf_broadcaster.sendTransform(tf_trans);

        //     current_time = ros::Time::now();
        //    // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        //     geometry_msgs::TransformStamped odom_trans;
        //     odom_trans.header.stamp = current_time;
        //     odom_trans.header.frame_id = "odom";
        //     odom_trans.child_frame_id = "base_link";
        //     odom_trans.transform.translation.x = odom_data.pose.pose.position.x;
        //     odom_trans.transform.translation.y = odom_data.pose.pose.position.y;
        //     odom_trans.transform.translation.z = odom_data.pose.pose.position.z;

        //     odom_trans.transform.rotation = odom_data.pose.pose.orientation;
        //     odom_broadcaster.sendTransform(odom_trans);
        //     odom_trans.transform.translation.x = 0;
        //     odom_trans.transform.translation.y = 0;
        //     odom_trans.transform.translation.z = 0.0;
        //     odom_trans.transform.rotation.x = 0;
        //     odom_trans.transform.rotation.y = 0;
        //     odom_trans.transform.rotation.z = 0;
        //     odom_trans.transform.rotation.w = 1;
        //     odom_trans.header.frame_id = "base_link";
        //     odom_trans.child_frame_id = "imu_link";
        //     odom_broadcaster.sendTransform(odom_trans);

        //     geometry_msgs::PoseStamped this_pose_stamped;
        //     this_pose_stamped.pose.position = odom_data.pose.pose.position;
        //     this_pose_stamped.pose.orientation = odom_data.pose.pose.orientation ;
        //     this_pose_stamped.header.stamp=current_time;
        //     this_pose_stamped.header.frame_id="odom";
        //     path.poses.push_back(this_pose_stamped);
        //     // sprintf(buffer, "%f,%f", this_pose_stamped.pose.position.x,this_pose_stamped.pose.position.y);
        //     // ROS_INFO_STREAM_ONCE(buffer);
        //     // sprintf(buffer, "%f,%f", this_pose_stamped.pose.orientation.x,this_pose_stamped.pose.orientation.y);
        //     // ROS_INFO_STREAM_ONCE(buffer);
        //     last_time = current_time;
        //     path_pub.publish(path);
        // 	ros::spinOnce();               // check for incoming messages
       // loop_rate.sleep();
        //f += 0.04;
    }

    return 0;
}

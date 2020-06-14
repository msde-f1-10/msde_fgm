#include "msde_fgm/fgm.h"

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>
#include <vector>
#include <string.h>

namespace fgm{


    double scan_angle_min;
    double scan_angle_max;
    double scan_angle_increment;
    int scan_range_size;


    FGM::FGM(const ros::NodeHandle h)
    :nh_c(h), loop_rate(100)
    {
        ROS_INFO("start fgm node");

        // get parameters for name of topics
        nh_c.getParam("/msde_driving/topic/scan", scan_topic_name);
        nh_c.getParam("/msde_driving/topic/odom", odom_topic_name);
        nh_c.getParam("/msde_driving/topic/drive", drive_topic_name);

        // get parameters for driving configurations
        nh_c.getParam("/msde_driving/drive/robot_scale", robot_scale);
        nh_c.getParam("/msde_driving/drive/threshold", threshold);
        nh_c.getParam("/msde_driving/drive/filter_size", filter_size);
        nh_c.getParam("/msde_driving/drive/filter_scale", filter_scale);


        sub_init_scan = nh_c.subscribe(scan_topic_name, 1, &FGM::sub_initLidarData, this);
        // subscriber and publisher init
        sub_scan = nh_c.subscribe(scan_topic_name, 10, &FGM::subCallback_scan, this);
        sub_odom = nh_c.subscribe(odom_topic_name, 10, &FGM::subCallback_odom, this);
        pub_ack = nh_c.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic_name, 5);


        // Ackermann Driving message init
        pub_ack_msg.header.stamp = ros::Time::now();
        pub_ack_msg.header.frame_id = "base_link";
        pub_ack_msg.drive.speed = 0;
        pub_ack_msg.drive.acceleration = 0;
        pub_ack_msg.drive.jerk = 0;
        pub_ack_msg.drive.steering_angle = 0;
        pub_ack_msg.drive.steering_angle_velocity = 0;



    }

    FGM::~FGM()
    {
    }
    

    void FGM::start_driving()
    {
        ROS_INFO("start driving");

        while(ros::ok())
        {
            ros::spinOnce();
            // init ackermann message header
            pub_ack_msg.header.stamp = ros::Time::now();
            pub_ack_msg.header.frame_id = "base_link";

            ROS_INFO("angle min: %f, angle max: %f", scan_angle_min, scan_angle_max);
            ROS_INFO("increment : %f", scan_angle_increment);
            ROS_INFO("size of rnages : %d", scan_range_size);


            loop_rate.sleep();
        }
    }

    /* get the laser scan data description */
    void FGM::sub_initLidarData(const sensor_msgs::LaserScan::ConstPtr& msg_sub)
    {
        ROS_INFO("GET THE LIDAR DESCRIPTION");
        scan_angle_min = msg_sub -> angle_min;
        scan_angle_max = msg_sub -> angle_max;
        scan_angle_increment = msg_sub -> angle_increment;
        scan_range_size = msg_sub -> ranges.size();

        sub_init_scan.shutdown();
    }

    void FGM::subCallback_scan(const sensor_msgs::LaserScan::ConstPtr& msg_sub)
    {

    }


    void FGM::subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg_sub)
    {
    }



}


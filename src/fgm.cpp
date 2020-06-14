#include "msde_fgm/fgm.h"

#include "msde_fgm/util.h"

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>
#include <vector>
#include <string.h>

namespace fgm{


    float scan_angle_min;
    float scan_angle_max;
    float scan_angle_increment;
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
        nh_c.getParam("/msde_driving/driving/robot_scale", robot_scale);
        nh_c.getParam("/msde_driving/driving/threshold", threshold);
        nh_c.getParam("/msde_driving/driving/filter_size", filter_size);
        nh_c.getParam("/msde_driving/driving/filter_scale", filter_scale);


        sub_init_scan = nh_c.subscribe(scan_topic_name, 1, &FGM::sub_initLidarData, this);
        // subscriber and publisher init
        sub_scan = nh_c.subscribe(scan_topic_name, 10, &FGM::subCallback_scan, this);
        sub_odom = nh_c.subscribe(odom_topic_name, 10, &FGM::subCallback_odom, this);
        pub_ack = nh_c.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic_name, 5);

        // test publisher
        pub_scan_filtered = nh_c.advertise<sensor_msgs::LaserScan>("filtered_scan", 1);


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
        delete []scan_origin;
        delete []scan_filtered;
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


            std::cout<<std::endl;
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

        // allocate the scan data array
        scan_origin = new float[ scan_range_size ];
        scan_filtered = new float[ scan_range_size ];

        sub_init_scan.shutdown();
    }

    void FGM::subCallback_scan(const sensor_msgs::LaserScan::ConstPtr& msg_sub)
    {
        int i;
        for(i=0; i<scan_range_size; i++){
            scan_origin[i] = msg_sub -> ranges[i];
            scan_filtered[i] = msg_sub -> ranges[i];
        }
        
        // zero filtering
        for(i=0; i<scan_range_size; i++)
        {
            if(scan_origin[i] == 0)
            {
                float sum = 0;
                int count = 0;
                int j;
                for(j=1; j<=20; j++)
                {
                    if(i-j >= 0){
                        if(scan_origin[i-j] != 0){
                            count++;
                            sum += scan_origin[i-j];
                        }
                    }
                    if(i+j < scan_range_size){
                        if(scan_origin[i+j] != 0){
                            count++;
                            sum += scan_origin[i+j];
                        }
                    }
                }
                scan_origin[i] = sum/count;
                scan_filtered[i] = sum/count;
            }
        }// end zero filtering

        // filtering the scan data
        for(i=0; i<scan_range_size-1; i++)
        {
            // right to left
            if(scan_origin[i]*filter_scale < scan_filtered[i+1])
            {
                float unit_length = scan_origin[i] * scan_angle_increment;
                float filter_num = robot_scale / unit_length;
                ROS_INFO("r2l unitle size : %f", unit_length);
                ROS_INFO("r2l filter size : %f", filter_num);
                int j;
                for(j=1; j<filter_num+1; j++)
                {
                    if(i+j < scan_range_size)
                    {
                        if(scan_filtered[i+j] > scan_origin[i]){
                            scan_filtered[i+j] = scan_origin[i];
                        } else {
                            break;
                        }
                    } else { break; }
                }
            }
            // left to right
            else if(scan_filtered[i] > scan_origin[i+1]*filter_scale)
            {
                float unit_length = scan_origin[i+1] * scan_angle_increment;
                float filter_num = robot_scale / unit_length;
                ROS_INFO("l2r unitle size : %f", unit_length);
                ROS_INFO("l2r filter size : %f", filter_num);
                int j;
                for(j=0; j<filter_num+1; j++)
                {
                    if(i-j > 0)
                    {
                        if(scan_filtered[i-j] > scan_origin[i+1]){
                            scan_filtered[i-j] = scan_origin[i+1];
                        } else {
                            break;
                        }
                    } else { break; }
                }
            }

        } // scan filter end

        scan_filter_test.header = msg_sub -> header;
        scan_filter_test.angle_min = scan_angle_min;
        scan_filter_test.angle_max = scan_angle_max;
        scan_filter_test.angle_increment = scan_angle_increment;
        scan_filter_test.time_increment = msg_sub -> time_increment;
        scan_filter_test.scan_time = msg_sub -> scan_time;
        scan_filter_test.range_min = msg_sub -> range_min;
        scan_filter_test.range_max = msg_sub -> range_max;
        scan_filter_test.ranges.clear();
        for(i=0; i<scan_range_size; i++) {
            scan_filter_test.ranges.push_back(scan_filtered[i]);
        }

        pub_scan_filtered.publish(scan_filter_test);


    }


    void FGM::subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg_sub)
    {
    }



}


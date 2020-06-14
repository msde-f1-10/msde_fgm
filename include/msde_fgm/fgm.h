#ifndef __HEADER_fgm_
#define __HEADER_fgm_

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"

#include "msde_fgm/util.h"

#include <string.h>

namespace fgm
{

    extern float scan_angle_min;
    extern float scan_angle_max;
    extern float scan_angle_increment;
    extern int scan_range_size;

    class FGM{
    private:
        ros::NodeHandle nh_c;
        ros::Rate loop_rate;
        ros::Subscriber sub_odom;
        ros::Subscriber sub_scan;
        ros::Publisher pub_ack;
        ros::Subscriber sub_init_scan;

        // parameter for topics
        std::string scan_topic_name;
        std::string odom_topic_name;
        std::string drive_topic_name;

        // parameter for driving
        float robot_scale;
        float threshold;
        float filter_size;
        float filter_scale;


        // Lidar data
        float * scan_origin;
        float * scan_filtered;

        // messages
        ackermann_msgs::AckermannDriveStamped pub_ack_msg;
        sensor_msgs::LaserScan scan_data;
        // message temp
        //sensor_msgs::MultiEchoLaserScan scan_filter_test;
        sensor_msgs::LaserScan scan_filter_test;
        ros::Publisher pub_scan_filtered;

    public:
        FGM(const ros::NodeHandle h);
        ~FGM();
        void start_driving();
        void subCallback_scan(const sensor_msgs::LaserScan::ConstPtr& msg_sub);
        void subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg_sub);
        void sub_initLidarData(const sensor_msgs::LaserScan::ConstPtr& msg_sub);



    };
}



#endif

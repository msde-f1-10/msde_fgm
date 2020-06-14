#ifndef __HEADER_fgm_
#define __HEADER_fgm_

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include "msde_fgm/util.h"

#include <string.h>

namespace fgm
{
    class FGM{
    private:
        ros::NodeHandle nh_c;
        ros::Rate loop_rate;
        ros::Subscriber sub_odom;
        ros::Subscriber sub_scan;
        ros::Publisher pub_ack;

        // parameter for topics
        std::string scan_topic_name;
        std::string odom_topic_name;
        std::string drive_topic_name;

        // parameter for driving
        double robot_scale;
        double threshold;
        double filter_size;
        double filter_scale;

        // messages
        ackermann_msgs::AckermannDriveStamped pub_ack_msg;
        sensor_msgs::LaserScan scan_data;

    public:
        FGM(const ros::NodeHandle h);
        ~FGM();
        void start_driving();
        void subCallback_scan(const sensor_msgs::LaserScan::ConstPtr& msg_sub);
        void subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg_sub);



    };
}



#endif

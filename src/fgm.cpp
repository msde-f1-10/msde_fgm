#include "msde_fgm/fgm.h"

#include "msde_fgm/util_msde.h"

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>
#include <vector>
#include <string.h>

namespace fgm{


    float scan_angle_min=0;
    float scan_angle_max=0;
    float scan_angle_increment=0;
    int scan_range_size=0;
    int range_mid_idx=0;

    // parameter for driving
    float robot_scale=0;
    float threshold=0;
    int gap_size=0;
    float filter_scale=0;


    FGM::FGM(const ros::NodeHandle h)
    :nh_c(h), loop_rate(200)
    {
        ROS_INFO("start fgm node");

        // get reference point
        nh_c.getParam("/sde_driving/reference_point/filepath", path_temp);
        filepath = new char[ path_temp.length()+1 ];
        strcpy(filepath, path_temp.c_str());
        get_reference_point();

        // get parameters for name of topics
        nh_c.getParam("/msde_driving/topic/scan", scan_topic_name);
        nh_c.getParam("/msde_driving/topic/odom", odom_topic_name);
        nh_c.getParam("/msde_driving/topic/nav", drive_topic_name);

        // get parameters for driving configurations
        nh_c.getParam("/msde_driving/driving/robot_scale", robot_scale);
        nh_c.getParam("/msde_driving/driving/threshold", threshold);
        nh_c.getParam("/msde_driving/driving/gap_size", gap_size);
        nh_c.getParam("/msde_driving/driving/filter_scale", filter_scale);
        nh_c.getParam("/msde_driving/driving/speed_max", speed_max);
        nh_c.getParam("/msde_driving/driving/speed_min", speed_min);


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
        delete []filepath;
    }

    void FGM::get_reference_point()
    {
        fs.open(filepath, std::ios::in);
        ROS_INFO("open file");
        util_msde::Point_xy point_temp;

        while(!fs.eof())
        {
            getline(fs, str_buf, ',');
            point_temp.x = std::strtof(str_buf.c_str(), 0);
            getline(fs, str_buf);
            point_temp.y = std::strtof(str_buf.c_str(), 0);
            point_temp.theta = 0;
            rf_points.push_back(point_temp);
        }
        fs.close();
    }
    

    void FGM::start_driving()
    {
        ROS_INFO("start driving");
        GAP gap_obj;

        while(ros::ok())
        {
            ros::spinOnce();
            if(scan_range_size == 0) continue;
            // init ackermann message header
            pub_ack_msg.header.stamp = ros::Time::now();
            pub_ack_msg.header.frame_id = "base_link";

            //find gap
            gap_obj.gap_finding(scan_filtered);
            gap_obj.for_gap_finding(scan_filtered);
            gap_obj.print_gapinfo();

            //test
            Gap goal_gap = gap_obj.get_maximum_gap();
            drive_test(goal_gap);

            std::cout<<std::endl;
            //std::system("clear");
            loop_rate.sleep();
        }
    }


    void FGM::drive_test(Gap goal)
    {
        float steering_angle;
        float angle = (goal.max_idx - range_mid_idx)*scan_angle_increment;
        float mid_angle = ((goal.start_idx + goal.end_idx)/2 - range_mid_idx)*scan_angle_increment;
        float speed = speed_max;

        float distance = 1;
        float path_radius = distance / (2 * sin(angle));
        steering_angle = atan(RACECAR_LENGTH/path_radius);
        


        pub_ack_msg.drive.steering_angle = 0.1*mid_angle + 0.9*angle;
        pub_ack_msg.drive.steering_angle_velocity = 0;
        pub_ack_msg.drive.speed = speed_min;
        pub_ack_msg.drive.acceleration = 0;
        pub_ack_msg.drive.jerk = 0;

        ROS_INFO("speed: %f,  angle: %f", speed, steering_angle);
        pub_ack.publish(pub_ack_msg);

        int i;
        scan_filter_test.ranges.clear();
        for(i=0; i<scan_range_size; i++) {
            if( i > goal.start_idx && i < goal.end_idx)
            {
                scan_filter_test.ranges.push_back(scan_filtered[i]);
            } else {
                scan_filter_test.ranges.push_back(0);
            }
        }
        pub_scan_filtered.publish(scan_filter_test);
    }


    /* get the laser scan data description */
    void FGM::sub_initLidarData(const sensor_msgs::LaserScan::ConstPtr& msg_sub)
    {
        ROS_INFO("GET THE LIDAR DESCRIPTION");
        scan_angle_min = msg_sub -> angle_min;
        scan_angle_max = msg_sub -> angle_max;
        scan_angle_increment = msg_sub -> angle_increment;
        scan_range_size = msg_sub -> ranges.size();
        range_mid_idx = (int)(scan_range_size/2);

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

        /*
        float start_ = range_mid_idx - (PI/2)/scan_angle_increment;
        float end_ = range_mid_idx + (PI/2)/scan_angle_increment;
        for(i=0; i<start_; i++)
        {
            scan_filtered[i] = 0;
        }
        for(i=end_; i<scan_range_size; i++)
        {
            scan_filtered[i] = 0;
        }
        */

        // publish filtered scan data
        scan_filter_test.header = msg_sub -> header;
        scan_filter_test.angle_min = scan_angle_min;
        scan_filter_test.angle_max = scan_angle_max;
        scan_filter_test.angle_increment = scan_angle_increment;
        scan_filter_test.time_increment = msg_sub -> time_increment;
        scan_filter_test.scan_time = msg_sub -> scan_time;
        scan_filter_test.range_min = msg_sub -> range_min;
        scan_filter_test.range_max = msg_sub -> range_max;

    }


    void FGM::subCallback_odom(const nav_msgs::Odometry::ConstPtr& msg_sub)
    {
        this->current_position = util_msde::quanternion2xyt(msg_sub);
        ROS_INFO("current position: (%f, %f)", current_position.x, current_position.y);
    }


    // GAP class
    GAP::GAP()
    {
        this->gaps.reserve(10);

        float theta_for = PI/3;
        int for_point = (int)(theta_for/scan_angle_increment);
        this->for_gap.start_idx = (range_mid_idx) - for_point;
        this->for_gap.end_idx = (range_mid_idx) + for_point;
    }

    GAP::~GAP()
    {
        std::vector<Gap>().swap(gaps);
    }

    void GAP::gap_finding(float* scan)
    {
        // clear previous gaps vector
        this->gaps.clear();
        int i, j;
        for(i=0; i<scan_range_size - gap_size; i++)
        {
            if(scan[i] > threshold)
            {
                float start_idx_temp = i;
                float end_idx_temp =i;
                float max_temp = scan[i];
                float max_idx_temp = i;
                // check is gap or not
                for(j=1; j<gap_size; j++)
                {
                    if(scan[++i] > threshold){
                        if(scan[i] > max_temp){
                            max_temp = scan[i];
                            max_idx_temp = i;
                        }
                    } else {
                        max_temp = -1;
                        break;
                    } 
                }//end checking is gap or not

                // is not gap
                if(max_temp == -1){
                    break;
                }
                // is gap 
                else{
                    while( (scan[i] > threshold) && (++i < scan_range_size) )
                    {
                        if(scan[i] > max_temp){
                            max_temp = scan[i];
                            max_idx_temp = i;
                        }
                    }
                    end_idx_temp = i;
                    // push the gap data
                    Gap gap_temp;
                    gap_temp.start_idx = start_idx_temp;
                    gap_temp.end_idx = end_idx_temp;
                    gap_temp.max_idx = max_idx_temp;
                    this->gaps.push_back(gap_temp);
                }

            }
        }// end gap finding loop
    }// end gap finding method

    void GAP::for_gap_finding(float* scan)
    {
        float theta_for = PI/3;
        int for_point = (int)(theta_for/scan_angle_increment);
        int start_idx_temp = (range_mid_idx) - for_point;
        int end_idx_temp = (range_mid_idx) + for_point;

        /*
        ROS_INFO("%d, %d", start_idx_temp, end_idx_temp);
        ROS_INFO("%d/ %d", range_mid_idx, scan_angle_increment);
        */

        int i;
        int max_idx_temp=start_idx_temp;
        float max_temp;
        max_temp = scan[start_idx_temp];

        for(i=start_idx_temp; i<end_idx_temp; i++)
        {
            if(max_temp < scan[i]){
                max_temp = scan[i];
                max_idx_temp = i;
            }
        }

        this->for_gap.start_idx = start_idx_temp; 
        this->for_gap.end_idx = end_idx_temp;
        this->for_gap.max_idx = max_idx_temp;
    }


    int GAP::get_gapsize()
    {
        return this->gaps.size();
    }

    Gap GAP::get_for_gap()
    {
        return this->for_gap;
    }

    // get maximum gap
    // return for_gap if there is no gap in gaps
    Gap GAP::get_maximum_gap()
    {
        int num = this->gaps.size();
        if( num == 0 ){
            return this->for_gap;
        }
        else{
            int i;
            int max_gap_idx = 0;
            int max_gap_size = this->gaps[0].end_idx - this->gaps[0].start_idx;
            int temp_gap_size = 0;
            for(i=0; i<num; i++)
            {
                temp_gap_size = this->gaps[i].end_idx - this->gaps[i].start_idx;
                if(temp_gap_size > max_gap_size){
                    max_gap_idx = i;
                    max_gap_size = temp_gap_size;
                }
            }
            return this->gaps[max_gap_idx];
        }

    }// end get maximum gpa method

    void GAP::print_gapinfo()
    {
        ROS_INFO("for gap s: %d, e: %d, max: %d", this->for_gap.start_idx, this->for_gap.end_idx, this->for_gap.max_idx);
        int num = this->gaps.size();
        ROS_INFO("number of gap : %d", num);
        int i;
        for(i=0; i<num; i++){
            ROS_INFO("start: %d,  end: %d, maximum: %d", this->gaps[i].start_idx, this->gaps[i].end_idx, this->gaps[i].max_idx);
        }
    }


}


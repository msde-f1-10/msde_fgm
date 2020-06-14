#ifndef __HEADER_fgm_
#define __HEADER_fgm_

#include "nav_msgs/Odometry.h"

#define PI 3.141592


namespace util{

    typedef struct _Point_xy{
        double x;
        double y;
        // direction of the point
        double theta;
    } Point_xy;

    typedef struct _Point_rt{
        double r;
        double theta;
    } Point_rt;


    // chang point expresion method
    Point_xy quanternion2xyt(nav_msgs::Odometry::ConstPtr& odom_data);
    Point_rt quanternion2rt(nav_msgs::Odometry::ConstPtr& odom_data);
    Point_rt xyt2rt(Point_xy original_point);
    Point_xy rt2xyt(double r, double t);



}



#endif

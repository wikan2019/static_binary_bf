#ifndef _STATIC_BINARY_BF_H_
#define _STATIC_BINARY_BF_H_

#endif

#include "special_define.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <math.h>
#include <fstream>

namespace Static_BBF_wjz{

    class wheel_pose2d{
    public:
        wheel_pose2d();
        ~wheel_pose2d();
        R8 x;
        R8 y;
        R8 th;
    };

    class Static_BBF{
    public:
        Static_BBF();
        ~Static_BBF();
        void run();
        void Simulation_Loop();
        void Mapping_Loop();
        void tf_publish_function(R8 time_interval);

    private:
        U4 seq,frame;
        X1 Simulation_Running;
        X1 Mapping_Running;
        R8 time_now;
        wheel_pose2d now_pose;
        ros::NodeHandle node_;
        ros::Time now_time;
        ros::Publisher global_map_pub;
        ros::Publisher local_map_pub;
        ros::Publisher odom_pub;

        nav_msgs::OccupancyGrid local_map;
        nav_msgs::Odometry wheel_odom;
        boost::thread *Simulation_thread;
        boost::thread *Mapping_thread;
        boost::thread *tf_thread;

        boost::mutex local_map_lock;
    };
}

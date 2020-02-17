#include "static_binary_BF.h"
namespace Static_BBF_wjz{
    wheel_pose2d::wheel_pose2d(){
        x  = -10.0;
        y  = -10.0;
        th = 0.0;
    }

    wheel_pose2d::~wheel_pose2d(){}

    Static_BBF::Static_BBF(){
        Simulation_Running = true;
        Mapping_Running    = true;
        frame = 0;
        seq   = 0;
        time_now = ros::Time::now().toSec();
    }

    Static_BBF::~Static_BBF(){}

    void Static_BBF::Simulation_Loop(){

        while(Simulation_Running){
            //1.get pose x+1,y+1;
            now_time = ros::Time::now();
            geometry_msgs::Quaternion th_q = tf::createQuaternionMsgFromYaw(now_pose.th);

            nav_msgs::Odometry odom_now;
            odom_now.header.stamp = ros::Time::now();
            odom_now.header.seq = seq;
            odom_now.header.frame_id = "/wheel_odom";
            odom_now.pose.pose.position.x  = now_pose.x;
            odom_now.pose.pose.position.y  = now_pose.y;
            odom_now.pose.pose.position.z  = 0.0;
            odom_now.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
            odom_now.twist.twist.linear.x  = 0.0;
            odom_now.twist.twist.linear.y  = 0.0;
            odom_now.twist.twist.angular.z = 0.0;
            odom_pub.publish(odom_now);

            //2.create local map from laserscan
            local_map_lock.lock();
            local_map.header.stamp = ros::Time::now();
            local_map.header.seq = seq++;
            local_map.header.frame_id = "/local_map";
            local_map.info.width = 10;
            local_map.info.height = 10;
            local_map.info.resolution = 0.5;
            local_map.info.origin.position.x  = now_pose.x-local_map.info.width*local_map.info.resolution/2;
            local_map.info.origin.position.y  = now_pose.y-local_map.info.height*local_map.info.resolution/2;
            local_map.info.origin.position.z  = 0.0;
            local_map.info.origin.orientation = th_q;
            local_map.data.resize(local_map.info.height*local_map.info.width);
            for(int i=0;i<local_map.info.width;i++){
                for(int j=0;j<local_map.info.height;j++){
                    local_map.data[i*local_map.info.height + j] = i*local_map.info.height+j;
                }
            }
            local_map_pub.publish(local_map);
            local_map_lock.unlock();
            time_now = ros::Time::now().toSec();
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = now_time;
            odom_trans.header.frame_id = "/wheel_odom";
            odom_trans.header.seq = seq;
            odom_trans.child_frame_id = "/base_link";
            odom_trans.transform.translation.x = now_pose.x;
            odom_trans.transform.translation.y = now_pose.y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = th_q;
            tf::TransformBroadcaster odom_broadcaster;
            odom_broadcaster.sendTransform(odom_trans);
            
            while((ros::Time::now().toSec() - time_now)<=0.5){
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }
            now_pose.x += 0.5;
            now_pose.y += 0.5;
            if(now_pose.x >= 500){
                break;
            }
        }
    }

    void Static_BBF::Mapping_Loop(){
        //3.Use static binary BF adding localmap to globalmap
        //3.1 init global_map
        nav_msgs::OccupancyGrid global_map;
        global_map.header.stamp = ros::Time::now();
        global_map.header.frame_id = "/global_map";
        global_map.header.seq = frame++;
        global_map.info.width = 6;
        global_map.info.height = 8;
        global_map.info.resolution = 0.5;
        global_map.info.origin.position.x = 0.0;
        global_map.info.origin.position.y = 0.0;
        global_map.info.origin.position.z = 0.0;
        global_map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
        global_map.data.resize(global_map.info.width*global_map.info.height);
        std::ofstream out;
        out.open("map_save.txt",std::ios::ate);
        out << global_map.info.resolution << " ";
        out << global_map.info.width << " ";
        out << global_map.info.height << " ";
        out << global_map.info.origin.position.x << " ";
        out << global_map.info.origin.position.y << "\n";
        for(int i=0; i<global_map.info.width*global_map.info.height; i++){
            global_map.data[i] = 1;  //unknown
            out << (I4)global_map.data[i] << "\n";
        }
        out.close();
        //3.2 runnning loop
        while(Mapping_Running){
            tf::TransformListener listener;
            tf::StampedTransform trans_listen;
            try{
                listener.waitForTransform("/wheel_odom","/base_link",now_time,ros::Duration(2.0));
                listener.lookupTransform("/wheel_odom","/base_link",now_time,trans_listen);
            }
            catch (tf::TransformException &ER){
                ROS_ERROR("%s",ER.what());
                ROS_ERROR("lookup odom to base_link transform failed!");
                continue;
            }
            R8 x = trans_listen.getOrigin().x();
            R8 y = trans_listen.getOrigin().y();
            tf::Quaternion th = trans_listen.getRotation();
            R8 roll,pitch,yaw;
            tf::Matrix3x3(th).getRPY(roll,pitch,yaw);
            ROS_INFO("get trans:x:%f,y:%f,th:%f",x,y,yaw);

            //4.put local map to global map together
            //as local map is global variable, we assume that we had subscribed it and
            //get completely
            R8 global_x_low  = global_map.info.origin.position.x;
            R8 global_y_low  = global_map.info.origin.position.y;
            R8 global_x_high = global_x_low + global_map.info.width*global_map.info.resolution;
            R8 global_y_high = global_y_low + global_map.info.height*global_map.info.resolution;

            local_map_lock.lock();
            R8 local_x_low   = local_map.info.origin.position.x;
            R8 local_y_low   = local_map.info.origin.position.y;
            R8 local_x_high  = local_x_low + local_map.info.width*local_map.info.resolution;
            R8 local_y_high  = local_y_low + local_map.info.height*local_map.info.resolution;

            R8 min_x_low     = (global_x_low<local_x_low)  ? global_x_low:local_x_low;
            R8 min_y_low     = (global_y_low<local_y_low)  ? global_y_low:local_y_low;
            R8 max_x_high    = (global_x_high>local_x_high)? global_x_high:local_x_high;
            R8 max_y_high    = (global_y_high>local_y_high)? global_y_high:local_y_high;

            global_map.header.stamp = ros::Time::now();
            global_map.header.seq = frame++;
            global_map.header.frame_id = "/global_map";
            global_map.info.width = ceil((max_x_high - min_x_low)/global_map.info.resolution);
            global_map.info.height = ceil((max_y_high - min_y_low)/global_map.info.resolution);
            global_map.info.resolution = 0.5;
            global_map.info.origin.position.x = min_x_low;
            global_map.info.origin.position.y = min_y_low;
            global_map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
            global_map.data.resize(global_map.info.width*global_map.info.height);

            std::ifstream infile;
            infile.open("map_save.txt");
            if(!infile.is_open()){
                ROS_ERROR("open file error");
                continue;
            }
            R8 pre_resolution,o_x,o_y;
            U4 pre_width,pre_height;
            infile >> pre_resolution;
            infile >> pre_width;
            infile >> pre_height;
            infile >> o_x;
            infile >> o_y;

            U4 x_bias = floor((o_x - global_map.info.origin.position.x)/global_map.info.resolution);
            U4 y_bias = floor((o_y - global_map.info.origin.position.y)/global_map.info.resolution);
            //expand map
            B1 in_area;
            for(U4 i = 0;i<global_map.info.height;i++){//y
                for(U4 j = 0;j<global_map.info.width;j++){//x
                    in_area = false;
                    if(i >= y_bias && i< y_bias+pre_height){//y
                        if(j >= x_bias && j < x_bias+pre_width){//x
                            if(infile){
                                I4 tmp;
                                infile >> tmp;
                                global_map.data[i*global_map.info.width + j] = (I4)tmp;
                                in_area = true;
                            }
                        }
                    }
                    if(!in_area){
                        global_map.data[i*global_map.info.width + j] = -1;
                    }
                }
            }
            //mix local and global map
            U4 x_local_bias = floor((local_map.info.origin.position.x - min_x_low)/local_map.info.resolution);
            U4 y_local_bias = floor((local_map.info.origin.position.y - min_y_low)/local_map.info.resolution);
            U4 x_width      = local_map.info.width;
            U4 y_height     = local_map.info.height;
            U4 local_pointer = 0;
            for(U4 i = 0;i<global_map.info.height;i++){//y
                for(U4 j = 0;j<global_map.info.width;j++){//x
                    if(i >= y_local_bias && i< y_local_bias+y_height){//y
                        if(j >= x_local_bias && j < x_local_bias+x_width){//x
                            if(local_pointer < local_map.data.size()){
                                if(global_map.data[i*global_map.info.width + j] == -1){
                                    global_map.data[i*global_map.info.width + j] = local_map.data[local_pointer++];
                                }
                                else if(global_map.data[i*global_map.info.width + j] != -1){
                                    //filter no use += for clearly  = Lt+1 +l + l0
                                    R8 pgx   = global_map.data[i*global_map.info.width + j]/100.0 + 0.001; // global prob - pre
                                    if(local_map.data[local_pointer]== -1){
                                        local_pointer ++;
                                        continue;
                                    }
                                    R8 plx   = local_map.data[local_pointer++]/100.0 + 0.001; //local prob - now
                                    R8 l_now = log(pgx/(1-pgx)) + log(plx/(1-plx));
                                    R8 px    = 1-1/(1+exp(l_now));
                                    ROS_WARN("p:%f",px);
                                    global_map.data[i*global_map.info.width + j] = (U4)(px*100);
                                }
                            }
                        }
                    }

                }
            }

            local_map_lock.unlock();
            infile.close();
            global_map_pub.publish(global_map);

            std::ofstream outl;  //save map
            outl.open("map_save.txt",std::ios::ate);
            outl << global_map.info.resolution << " ";
            outl << global_map.info.width << " ";
            outl << global_map.info.height << " ";
            outl << global_map.info.origin.position.x << " ";
            outl << global_map.info.origin.position.y << "\n";
            for(int i=0; i<global_map.info.width*global_map.info.height; i++){
                outl << (I4)global_map.data[i] << "\n";
            }
            outl.close();
        }
    }

    void Static_BBF::tf_publish_function(R8 time_interval){
        ros::Rate r(1.0 / time_interval);
        while(ros::ok()){
            geometry_msgs::Quaternion th_q = tf::createQuaternionMsgFromYaw(now_pose.th);
            geometry_msgs::TransformStamped local_map_trans;
            local_map_trans.header.stamp = ros::Time::now() + ros::Duration(time_interval);
            local_map_trans.header.seq   = seq;
            local_map_trans.header.frame_id = "/wheel_odom";
            local_map_trans.child_frame_id = "/local_map";
            local_map_trans.transform.translation.x = 0.0; //you had known
            local_map_trans.transform.translation.y = 0.0;
            local_map_trans.transform.translation.z = 0.0;
            local_map_trans.transform.rotation = th_q;
            tf::TransformBroadcaster local_map_broadcaster;
            local_map_broadcaster.sendTransform(local_map_trans);
            geometry_msgs::TransformStamped global_map_trans;
            global_map_trans.header.stamp = ros::Time::now() + ros::Duration(time_interval);
            global_map_trans.header.seq   = seq;
            global_map_trans.header.frame_id = "/wheel_odom";
            global_map_trans.child_frame_id = "/global_map";
            global_map_trans.transform.translation.x = 0.0; //you had known
            global_map_trans.transform.translation.y = 0.0;
            global_map_trans.transform.translation.z = 0.0;
            global_map_trans.transform.rotation = th_q;
            tf::TransformBroadcaster global_map_broadcaster;
            global_map_broadcaster.sendTransform(global_map_trans);
            r.sleep();
        }
    }

    void Static_BBF::run(){
        global_map_pub = node_.advertise<nav_msgs::OccupancyGrid>("/global_map",1,true);
        local_map_pub = node_.advertise<nav_msgs::OccupancyGrid>("/local_map",1,true);
        odom_pub = node_.advertise<nav_msgs::Odometry>("/wheel_odom",10,true);
        Simulation_thread = new boost::thread(&Static_BBF::Simulation_Loop,this);
        Mapping_thread    = new boost::thread(&Static_BBF::Mapping_Loop,this);
        tf_thread = new boost::thread(boost::bind(&Static_BBF::tf_publish_function, this, 0.05));
    }
}
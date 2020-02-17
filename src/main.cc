#include "static_binary_BF.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "static_binary_BF_wjz");
    Static_BBF_wjz::Static_BBF Static_BBF_main;
    Static_BBF_main.run();
    ros::spin();
}

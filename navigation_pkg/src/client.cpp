#include "defines.h"
#include <ros/callback_queue.h>

/**********************MAIN FUNCTION******************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_node");
    ros::NodeHandle nh;
    ros::CallbackQueue my_queue;
    nh.setCallbackQueue(&my_queue);
    initialize_device(argc, argv, nh);

    while(ros::ok())
    {
        my_queue.callAvailable(ros::WallDuration());
        rate->sleep();
    }
    
    /*while(state != quit)
    {
    	ROS_INFO("%d", state);
        ros::spinOnce();
        rate->sleep();
    }*/
    return 0;
}
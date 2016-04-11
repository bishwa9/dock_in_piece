#include "navigation.h"
#include <ros/callback_queue.h>

/**********************MAIN FUNCTION******************************/
int main(int argc, char **argv)
{
    ros::Rate *rate;
    ros::init(argc, argv, "nav_node");
    ros::NodeHandle nh;
    ros::CallbackQueue my_queue;
    nh.setCallbackQueue(&my_queue);
    navigation* nav_obj = new navigation(nh);

    while(ros::ok())
    {
        my_queue.callAvailable(ros::WallDuration());
        rate->sleep();
    }
    return 0;
}
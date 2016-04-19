#ifndef _DEFINE_ST_H_
#define _DEFINE_ST_H_
//ros
#include <ros/ros.h>
//dji
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk_node.h>
//state
//#include <palantir_pkg/palantir_msg.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <palantir_pkg/palantir_msg.h>
#include <time.h>											//to calculate the elapsed time
#include <ctime>
#include <signal.h>											//Ctrl+C
#include <iostream>											//output and inputs
#include <signal.h>

/*****************DEFINES*****************/

//states
#define in_air_state        	3 		//quad state when in air
#define landed_state        	5		//quad state when landed
//times
#define hover_for_min_time    	10 		//hover1 time (before move)

/*****************USER DEFINED TYPES*****************/

//state machine for simple maneuver
typedef enum {  
	init = 0, 
	request_control, 
	takeoff, 
	wait_for_takeoff, 
	hover_for_min,
	stabilize_collect,
	palantir_decision_made,
	approach_dock,
    dock_rejected,
	docked,
	error_activation, 
	error_permission, 
	error_landed,
	quit
} system_state;

/*****************USER DEFINED VARIABLES*****************/

//ros
ros::Rate *rate;

ros::Timer timer;

//DJI
DJIDrone* drone;

ros::Publisher system_st_pub;
ros::Subscriber decision_sub;
ros::Subscriber palantir_sub;

//system state
system_state state;

std::clock_t start_time, end_time;
double elapsed_time;

volatile sig_atomic_t flag = 0;
volatile bool flag_decision, flag_palantir, flag_rejected;

/*************USER DEFINED FUNCTIONS TO PUBLISH INFORMATION*************/

void publish_state()
{
    std_msgs::UInt8 msg;
    msg.data = state;
    system_st_pub.publish(msg);    
}

/*************CALLBACK FUNCTIONS*************/

//Timer callback
void timer_callback(const ros::TimerEvent& event)
{
    publish_state();
}

//Ctrl+C interrupt
void ctrl_c_callback(int sig)
{ // can be called asynchronously
    flag = 1; // set flag
}

void decision_callback(const std_msgs::Float64::ConstPtr& data)
{	
    if(!flag_rejected)
    {
        flag_decision = true;
    }
}

void palantir_callback(const palantir_pkg::palantir_msg::ConstPtr& data)
{
	flag_palantir = true;
    if(data->timeToMove < 0)
    {
        flag_rejected = true;
        flag_decision = false;
    }
}

void initialize_state_node(int argc, char **argv, ros::NodeHandle nh)
{
	//INITIALIZATIONS
    ROS_INFO("STATE STARTED");    

    // Register ctrl+c signal 
    signal(SIGINT, ctrl_c_callback);
    
    timer = nh.createTimer(ros::Duration(0.1), &timer_callback);
    rate = new ros::Rate(30);
    drone = new DJIDrone(nh);

    //publishers
    system_st_pub = nh.advertise<std_msgs::UInt8>("/state",10);

    //subscribers
    decision_sub = nh.subscribe("/decision/prediction", 10, &decision_callback);
    palantir_sub = nh.subscribe("/palantir/prediction", 10, &palantir_callback);

    state = init;
    flag_decision = false;
    flag_palantir = false;
    flag_rejected = false;

    ROS_INFO("State Set-Up complete, Please press Ctrl+C to properly terminate process!");	
}


#endif
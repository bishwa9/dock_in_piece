//ros
#include <ros/ros.h>
//dji
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk_node.h>
//state
#include <std_msgs/UInt8.h>
#include <time.h>											//to calculate the elapsed time
#include <ctime>
#include <signal.h>											//Ctrl+C
#include <iostream>											//output and inputs


/*****************DEFINES*****************/

//states
#define in_air_state        	3 		//quad state when in air
#define landed_state        	5		//quad state when landed
//times
#define hover_for_min_time    	6 		//hover1 time (before move)

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

bool flag_decision, flag_palantir;

/*************USER DEFINED FUNCTIONS TO PUBLISH INFORMATION*************/

void publish_state()
{
    std_msgs::Int8 msg;
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
	flag_decision = true;
}

void palantir_callback(const std_msgs::Float64::ConstPtr& data)
{
	flag_palantir = true;
}


void initialize_state_node(int argc, char **argv, ros::NodeHandle nh)
{
	//INITIALIZATIONS
    ROS_INFO("STATE STARTED");    
    
    timer = nh.createTimer(ros::Duration(0.1), &timer_callback);
    rate = new ros::Rate(30);
    drone = new DJIDrone(nh);

    //publishers
    system_st_pub = nh.advertise<std_msgs::UInt8>("/state",10);

    //subscribers
    decision_sub = nh.subscribe("/decision/prediction", 1, &decision_callback);
    palantir_sub = nh.subscribe("/palantir/prediction", 1, &palantir_callback);

    // Register ctrl+c signal 
    signal(SIGINT, ctrl_c_callback);

    state = init;
    flag_decision = false;
    flag_palantir = false;

    ROS_INFO("State Set-Up complete, Please press Ctrl+C to properly terminate process!");	
}

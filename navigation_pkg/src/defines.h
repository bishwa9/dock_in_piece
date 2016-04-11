#ifndef _DEFINES_H_
#define _DEFINES_H_

/*****************INCLUDES*****************/

//ros
#include <ros/ros.h>
//dji
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk_node.h>
//messages
#include <dji_sdk/Velocity.h>								//for current velocities
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>								//read floats from cv node
#include <sensor_msgs/LaserScan.h> 							//obstacle distance
#include <navigation_pkg/calPos.h>							//publish calculated position
#include <navigation_pkg/prev_velocity.h>					//publish previous velocity
#include <navigation_pkg/times_used.h>						//publish time?
#include <navigation_pkg/command_velocity.h>				//publish command velocity
//normal
#include <time.h>											//to calculate the elapsed time
#include <ctime>
											//Ctrl+C
#include <iostream>											//output and inputs
//PID code
#include "pid.h"											//user defined PID class

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

typedef struct {
	float velX;
	float velY;
	float velZ;
	float dt;
} vel_cmd_type;

/*****************DEFINES*****************/
//PID cosntsatns and min/max outputs
#define KP_x					1.0
#define KP_z					0.5
#define KI_x					0.0
#define KD_x					0.0
#define max_x					1
#define min_x					-1
#define max_z					0.5
#define min_z					-0.3
//number of cameras on Guidance Sensor package
#define NUM_GUIDANCE_CAMERAS	5
//Tolerances
#define MIN_DIST			 	0   	//distance from wall
#define ALLOWED_ERROR			0.01 	//meters (10cm)
//Conversion
#define cmTom					0.01
//Offset
#define Offset 					0.5

/*****************USER DEFINED VARIABLES*****************/

//ros
ros::Rate *rate;

ros::Timer timer;

//DJI
DJIDrone* drone;

//PID VARIABLES
PID pid_x(max_x, min_x, KP_x, KD_x, KI_x);
PID pid_y(max_x, min_x, KP_x, KD_x, KI_x);
PID pid_z(max_z, min_z, KP_z, KD_x, KI_x);

//determine if no obstacle is present
bool isClear = true;
volatile system_state state;

//Publishers
ros::Publisher clear_pub;
ros::Publisher calPos_pub;
ros::Publisher desPos_pub;
ros::Publisher prev_velocity_pub;
ros::Publisher times_used_pub;
ros::Publisher command_velocity_pub;

//Subscribers
ros::Subscriber x_sub, y_sub, z_sub;		//subscribes to CV node
ros::Subscriber cur_vel_sub;		//subscribe to current velocity from DJI_node
ros::Subscriber obst_dist_sub;		//obstacle distance
ros::Subscriber st_sub;
ros::Subscriber decision_sub;

/*****************USER DEFINED CALLBACK VARIABLES*****************/

//x and y desired requested
volatile float z_init, z_set, z_min, approach_vel;
volatile float x_des, y_des; 		//updated in interrupt
float x_toGo, y_toGo, z_toGo; 	     		//registered once moving from hover1 to move states

//position update using velocity
volatile float x_cur, y_cur, z_cur, vx_prev, vy_prev,vz_prev;
volatile std::clock_t vel_last_time;
volatile float vel_e_time;
volatile bool z_found;

volatile vel_cmd_type vel_command;

/*************USER DEFINED FUNCTIONS TO PUBLISH INFORMATION*************/

void publish_clearance()
{
	std_msgs::Int8 msg;
    msg.data = (isClear) ? 1:0;
    clear_pub.publish(msg);
}

void publish_calPos(float x, float y, float z)
{	
	navigation_pkg::calPos msg;
	msg.header.frame_id = "navigation_node";
	msg.header.stamp    = ros::Time::now();
	msg.approx_x = x;
	msg.approx_y = y;
	msg.approx_z = z;
	calPos_pub.publish(msg);
}

void publish_desPos(float x, float y, float z)
{	
	navigation_pkg::calPos msg;
	msg.header.frame_id = "navigation_node";
	msg.header.stamp    = ros::Time::now();
	msg.approx_x = x;
	msg.approx_y = y;
	msg.approx_z = z;
	desPos_pub.publish(msg);
}

void publish_prev_velocity(float vx, float vy)
{	
	navigation_pkg::prev_velocity msg;
	msg.header.frame_id = "navigation_node";
	msg.header.stamp    = ros::Time::now();
	msg.vx_prev = vx;
	msg.vy_prev = vy;
	prev_velocity_pub.publish(msg);
}

void publish_times_used(double last_time)
{
	navigation_pkg::times_used msg;
	msg.header.frame_id = "navigation_node";
	msg.header.stamp    = ros::Time::now();
	msg.elapsed_time = (float)last_time;
	times_used_pub.publish(msg);
}

void publish_command_velocity(float x, float x_dt, float y, float y_dt, float z, float z_dt)
{	
	navigation_pkg::command_velocity msg;
	msg.header.frame_id = "navigation_node";
	msg.header.stamp    = ros::Time::now();
	msg.cmd_vel_x = x; 
	msg.x_PID_dt = x_dt;
	msg.cmd_vel_y = y;
	msg.y_PID_dt = y_dt;
	msg.cmd_vel_z = z;
	msg.z_PID_dt = z_dt;
	command_velocity_pub.publish(msg);
}

/*************USER DEFINED FUNCTIONS*************/
float calcDistance(float vPrev, float dPrev, float time_)
{
    float incDist = vPrev*time_;
    return dPrev + incDist;     
}

/*************CALLBACK FUNCTIONS*************/

void velPub_callback(const ros::TimerEvent& not_used)
{
	publish_command_velocity(vel_command.velX, vel_command.dt,
							 vel_command.velY, vel_command.dt,
							 vel_command.velZ, vel_command.dt);
   	drone->attitude_control(HORIZ_VEL|VERT_VEL|YAW_ANG|HORIZ_GND|YAW_GND, 
   								vel_command.velX, vel_command.velY, vel_command.velZ, 0);
}

void state_callback(const std_msgs::UInt8::ConstPtr& state_)
{
	state = (system_state)state_->data;
}

void obst_dist_callback(const sensor_msgs::LaserScan::ConstPtr& obst_dist_)
{
	isClear = true;
	for(int i = 1; i < NUM_GUIDANCE_CAMERAS; i++)
	{
		isClear = isClear && (obst_dist_->ranges[i] > MIN_DIST); 
	}
	publish_clearance();
}

void vel_callback(const dji_sdk::Velocity::ConstPtr& vel_)
{
	float cmd_vel_x = 0;
	float cmd_vel_y = 0;
	float cmd_vel_z = 0;
	PID_OUTPUT_TYPE PID_cmd_x, PID_cmd_y, PID_cmd_z;

	//update dist
	std::clock_t vel_now_time, last_time;
    vel_now_time = std::clock();
    vel_e_time = float(vel_now_time - vel_last_time) / CLOCKS_PER_SEC; 	//seconds
    //publish_times_used(10*vel_e_time);

	x_cur = calcDistance(vx_prev, x_cur, vel_e_time);		//update x
    y_cur = calcDistance(vy_prev, y_cur, vel_e_time);		//update y
    z_cur = (drone->local_position).z;		//update z
    publish_calPos(x_cur, y_cur, z_cur);
    publish_desPos(x_toGo, y_toGo, z_toGo);

    //update velocities
    vx_prev = (drone->velocity).vx;
    vy_prev = (drone->velocity).vy;
    //vz_prev = -(drone->local_position).z;
    //publish_prev_velocity(vx_prev,vy_prev);
                    
    //update time
    vel_last_time = std::clock();

    //using updated distance and pid what's next velocity command
    PID_cmd_x = pid_x.calculate(x_toGo, x_cur);
    PID_cmd_y = pid_y.calculate(y_toGo, y_cur);
    PID_cmd_z = pid_z.calculate(z_toGo, z_cur);
    //PID_cmd_z = pid_z.calculate(z_cur, z_cur);

    cmd_vel_x = (isClear) ? PID_cmd_x.PID_out : 0;
    cmd_vel_y = (isClear) ? PID_cmd_y.PID_out : 0;
    cmd_vel_z = (isClear) ? 
    			( (state == hover_for_min) ? 0 : 
    				(state == approach_dock) ? approach_vel : PID_cmd_z.PID_out ) : 
    			0;

    //ROS_INFO("x_calc = %f, vx = %f, time = %f", x_cur, cmd_vel_x, 10*vel_e_time);

    if(state >= hover_for_min && z_found)
    {
    	vel_command.velX = cmd_vel_x;
    	vel_command.velY = cmd_vel_y;
    	vel_command.velZ = cmd_vel_z;
    	vel_command.dt = PID_cmd_x.PID_dt;
  //   	publish_command_velocity(cmd_vel_x, PID_cmd_x.PID_dt, cmd_vel_y, PID_cmd_y.PID_dt, cmd_vel_z, PID_cmd_z.PID_dt);
  //   	bool not_done = !drone->attitude_control(HORIZ_VEL|VERT_VEL|YAW_ANG|HORIZ_GND|YAW_GND, cmd_vel_x, cmd_vel_y, cmd_vel_z, 0);
		// std::cout << "service result" << ( (not_done) ? "0.0" : "1.0" ) << std::endl;
    	// bool not_done = true;
    	// while(not_done)
    	// {
    	// 	not_done = !drone->attitude_control(HORIZ_VEL|VERT_VEL|YAW_ANG|HORIZ_GND|YAW_GND, cmd_vel_x, cmd_vel_y, cmd_vel_z, 0);
    	// 	ROS_INFO("service result = %f", (not_done) ? 0.0 : 1.0 );
    	// }
    }
}

void x_callback(const std_msgs::Float64::ConstPtr& x_)
{
	if( state >= hover_for_min )
	{
		x_toGo = (x_->data*cmTom) + x_cur;
	}
	
}

void y_callback(const std_msgs::Float64::ConstPtr& y_)
{
	if( state >= hover_for_min )
	{
		y_toGo = (y_->data*cmTom) + y_cur;
	}
}

void z_callback(const std_msgs::Float64::ConstPtr& z_)
{
	float z_fromAT = -(z_->data*cmTom);
	float dist_to_maintain = 0.60;

	if( (state == hover_for_min && z_fromAT < z_min) || (!z_found && z_fromAT < z_min) )
	{
		z_min = z_fromAT;
		z_set = z_min - dist_to_maintain;
		z_init = z_cur;
		z_found = true;
	}

	if( state == stabilize_collect && z_found)
	{
		//z_toGo = z_set - (z_cur - z_init);
		z_toGo = z_init+z_set;
	} 	
	ROS_INFO("z_min: %f, z_set: %f, z_cur: %f, z_init: %f, z_toGo: %f",z_min, z_set, z_cur, z_init, z_toGo);
}

void decision_callback(const std_msgs::Float64::ConstPtr& dec_)
{
	approach_vel = dec_->data;
}


/*************USER DEFINED FUNCTION TO INITIALIZE ALL VARIABLE*************/

void initialize_device(int argc, char **argv, ros::NodeHandle nh)
{
	//INITIALIZATIONS
    ROS_INFO("NAVIGATION STARTED");  
    std::cout << "NAVIGATION STARTED" << std::endl;  
    
    rate = new ros::Rate(100);
    drone = new DJIDrone(nh);    

    //publishers
    clear_pub = nh.advertise<std_msgs::Int8>("/nav_node/isclear", 10);
    calPos_pub = nh.advertise<navigation_pkg::calPos>("/nav_node/Calculated_pos", 10);
    desPos_pub = nh.advertise<navigation_pkg::calPos>("/nav_node/Desired_pos", 10);
    //prev_velocity_pub = nh.advertaise<navigation_pkg::prev_velocity>("/nav_node/prev_velocity", 10);
    times_used_pub = nh.advertise<navigation_pkg::times_used>("/nav_node/times_used_pos", 10);
    command_velocity_pub = nh.advertise<navigation_pkg::command_velocity>("/nav_node/command_velocity", 10);
    
    //subscribers
    cur_vel_sub = nh.subscribe("/dji_sdk/velocity", 1, &vel_callback);
    obst_dist_sub = nh.subscribe("/guidance/obstacle_distance", 1, &obst_dist_callback);
    x_sub = nh.subscribe("/move_Xdist", 1, &x_callback);
    y_sub = nh.subscribe("/move_Ydist", 1, &y_callback);
    z_sub = nh.subscribe("/move_Zdist", 1, &z_callback);
    st_sub = nh.subscribe("/state", 1, &state_callback);
    decision_sub = nh.subscribe("/decision/prediction", 1, &decision_callback);
    timer = nh.createTimer(ros::Duration(0.015), velPub_callback);

    state = init;

    x_toGo = 0.0;
    y_toGo = 0.0;
    z_toGo = 1.2;
    z_min = 10000.0;
    approach_vel = 0;
    z_found = false;
	vel_last_time = std::clock();

	vel_command.velX = 0;
	vel_command.velY = 0;
	vel_command.velZ = 0;
	vel_command.dt = 0;

    ROS_INFO("Set-Up complete, Please press Ctrl+C to properly terminate process!");
}

#endif


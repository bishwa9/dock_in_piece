#ifndef _DEFINES_H_
#define _DEFINES_H_

/*****************INCLUDES*****************/
//ros
#include <ros/ros.h>
//dji
#include <dji_sdk/dji_drone.h>
//messages
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>								//read floats from cv node
#include <sensor_msgs/LaserScan.h> 							//obstacle distance
#include <navigation_pkg/calPos.h>							//publish calculated position
#include <navigation_pkg/command_velocity.h>				//publish command velocity
//normal
#include <time.h>											//to calculate the elapsed time
#include <ctime>
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

#endif


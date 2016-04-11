#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include "defines.h"

class navigation
{
private:
	//Publishers
	ros::Publisher clear_pub;
	ros::Publisher calPos_pub;
	ros::Publisher desPos_pub;
	ros::Publisher command_velocity_pub;

	//Subscribers
	ros::Subscriber x_sub, y_sub, z_sub;		//subscribes to CV node
	ros::Subscriber cur_vel_sub;				//subscribe to current velocity from DJI_node
	ros::Subscriber obst_dist_sub;				//obstacle distance
	ros::Subscriber st_sub;
	ros::Subscriber decision_sub;

	//Timer
	ros::Timer timer;

	//PID Objects for x,y,z position control
	PID pid_x(max_x, min_x, KP_x, KD_x, KI_x);
	PID pid_y(max_x, min_x, KP_x, KD_x, KI_x);
	PID pid_z(max_z, min_z, KP_z, KD_x, KI_x);

	//internal state variables
	//DJI Object
	DJIDrone* drone;

	//callback variables
	volatile bool isClear;
	//State
	volatile system_state state;
	//velocity calculations
	volatile float z_init, z_set, z_min, z_cur, vz_prev;
	volatile float x_cur, x_des, vx_prev;
	volatile float y_cur, y_des, vy_prev;
	volatile std::clock_t vel_last_time;
	volatile float vel_e_time;
	volatile float approach_vel;
	volatile bool z_found;

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

	/*************TIMER CALLBACK FUNCTIONS*************/

	void velCalc_callback(const ros::TimerEvent& not_used)
	{
		float cmd_vel_x = 0;
		float cmd_vel_y = 0;
		float cmd_vel_z = 0;
		PID_OUTPUT_TYPE PID_cmd_x, PID_cmd_y, PID_cmd_z;

		//update dist
		std::clock_t vel_now_time, last_time;
	    vel_now_time = std::clock();
	    vel_e_time = float(vel_now_time - vel_last_time) / CLOCKS_PER_SEC; 	//seconds

		x_cur = calcDistance(vx_prev, x_cur, vel_e_time);		//update x
	    y_cur = calcDistance(vy_prev, y_cur, vel_e_time);		//update y
	    z_cur = (drone->local_position).z;						//update z
	    publish_calPos(x_cur, y_cur, z_cur);
	    publish_desPos(x_toGo, y_toGo, z_toGo);

	    //update velocities
	    vx_prev = (drone->velocity).vx;
	    vy_prev = (drone->velocity).vy;
	                    
	    //update time
	    vel_last_time = std::clock();

	    //using updated distance and pid what's next velocity command
	    PID_cmd_x = pid_x.calculate(x_toGo, x_cur);
	    PID_cmd_y = pid_y.calculate(y_toGo, y_cur);
	    PID_cmd_z = pid_z.calculate(z_toGo, z_cur);

	    cmd_vel_x = (isClear) ? 
	    			( (state == approach_dock) ? 0 : PID_cmd_x.PID_out ) : 
	    			0;
	    cmd_vel_y = (isClear) ? 
	    			( (state == approach_dock) ? 0 : PID_cmd_y.PID_out ) : 
	    			0;
	    cmd_vel_z = (isClear) ? 
	    			( (state == approach_dock) ? approach_vel : PID_cmd_z.PID_out ) : 
	    			0;

	    if(state >= stabilize_collect && z_found)
	    {
	    	publish_command_velocity(cmd_vel_x, PID_cmd_x.PID_dt, cmd_vel_y, PID_cmd_y.PID_dt, cmd_vel_z, PID_cmd_z.PID_dt);
	    	bool not_done = !drone->attitude_control(HORIZ_VEL|VERT_VEL|YAW_ANG|HORIZ_GND|YAW_GND, cmd_vel_x, cmd_vel_y, cmd_vel_z, 0);
			// std::cout << "service result" << ( (not_done) ? "0.0" : "1.0" ) << std::endl;
	    }
	}

	/*************SUBSCRIBER CALLBACK FUNCTIONS*************/

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
			z_toGo = z_init+z_set;
		}
		//ROS_INFO("z_min: %f, z_set: %f, z_cur: %f, z_init: %f, z_toGo: %f",z_min, z_set, z_cur, z_init, z_toGo);
	}

	void decision_callback(const std_msgs::Float64::ConstPtr& dec_)
	{
		approach_vel = dec_->data;
	}

	/*************INITIALIZATION FUNCTIONS*************/

	void init_Subscribers(ros::NodeHandle& nh)
	{
    	obst_dist_sub = nh.subscribe("/guidance/obstacle_distance", 1, &obst_dist_callback);
    	x_sub = nh.subscribe("/move_Xdist", 1, &x_callback);
    	y_sub = nh.subscribe("/move_Ydist", 1, &y_callback);
    	z_sub = nh.subscribe("/move_Zdist", 1, &z_callback);
    	st_sub = nh.subscribe("/state", 1, &state_callback);
    	decision_sub = nh.subscribe("/decision/prediction", 1, &decision_callback);
	}

	void init_timer(ros::NodeHandle& nh)
	{
		timer = nh.createTimer(ros::Duration(0.015), velCalc_callback);
	}

	void init_Publishers(ros::NodeHandle& nh)
	{
		clear_pub = nh.advertise<std_msgs::Int8>("/nav_node/isclear", 10);
	    calPos_pub = nh.advertise<navigation_pkg::calPos>("/nav_node/Calculated_pos", 10);
	    desPos_pub = nh.advertise<navigation_pkg::calPos>("/nav_node/Desired_pos", 10);
	    command_velocity_pub = nh.advertise<navigation_pkg::command_velocity>("/nav_node/command_velocity", 10);
	}

	void init_variables(ros::NodeHandle& nh)
	{
		drone = new DJIDrone(nh); 

		state = init;

		isClear = true;

	    x_toGo = 0.0;
	    y_toGo = 0.0;
	    z_toGo = 0.0;
	    z_min = 10000.0;
	    approach_vel = 0;
	    z_found = false;
		vel_last_time = std::clock();
	}

public:
	navigation(ros::NodeHandle& nh)
	{
		init_variables(nh);
		init_Subscribers(nh);
		init_Publishers(nh);
		init_timer(nh);
		ROS_INFO("Set-Up complete, Please press Ctrl+C to properly terminate process!");
	}
};

#endif
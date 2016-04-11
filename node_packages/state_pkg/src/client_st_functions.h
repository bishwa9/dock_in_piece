#ifndef _CLIENT_ST_FUNCTIONS_H_
#define _CLIENT_ST_FUNCTIONS_H_

#include "defines_st.h"

/**
*   Purpose: To see if the quadcopter has reach atleast 1 m.
*   INPUT:      N/A
*   OUPTUT:     True if quadcopter > 1m off ground, False otherwise
**/
bool takeoff_done()
{
	return (drone->local_position).z > 1;
}

/**
*   Purpose: Activation has been lost, change state
*   INPUT:      N/A
*   OUPTUT:     N/A
**/
void handle_err_activation()
{
	state = error_activation;
	//publish_state();
}

/**
*   Purpose: N1 flight controller is no longer being commanded by Onboard device (lost permission)
*   INPUT:      N/A
*   OUPTUT:     N/A
**/
void handle_err_permission()
{
	state = error_permission;
	//publish_state();
}

/**
*   Purpose: Unexpectedly the quadcopter's state changed to landed
*   INPUT:      N/A
*   OUPTUT:     N/A
**/
void handle_err_landed()
{
	state = error_landed;
	//publish_state();	
}

/**************************************************************************/
/**
*   Purpose: Function to handle the appropriate states
*   INPUT:      N/A
*   OUPTUT:     N/A
**/
void handle_init()
{
	// make sure the N1 has been properly activated
	if(drone->activation)
	{
		state = request_control;
		ROS_INFO("Requesting Control");
		//publish_state();
	}
}

void handle_request_control()
{
	if(!drone->activation)
    {//lost activation
    	handle_err_activation();
    	return;
    }
    drone->request_sdk_permission_control();
    if(drone->sdk_permission_opened)
    {
    	int dummy;
    	state = takeoff;
        ROS_INFO("Reset controller and enter a number and hit enter");
        std::cin>>dummy;
        ROS_INFO("Take Off");
    	//publish_state();
    	return;
    }
}

void handle_takeoff()
{
	if(!drone->activation)
    {//lost activation
    	handle_err_activation();
    	return;
    }
    if(!drone->sdk_permission_opened)
    {//lost permission
    	handle_err_permission();
    	return;
    }
    drone->takeoff();
    state = wait_for_takeoff;
    ROS_INFO("Waiting for Take Off");
    //publish_state();
}

void handle_wait_for_takeoff()
{
	if(!drone->activation)
    {//lost activation
    	handle_err_activation();
     	return;
    }
    if(!drone->sdk_permission_opened)
    {//lost permission
    	handle_err_permission();
    	return;
    }
    if( takeoff_done() )
    {
    	state = hover_for_min;

        //request for destination
    	/*publish_state();
        float x_, y_;
        std::cout << "Please Enter desired X offset: ";
        std::cin >> x_;
        std::cout << std::endl;
        std::cout << "Please Enter desired Y offset: ";
        std::cin >> y_;
        std::cout << std::endl; 
        x_des = x_; y_des = y_;  */

        time(&start_time);
        ROS_INFO("Hovering for min"); 
        elapsed_time = 0;                
    }
}

void handle_hover_for_min()
{
	if(!drone->activation)
    {//lost activation
    	handle_err_activation();
    	return;
    }
    if(!drone->sdk_permission_opened)
    {//lost permission
    	handle_err_permission();
    	return;
    }
    if( drone->flight_status != in_air_state )
    {//fell off air
    	handle_err_landed();
    	return;
    }
    time(&end_time);
    elapsed_time = difftime(end_time, start_time);
    if( elapsed_time >= hover_for_min_time )
    {
    	//x_toGo = x_des;
    	//y_toGo = y_des;
        state = stabilize_collect;
    	//publish_state();
    	ROS_INFO("Waiting for palantir");
    	elapsed_time = 0;
    }
}

void handle_stabilize_collect()
{
	if(!drone->activation)
    {//lost activation
    	handle_err_activation();
    	return;
    }
    if(!drone->sdk_permission_opened)
    {//lost permission
    	handle_err_permission();
    	return;
    }
    if( drone->flight_status != in_air_state )
    {//fell off air
    	handle_err_landed();
    	return;
    }
    //send position command
    //drone->local_position_navigation_send_request(x_toGo, y_toGo, (drone->local_position).z);
    if(flag_palantir)
    {
        state = palantir_decision_made;
        ROS_INFO("Palantir has made a decision");
    }    
    //publish_state();
}

void handle_palantir_decision_made()
{
    if(!drone->activation)
    {//lost activation
        handle_err_activation();
        return;
    }
    if(!drone->sdk_permission_opened)
    {//lost permission
        handle_err_permission();
        return;
    }
    if( drone->flight_status != in_air_state )
    {//fell off air
        handle_err_landed();
        return;
    }
    //send position command
    //drone->local_position_navigation_send_request(x_toGo, y_toGo, (drone->local_position).z);
    if(flag_decision)
    {
        state = approach_dock;
        ROS_INFO("Time to move up");
    }    
    //publish_state();
}

void handle_approach_dock()
{
	if(!drone->activation)
    {//lost activation
    	handle_err_activation();
    	return;
    }
    if(!drone->sdk_permission_opened)
    {//lost permission
    	handle_err_permission();
    	return;
    }
    if( drone->flight_status != in_air_state )
    {//fell off air
    	handle_err_landed();
    	return;
    }
    
   /* if( flag )
    {
    	state = hover2;
    	ROS_INFO("Hover 2"); 
    	//publish_state(); 
    	time(&start_time);  
        elapsed_time = 0;                 
    } */
}

void handle_docked()
{
	/*if(!drone->activation)
    {//lost activation
    	handle_err_activation();
    	return;
    }
    if(!drone->sdk_permission_opened)
    {//lost permission
    	handle_err_permission();
    	return;
    }
    if( drone->flight_status != in_air_state )
    {//fell off air
    	handle_err_landed();
    	return;
    }
    time(&end_time);
    elapsed_time = difftime(end_time, start_time);
    if( elapsed_time >= hover2_time )
    {
    	state = land;
    	ROS_INFO("Land");
    	//publish_state();
    	elapsed_time = 0;
    }*/
}

bool handle_default()
{
	ROS_INFO("Error");
	drone->release_sdk_permission_control();
	//publish_state();
	//if(termination)     return false;
}
/**************************************************************************/

#endif
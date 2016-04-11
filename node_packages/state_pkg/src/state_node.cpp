#include "defines_st.h"
//state functions
#include "client_st_functions.h"

/**********************MAIN FUNCTION******************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_node");
    ros::NodeHandle nh;
    initialize_state_node(argc, argv, nh);
    
    while(state != quit)
    {
        if(flag){
            //ROS_INFO("TERMINATION OF PROCESS REQUESTED!");
            //termination = true;
        }

        switch(state)
        {
            case init:
            {
                handle_init();
                break;
            }
            case request_control:
            {   
                handle_request_control();
                break;                
            }
            case takeoff:
            {
                handle_takeoff();
                break;                
            }
            case wait_for_takeoff:
            {
                handle_wait_for_takeoff();
                break;
            }
            case hover_for_min:
            {
                handle_hover_for_min();
                break;
            } 
            case stabilize_collect:
            {
                handle_stabilize_collect();
                break;
            }
            case palantir_decision_made:
            {
                handle_palantir_decision_made();
                break;
            }
            case approach_dock:
            {
                handle_approach_dock();
                break;
            }
            case docked:
            {
                handle_docked();
                break;
            } 
            case error_activation:
            case error_permission:
            case error_landed:
            default:
            {
                if( !handle_default() )
                {
                    return 0;
                }
            }
        }
        ros::spinOnce();
        rate->sleep();
    }
    return 0;
}








/**
 * @file mediator_node.cpp
 * @brief Mediator node between the ROS control system and the PX4 flightstack, written with MAVROS version 0.19.x, PX4 Pro Flight
 * 
 */
#include <ros/ros.h>
// necessary includes are performed in header
#include "offboard_interface.h"

//internal variables:
mavros_msgs::State current_state;
float input[6];
bool drone_armed=false;

float scaleControlOutputToActuators(float in)
{float out;
float upper_limit = 100;
float lower_limit = 0;

out = 2*(abs(in)/upper_limit)-1;
return out;
}
// define callback functions 
    void stateMessageCallback(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}
    void controlMessageCallback(const flypulator_common_msgs::RotorVelStamped msg)
{
 /* ROS_INFO("Control: %f %f %f %f  ", */ 
        /* msg.velocity[0], */
        /* msg.velocity[1], */
        /* msg.velocity[2], */
        /* msg.velocity[3]); */

input[0] = scaleControlOutputToActuators(msg.velocity[0]);
input[1] = scaleControlOutputToActuators(msg.velocity[1]); 
input[2] = scaleControlOutputToActuators(msg.velocity[2]); 
input[3] = scaleControlOutputToActuators(msg.velocity[3]); 
input[4] = scaleControlOutputToActuators(msg.velocity[4]); 
input[5] = scaleControlOutputToActuators(msg.velocity[5]); 

}

    void dynamicParamCallback(flypulator_mavros::offb_parameterConfig &config, uint32_t level)
{
    

 drone_armed = config.drone_armed;

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;
    //
// add subscribers for topics state and control output
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateMessageCallback);
    ros::Subscriber control_sub = nh.subscribe<flypulator_common_msgs::RotorVelStamped>("/drone/rotor_cmd", 100, controlMessageCallback);
    //
// add publisher for actuator commands
    ros::Publisher pub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 100); 
    
    
  // add service clients for arming and mode setting
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
// set up callbacks for dynamic reconfigure
        dynamic_reconfigure::Server<flypulator_mavros::offb_parameterConfig> param_srv;
        dynamic_reconfigure::Server<flypulator_mavros::offb_parameterConfig>::CallbackType cb;
        cb=boost::bind(&dynamicParamCallback, _1, _2);
        param_srv.setCallback(cb);

    // the rate must be higher than 2 Hz    
        ros::Rate loop_rate(50);
        ros::Rate wait_rate(5);
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value =drone_armed;
        mavros_msgs::ActuatorControl ac_msg; 

        ros::Time last_request = ros::Time::now();
    // run while the FCU is not connected    
        while(ros::ok() && !current_state.connected){
        ROS_DEBUG("Pixhawk is not connected");
     ac_msg.header.stamp = ros::Time::now();
   for (int i =0; i<6; i++){
    ac_msg.controls[i]=input[i];
    }
    pub.publish(ac_msg);            ros::spinOnce();
            wait_rate.sleep();
        }
        
    // loop to run when the FC is in offboard mode
        while(ros::ok()){ 
     for(int i=0; i<6; i++){
            printf("%f, ", input[i]);
        }
        printf("\n");

     if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


    ac_msg.header.stamp = ros::Time::now();
   for (int i =0; i<6; i++){
    ac_msg.controls[i]=input[i];
    }
    pub.publish(ac_msg); 
    
    
    ros::spinOnce();
    loop_rate.sleep();
    }
    return 0;
}
// loop  to enter offboard mode and arm vehicle



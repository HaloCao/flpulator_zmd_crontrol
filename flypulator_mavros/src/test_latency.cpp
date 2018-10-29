/**
 * @file mediator_node.cpp
 * @brief Mediator node between the ROS control system and the PX4 flightstack, written with MAVROS version 0.19.x, PX4 Pro Flight
 * 
 */
#include <stdlib.h>
#include <vector>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <flypulator_mavros/offb_parameterConfig.h> 
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Joy.h> // for testing with the 3Dconnexion mouse


    mavros_msgs::State current_state;
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

void param_cb(flypulator_mavros::offb_parameterConfig &config, uint32_t level)
{
        

}

   
int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_latency");
    ros::NodeHandle nh;
// add subscribers for topics state and control output
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
   
// add publisher for actuator commands
    ros::Publisher pub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 100); 

// add service clients for arming and mode setting

        dynamic_reconfigure::Server<flypulator_mavros::offb_parameterConfig> param_srv;
        dynamic_reconfigure::Server<flypulator_mavros::offb_parameterConfig>::CallbackType cb;
        cb=boost::bind(&param_cb, _1, _2);
        param_srv.setCallback(cb);
    // the rate must be higher than 2 Hz    
        ros::Rate LoopRate(50);
        mavros_msgs::ActuatorControl ac_msg; 

        ros::Time last_request = ros::Time::now();
    int signal_out=1;
    int flank=20;
    int number =0;

   while(ros::ok()){ 
       /* while(ros::ok() && current_state.mode=="OFFBOARD" && current_state.armed){ */ 
    
           if(flank>0)
           {
               
               flank -=1  ;
           }
           else
           { signal_out *= -1;
               flank = 20;
           }  
           
    
    for (int i =0; i<6; i++){
    ac_msg.controls[i]=signal_out;
    }
    ac_msg.header.stamp = ros::Time::now();
    ac_msg.header.frame_id = "blabla";

    pub.publish(ac_msg); 
    
    
    ros::spinOnce();
    LoopRate.sleep();
    }
    return 0;
}
// loop  to enter offboard mode and arm vehicle



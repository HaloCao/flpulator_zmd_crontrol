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

    float Input[6];
bool Armed=false;
void param_cb(flypulator_mavros::offb_parameterConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %f %f %f %f %f %f",
            config.motor1_speed,
            config.motor2_speed, 
            config.motor3_speed, 
            config.motor4_speed, 
            config.motor5_speed, 
            config.motor6_speed);
 Armed = config.drone_armed;
 Input[0] = config.motor6_speed; //this channel sets drone motor 6
 Input[1] = config.motor3_speed; // drone motor 3
 Input[2] = config.motor2_speed; // drone motor 2
 Input[3] = config.motor1_speed; // drone motor 1
 Input[4] = config.motor5_speed; // drone motor 5
 Input[5] = config.motor4_speed; //drone motor 4

    

}


    void input_cb(const sensor_msgs::Joy::ConstPtr& msg){
                int i = 0;
    for(std::vector<float>::const_iterator it = msg->axes.begin(); it != msg->axes.end(); ++it)
    {
                Input[i] = *it;
                /* ROS_INFO("%f: ", *it); */
                i++;
    }
    return;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mediator_node");
    ros::NodeHandle nh;
// add subscribers for topics state and control output
    ros::Subscriber input_sub = nh.subscribe("spacenav/joy", 10, input_cb); 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

   
// add publisher for actuator commands
    ros::Publisher pub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 100); 

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
// add service clients for arming and mode setting
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

        dynamic_reconfigure::Server<flypulator_mavros::offb_parameterConfig> param_srv;
        dynamic_reconfigure::Server<flypulator_mavros::offb_parameterConfig>::CallbackType cb;
        cb=boost::bind(&param_cb, _1, _2);
        param_srv.setCallback(cb);
    // the rate must be higher than 2 Hz    
        ros::Rate LoopRate(50);
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value =Armed;
        mavros_msgs::ActuatorControl ac_msg; 


        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2;


        ros::Time last_request = ros::Time::now();
    // run while the FCU is not connected    
        /* while(ros::ok() && !current_state.connected){ */
        /* for(int i=0; i<6; i++){ */
        /*     printf("%f, ", Input[i]); */
        /* } */
        /* printf("\n"); */
     
        /*     ros::spinOnce(); */
        /*     LoopRate.sleep(); */
        /* } */
        for(int i = 100; ros::ok() && i > 0; --i){
            local_pos_pub.publish(pose);
            ros::spinOnce();
            LoopRate.sleep();
        }
    // loop to run when the FC is in offboard mode
        while(ros::ok()){ 
     for(int i=0; i<6; i++){
            printf("%f, ", Input[i]);
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
    ac_msg.controls[i]=Input[i];
    }
    pub.publish(ac_msg); 
    
    
    ros::spinOnce();
    LoopRate.sleep();
    }
    return 0;
}
// loop  to enter offboard mode and arm vehicle



/**
 * @file mediator_node.cpp
 * @brief Mediator node between the ROS control system and the PX4 flightstack, written with MAVROS version 0.19.x, PX4
 * Pro Flight
 *
 */
#include <ros/ros.h>
// necessary includes are performed in header
#include "offboard_interface.h"

// internal variables:
mavros_msgs::State current_state;
mavros_msgs::State last_state;
ros::Publisher *g_pub;
float input[6];
mavros_msgs::CommandBool arm_cmd;
bool last_arming_state;
bool control_active;
bool test_signal;
float upper_limit;

float scaleControlOutputToActuators(float in)
{
  float out;
  // Upper limit is set through dynamic reconfigure.
  // assumed max RPM 6000 -> 628 rad/s

  out = 2 * (abs(in) / upper_limit) - 1;
  return out;
}
// define callback functions
void stateMessageCallback(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}
void controlMessageCallback(const flypulator_common_msgs::RotorVelStamped msg)
{
  mavros_msgs::ActuatorControl ac_msg;
  control_active = true;  // to prevent use of test parameters when flying.

  // The matching of motors to input channels is dependent on the wiring on the UAV and the
  // programming of the ESCs. The matching cen be tested by altering the motor speed of individual
  // motors in dynamic reconfigure.
  input[0] = scaleControlOutputToActuators(msg.velocity[1]);
  input[1] = scaleControlOutputToActuators(msg.velocity[4]);
  input[2] = scaleControlOutputToActuators(msg.velocity[0]);
  input[3] = scaleControlOutputToActuators(msg.velocity[3]);
  input[4] = scaleControlOutputToActuators(msg.velocity[5]);
  input[5] = scaleControlOutputToActuators(msg.velocity[2]);
  ac_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < 6; i++)
  {
    if (current_state.armed)
      ac_msg.controls[i] = input[i];
    else
      ac_msg.controls[i] = -1.0;
  }

  g_pub->publish(ac_msg);
}

void dynamicParamCallback(flypulator_mavros::offb_parameterConfig &config, uint32_t level)
{
  if (!control_active)
  {
    input[0] = config.motor2_speed;  // drone motor 2
    input[1] = config.motor5_speed;  // drone motor 5
    input[2] = config.motor1_speed;  // drone motor 1
    input[3] = config.motor4_speed;  // drone motor 4
    input[4] = config.motor6_speed;  // drone motor 6
    input[5] = config.motor3_speed;  // drone motor 3
  }
  else
  {
    // ROS_INFO("Messages on topic /drone/rotor_cmd. Will not interfer");
  }

  upper_limit = config.upper_limit;
  if (config.drone_armed)
  {
    arm_cmd.request.value = true;
  }

  else
  {
    arm_cmd.request.value = false;
    input[0] = -1.0;
    input[1] = -1.0;
    input[2] = -1.0;
    input[3] = -1.0;
    input[4] = -1.0;
    input[5] = -1.0;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offboard_node");
  ros::NodeHandle nh;

  // add subscribers for topics state and control output
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateMessageCallback);
  ros::Subscriber control_sub =
      nh.subscribe<flypulator_common_msgs::RotorVelStamped>("/drone/rotor_cmd", 10, controlMessageCallback);

  // add publisher for actuator commands
  ros::Publisher pub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 10);
  g_pub = &pub;

  // add service clients for arming and mode setting
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  // set up callbacks for dynamic reconfigure
  dynamic_reconfigure::Server<flypulator_mavros::offb_parameterConfig> param_srv;
  dynamic_reconfigure::Server<flypulator_mavros::offb_parameterConfig>::CallbackType cb;
  cb = boost::bind(&dynamicParamCallback, _1, _2);
  param_srv.setCallback(cb);

  // the rate must be higher than 2 Hz
  ros::Rate loop_rate(100);
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  //  loop to handle vehicle states

  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD")
    {
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        ROS_INFO("Waiting for controller..");
    }
    else
    {
      if (last_arming_state != arm_cmd.request.value)
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("arming requested");
        }
        last_arming_state = arm_cmd.request.value;
      }

      if (last_state.armed != current_state.armed)
      {
        if (current_state.armed == true)
          ROS_INFO("Vehicle armed");
        else
        {
          for (int i = 0; i < 6; i++)
          {
            input[i] = -1.0;
          }
          ROS_INFO("Vehicle disarmed");
        }

        last_state = current_state;
      }
    }
// to enable individual motor control through dynamic reconfigure
    if (!control_active)
    {
      mavros_msgs::ActuatorControl ac_msg;
      ac_msg.header.stamp = ros::Time::now();
      for (int i = 0; i < 6; i++)
      {
        ac_msg.controls[i] = input[i];
      }

      g_pub->publish(ac_msg);
    }
  
  ros::spinOnce();
  loop_rate.sleep();
}

return 0;
}

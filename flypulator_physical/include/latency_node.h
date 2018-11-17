#ifndef LATENCY_NODE_H
#define LATENCY_NODE_H

// include standard dependencies
#include <ros/ros.h>
#include <stdlib.h>
//#include <vector>
// include for dynamic reconfigure
//#include <flypulator_mavros/offb_parameterConfig.h>
//#include <dynamic_reconfigure/server.h>
// include message structs
#include <geometry_msgs/WrenchStamped.h>                   //for controller_wrench
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>  //for trajectory
//#include <std_msgs/> // for delay
#include <mavros_msgs/RCOut.h>                          //for motor PWM signals
#include <flypulator_common_msgs/UavStateRPYStamped.h>  // for desired_pose

class Latency
{
public:
  //    OffboardInterface();

private:
  void motorCallback(const mavros_msgs::RCOut::ConstPtr& msg);

  void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  void desired_poseCallback(const flypulator_common_msgs::UavStateRPYStamped::ConstPtr& msg);

  int last_motor_pwm_;
  int current_motor_pwm_;

  float current_wrench_z_;
  float last_wrench_z_;

  float current_desired_pose_z_;
  float last_desired_pose_z;
};
#endif  // LATENCY_NODE_H

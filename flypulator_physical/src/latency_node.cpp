/**
 * @file latency_node.cpp
 * A node to test the latency between different points in the control system
 */
#include <ros/ros.h>
// necessary includes are performed in header
#include "latency_node.h"
ros::Publisher* g_trajectory_pub;
ros::Publisher* g_delay_pub;

// std::vector<float> delay_times;
ros::Time t0;
ros::Time t1_step; //step in desired pose
ros::Time t2_step; //step in wrench
ros::Time t3_step; //step in rotor velocity
ros::Time t4_step; //step in actuator commands
ros::Time t5_step; //step in observed pwm

ros::Time last_time_pose;
ros::Time last_time_wrench;
ros::Time last_time_rotor;
ros::Time last_time_actuator;
ros::Time last_time_motor;

float last_desired_pose_z_;
float last_wrench_z_;
float last_rotor_vel;
float last_actuator_ctrl;
int last_motor_pwm_;

bool high = false;
float last_step_z;

void publishDelay()
{
  flypulator_common_msgs::DelayStamped delay_msg;
  delay_msg.header.stamp = ros::Time::now();
  float start = t0.toSec();

  delay_msg.trajToDesiredPose = t1_step.toSec() - start;
  delay_msg.desiredPoseToWrench = t2_step.toSec() - t1_step.toSec();
  delay_msg.wrenchToRotorCmd = t3_step.toSec() - t2_step.toSec();
  delay_msg.rotorCmdToActuatorCmd = t4_step.toSec() - t3_step.toSec();
  delay_msg.actuatorCmdToPwm = t5_step.toSec() - t4_step.toSec();
  delay_msg.desiredPoseToPwm = t5_step.toSec() - t1_step.toSec();

  g_delay_pub->publish(delay_msg);
}

void desired_poseCallback(const flypulator_common_msgs::UavStateRPYStamped::ConstPtr& msg)
{
  float current_desired_pose_z_ = msg->pose.z;
  ros::Time t1 = msg->header.stamp;
  if (current_desired_pose_z_ != last_desired_pose_z_)
  {
    t1_step = last_time_pose;
    ROS_INFO("flank in des pose");
  }
  last_desired_pose_z_ = current_desired_pose_z_;
  last_time_pose = t1;
}
void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{  
  float buffer = 1.0;
  ros::Time t2 = msg->header.stamp;
  float current_wrench_z_ = msg->wrench.force.z;
  // ROS_INFO("delay desired pose - wrench: %f \n", (t2.toSec() - t1.toSec()));
  if ((last_wrench_z_ + buffer) < current_wrench_z_ | (last_wrench_z_ - buffer) > current_wrench_z_)
  {
    ROS_INFO(" flanke in  wrench ");
    t2_step = last_time_wrench;
  }
  last_wrench_z_ = current_wrench_z_;
  last_time_wrench = t2;
}

void rotorCallback(const flypulator_common_msgs::RotorVelStamped::ConstPtr& msg)
{
  float buffer = 10.0;
  float current_rotor_vel = msg->velocity[1];

  ros::Time t3 = msg->header.stamp;

  if ((last_rotor_vel + buffer) < current_rotor_vel | (last_rotor_vel - buffer) > current_rotor_vel)
  {
    t3_step = last_time_rotor;
    ROS_INFO("flank in rotor vel");
  }
  last_rotor_vel = current_rotor_vel;
  last_time_rotor = t3;
}

void actuatorCtrlCallback(const mavros_msgs::ActuatorControl::ConstPtr& msg)
{
  float buffer = 0.1;
  float current_actuator_ctrl = msg->controls[1];
  ros::Time t4 = msg->header.stamp;

  if ((last_actuator_ctrl + buffer) < current_actuator_ctrl | (last_actuator_ctrl - buffer) > current_actuator_ctrl)
  {
    t4_step = last_time_actuator;
    ROS_INFO("flank in actuator control");
  }
  last_actuator_ctrl = current_actuator_ctrl;
  last_time_actuator = t4;
}

void motorCallback(const mavros_msgs::RCOut::ConstPtr& msg)
{
  int buffer = 30;
  int current_motor_pwm = msg->channels[1];
  ros::Time t5 = msg->header.stamp;

  if (current_motor_pwm > (last_motor_pwm_ + buffer) | current_motor_pwm < (last_motor_pwm_ - buffer))
  {
    ROS_INFO("motor step detected. current rpm : %i last rpm %i \n", current_motor_pwm, last_motor_pwm_);
    // ROS_INFO("motor step detected. Duration: %f", (t5.toSec() - last_rpm_time.toSec()));
    // ROS_INFO("delay from desired pose: %f", (last_rpm_time.toSec() - t1_step.toSec()));
    // ROS_INFO("delay from wrench: %f \n", (last_rpm_time.toSec() - t2_step.toSec()));
    t5_step = last_time_motor;
    publishDelay();
  }
  last_motor_pwm_ = current_motor_pwm;
  last_time_motor = t5;
}

void sendStep(float step_z)
{
  // prepare vectors
  geometry_msgs::Vector3 p_msg;
  p_msg.x = 0.0;
  p_msg.y = 0.0;
  p_msg.z = step_z;

  geometry_msgs::Vector3 p_dot_msg;
  p_dot_msg.x = 0.0;
  p_dot_msg.y = 0.0;
  p_dot_msg.z = 0.0;

  geometry_msgs::Vector3 p_ddot_msg;
  p_ddot_msg.x = 0.0;
  p_ddot_msg.y = 0.0;
  p_ddot_msg.z = 0.0;

  geometry_msgs::Vector3 omega_msg;
  omega_msg.x = 0.0;
  omega_msg.y = 0.0;
  omega_msg.z = 0.0;

  geometry_msgs::Vector3 omega_dot_msg;
  omega_dot_msg.x = 0.0;
  omega_dot_msg.y = 0.0;
  omega_dot_msg.z = 0.0;

  geometry_msgs::Quaternion q_msg;
  q_msg.w = 1.0;
  q_msg.x = 0.0;
  q_msg.y = 0.0;
  q_msg.z = 0.0;

  geometry_msgs::Transform transform_msg;
  transform_msg.translation = p_msg;
  transform_msg.rotation = q_msg;

  geometry_msgs::Twist velocities_msg;
  velocities_msg.linear = p_dot_msg;
  velocities_msg.angular = omega_msg;

  geometry_msgs::Twist accelerations_msg;
  accelerations_msg.linear = p_ddot_msg;
  accelerations_msg.angular = omega_dot_msg;

  // pack toghether in trajectory message

  ros::Duration traj_duration(0, 0);
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_msg;
  trajectory_msg.transforms.push_back(transform_msg);
  trajectory_msg.velocities.push_back(velocities_msg);
  trajectory_msg.accelerations.push_back(accelerations_msg);
  trajectory_msg.time_from_start = traj_duration;
  if (last_step_z != step_z)
  {
    last_step_z = step_z;
    t0 = ros::Time::now();
  }

  g_trajectory_pub->publish(trajectory_msg);
}

void timerCallback(const ros::TimerEvent& event)
{
  if (high)
    high = false;
  else
    high = true;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "latency_node");
  ros::NodeHandle nh;

  // add subscribers

  ros::Subscriber desired_pose_sub =
      nh.subscribe<flypulator_common_msgs::UavStateRPYStamped>("/drone/desired_pose", 1, desired_poseCallback);

  ros::Subscriber wrench_sub =
      nh.subscribe<geometry_msgs::WrenchStamped>("/drone/controller_wrench", 1, wrenchCallback);

  ros::Subscriber rotor_sub =
      nh.subscribe<flypulator_common_msgs::RotorVelStamped>("/drone/rotor_cmd", 1, rotorCallback);

  ros::Subscriber actuatorCtrl_sub =
      nh.subscribe<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1, actuatorCtrlCallback);

  ros::Subscriber motor_sub = nh.subscribe<mavros_msgs::RCOut>("/mavros/rc/out", 1, motorCallback);

  // add publisher for delay msg
  ros::Publisher delay_pub = nh.advertise<flypulator_common_msgs::DelayStamped>("/drone/delay", 10);
  g_delay_pub = &delay_pub;

  ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("trajectory", 10);
  g_trajectory_pub = &trajectory_pub;

  ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    if (high)
    {
      sendStep(200.0);
    }
    else
    {
      sendStep(0.0);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


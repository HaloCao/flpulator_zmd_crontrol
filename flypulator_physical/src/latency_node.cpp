/**
 * @file latency_node.cpp
 * A node to test the latency between different signals in the control system
 */
#include <ros/ros.h>
// necessary includes are performed in header
#include "latency_node.h"
ros::Publisher* g_trajectory_pub;
int last_motor_pwm_;
int current_motor_pwm_;
int buffer = 50;  // empirical value to filter out fluctuations in rpm.
ros::Time last_rpm_time;
ros::Time current_rpm_time;
ros::Time t0;
float current_wrench_z_;

float last_wrench_z_;

float current_desired_pose_z_;
float last_desired_pose_z;
bool end_of_loop;
bool high = false;

void motorCallback(const mavros_msgs::RCOut::ConstPtr& msg)
{  // detect step response
  current_motor_pwm_ = msg->channels[0];
  current_rpm_time = msg->header.stamp;

  if (current_motor_pwm_ > (last_motor_pwm_ + buffer))
  {
    end_of_loop = true;
    ROS_INFO("motor step detected. current rpm : %i last rpm %i", current_motor_pwm_, last_motor_pwm_);
    ROS_INFO("motor step detected. Duration: %f", (current_rpm_time.toSec() - last_rpm_time.toSec()));
    ROS_INFO("delay: %f", (current_rpm_time.toSec() - t0.toSec()));
  }

  last_motor_pwm_ = current_motor_pwm_;
  last_rpm_time = current_rpm_time;
}

void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{  // detect step response
   // store variable
   // ROS_INFO("wrench callback");
}

void desired_poseCallback(const flypulator_common_msgs::UavStateRPYStamped::ConstPtr& msg)
{  // detect step response

  // ROS_INFO("desired pose callback");
}

void publishDelay()
{  // publish the delay of the messages.
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

  ros::Duration traj_duration(0, 0);
  // pack toghether in trajectory message
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_msg;
  trajectory_msg.transforms.push_back(transform_msg);
  trajectory_msg.velocities.push_back(velocities_msg);
  trajectory_msg.accelerations.push_back(accelerations_msg);
  trajectory_msg.time_from_start = traj_duration;
  t0 = ros::Time::now();
  end_of_loop = false;
  g_trajectory_pub->publish(trajectory_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "latency_node");
  ros::NodeHandle nh;

  // add subscribers
  ros::Subscriber motor_sub = nh.subscribe<mavros_msgs::RCOut>("/mavros/rc/out", 1, motorCallback);

  ros::Subscriber wrench_sub =
      nh.subscribe<geometry_msgs::WrenchStamped>("/drone/controller_wrench", 1, wrenchCallback);

  ros::Subscriber desired_pose_sub =
      nh.subscribe<flypulator_common_msgs::UavStateRPYStamped>("/drone/desired_pose", 1, desired_poseCallback);

  // add publisher for delay msg
  ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("trajectory", 10);
  g_trajectory_pub = &trajectory_pub;

  // add service client for trajectory
  //    ros::ServiceClient trajectory_client = nh.serviceClient<mavros_msgs::CommandBool>("trajectory");
  // set up callbacks for dynamic reconfigure

  ros::Rate loop_rate(100);

  ros::Time last_request = ros::Time::now();
  //  loop to handle vehicle states

  while (ros::ok())
  {
    if (end_of_loop)
    {
      if (high)
      {
        sendStep(200.0);
        high = false;
      }
      else
      {
        sendStep(0.0);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


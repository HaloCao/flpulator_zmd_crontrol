#include "ros/ros.h"
#include "flypulator_control/controller_interface.h"  // performs all necessary includes
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "flypulator_common_msgs/UavStateStamped.h"
#include "geometry_msgs/WrenchStamped.h"

ControllerInterface* g_drone_controller_p;
PoseVelocityAcceleration g_current_pose;
PoseVelocityAcceleration g_desired_pose;
int g_rotor_vel_message_counter = 0;
ros::Publisher* g_rotor_cmd_pub;
Eigen::Matrix<float, 6, 1> g_spinning_rates;

ros::Publisher* g_control_wrench_pub;

void computeControlOutputAndPublish()
{
  Eigen::Matrix<float,6,1> control_wrench;
  // compute spinning rates
  ROS_DEBUG("Compute Control Output..");
  g_drone_controller_p->computeControlOutput(g_desired_pose, g_current_pose, g_spinning_rates);
  ROS_DEBUG("Control Output computed! Prepare rotor cmd message...");
  control_wrench = g_drone_controller_p->getControlWrench();
  // build message
  flypulator_common_msgs::RotorVelStamped msg;
  msg.header.stamp = ros::Time::now();
  for (int i = 0; i < 6; i++)
  {
    msg.velocity.push_back(g_spinning_rates(i, 0));
    msg.name.push_back(std::string("blade_joint") + std::to_string(i));
  }
  ROS_DEBUG("Send rotor cmd message..");
  g_rotor_cmd_pub->publish(msg);

  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = ros::Time::now();
  wrench_msg.wrench.force.x = control_wrench(0,0);
  wrench_msg.wrench.force.y = control_wrench(1,0);
  wrench_msg.wrench.force.z = control_wrench(2,0);
  wrench_msg.wrench.torque.x = control_wrench(3,0);
  wrench_msg.wrench.torque.y = control_wrench(4,0);
  wrench_msg.wrench.torque.z = control_wrench(5,0);
  g_control_wrench_pub->publish(wrench_msg);
}

template <class T>
T GetMax(T a, T b)
{
  T result;
  result = (a > b) ? a : b;
  return (result);
}

// encode a trajectory_msgs::MultiDOFJointTrajectoryPoint message to PoseVelocityAcceleration object
void encodeTrajectoryMsg(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg,
                         PoseVelocityAcceleration& pose_dest)
{
  geometry_msgs::Transform transform = msg->transforms[0];
  geometry_msgs::Twist velocity = msg->velocities[0];
  geometry_msgs::Twist acceleration = msg->accelerations[0];

  geometry_msgs::Vector3 p_des_msg = transform.translation;
  geometry_msgs::Quaternion q_des_msg = transform.rotation;
  geometry_msgs::Vector3 p_dot_des_msg = velocity.linear;
  geometry_msgs::Vector3 omega_des_msg = velocity.angular;
  geometry_msgs::Vector3 p_ddot_des_msg = acceleration.linear;
  geometry_msgs::Vector3 omega_dot_des_msg = acceleration.angular;

  Eigen::Vector3f p_des(p_des_msg.x, p_des_msg.y, p_des_msg.z);
  Eigen::Quaternionf q_des(q_des_msg.w, q_des_msg.x, q_des_msg.y, q_des_msg.z);

  Eigen::Vector3f p_dot_des(p_dot_des_msg.x, p_dot_des_msg.y, p_dot_des_msg.z);
  Eigen::Vector3f omega_des(omega_des_msg.x, omega_des_msg.y, omega_des_msg.z);

  Eigen::Vector3f p_ddot_des(p_ddot_des_msg.x, p_ddot_des_msg.y, p_ddot_des_msg.z);
  Eigen::Vector3f omega_dot_des(omega_dot_des_msg.x, omega_dot_des_msg.y, omega_dot_des_msg.z);

  // update global variable for desired pose
  pose_dest.p = p_des;
  pose_dest.q = q_des;
  pose_dest.p_dot = p_dot_des;
  pose_dest.omega = omega_des;
  pose_dest.p_ddot = p_ddot_des;
  pose_dest.omega_dot = omega_dot_des;
}

void encodeStateMsg(const flypulator_common_msgs::UavStateStamped::ConstPtr& msg, PoseVelocityAcceleration& pose_dest)
{
  geometry_msgs::Pose transform = msg->pose;
  geometry_msgs::Twist velocity = msg->velocity;
  geometry_msgs::Accel acceleration = msg->acceleration;

  geometry_msgs::Point p_des_msg = transform.position;
  geometry_msgs::Quaternion q_des_msg = transform.orientation;
  geometry_msgs::Vector3 p_dot_des_msg = velocity.linear;
  geometry_msgs::Vector3 omega_des_msg = velocity.angular;
  geometry_msgs::Vector3 p_ddot_des_msg = acceleration.linear;
  geometry_msgs::Vector3 omega_dot_des_msg = acceleration.angular;

  Eigen::Vector3f p_des(p_des_msg.x, p_des_msg.y, p_des_msg.z);
  Eigen::Quaternionf q_des(q_des_msg.w, q_des_msg.x, q_des_msg.y, q_des_msg.z);

  Eigen::Vector3f p_dot_des(p_dot_des_msg.x, p_dot_des_msg.y, p_dot_des_msg.z);
  Eigen::Vector3f omega_des(omega_des_msg.x, omega_des_msg.y, omega_des_msg.z);

  Eigen::Vector3f p_ddot_des(p_ddot_des_msg.x, p_ddot_des_msg.y, p_ddot_des_msg.z);
  Eigen::Vector3f omega_dot_des(omega_dot_des_msg.x, omega_dot_des_msg.y, omega_dot_des_msg.z);

  // update global variable for desired pose
  pose_dest.p = p_des;
  pose_dest.q = q_des;
  pose_dest.p_dot = p_dot_des;
  pose_dest.omega = omega_des;
  pose_dest.p_ddot = p_ddot_des;
  pose_dest.omega_dot = omega_dot_des;
}

// receive trajectory message
void trajectoryMessageCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg)
{
  // encode trajectory message to PoseVelocityAcceleration object
  encodeTrajectoryMsg(msg, g_desired_pose);

  ros::Duration duration = msg->time_from_start;

  ROS_DEBUG("Received trajectory message: x_des = [%f, %f, %f], q_des = [%f, %f, %f, %f]", g_desired_pose.p.x(),
            g_desired_pose.p.y(), g_desired_pose.p.z(), g_desired_pose.q.w(), g_desired_pose.q.x(),
            g_desired_pose.q.y(), g_desired_pose.q.z());
  ROS_DEBUG("    Time from start: %f s", duration.toSec());
}

// receive state estimation message
void stateMessageCallback(const flypulator_common_msgs::UavStateStamped::ConstPtr& msg)
{
  // Tencode state message to PoseVelocityAcceleration object
  encodeStateMsg(msg, g_current_pose);

  // ros::Duration duration = msg->time_from_start;

  ROS_DEBUG("Received state message: x_cur = [%f, %f, %f], q_cur = [%f, %f, %f, %f]", g_current_pose.p.x(),
            g_current_pose.p.y(), g_current_pose.p.z(), g_current_pose.q.w(), g_current_pose.q.x(),
            g_current_pose.q.y(), g_current_pose.q.z());
  // ROS_DEBUG("    Time from start: %f s", duration.toSec());

  // compute control output to updated state information
  computeControlOutputAndPublish();
}

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "controller");

  ros::NodeHandle n;

  // ready to publish controller wrench
  ros::Publisher control_wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("/drone/controller_wrench", 100);
  g_control_wrench_pub = &control_wrench_pub;

  // suscribe to trajectory messages
  ros::Subscriber sub = n.subscribe("trajectory", 1000, trajectoryMessageCallback);

  // suscribe to state estimation messages
  ros::Subscriber sub_2 = n.subscribe("/drone/meas_state", 1000, stateMessageCallback);

  // ready to publish rotor command messages
  ros::Publisher rotor_cmd_pub = n.advertise<flypulator_common_msgs::RotorVelStamped>("/drone/rotor_cmd", 1000);
  g_rotor_cmd_pub = &rotor_cmd_pub;

  // create controller
  ControllerInterface m_drone_controller;
  g_drone_controller_p = &m_drone_controller;

  // Set up a dynamic reconfigure server following
  // https://github.com/UCSD-E4E/stingray-auv/wiki/Writing-publisher-subscriber-with-dynamic-reconfigure-and-parameter-server-(C----)
  dynamic_reconfigure::Server<flypulator_control::ism_parameterConfig> dr_srv;
  dynamic_reconfigure::Server<flypulator_control::ism_parameterConfig>::CallbackType cb;
  cb = boost::bind(&BaseController::configCallback, g_drone_controller_p->getControllerReference(), _1,
                   _2);  // set callback of controller object
  dr_srv.setCallback(cb);

  // set inital pose
  g_desired_pose.p = Eigen::Vector3f(0, 0, 0.23f);
  g_current_pose.p = Eigen::Vector3f(0, 0, 0.22f);

  ros::spin();

  return 0;
}

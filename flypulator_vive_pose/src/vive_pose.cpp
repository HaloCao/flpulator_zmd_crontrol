#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <flypulator_common_msgs/UavStateStamped.h>

ros::Publisher pub_pose_tracker;
// ros::Publisher pub_linear_vel_tracker;
// ros::Publisher pub_linear_acc_tracker;
// ros::Publisher pub_angular_vel_tracker;
// ros::Publisher pub_angular_acc_tracker;
ros::Publisher pub_uav_state;

/**
 * 3d-vector subtraction
 */
geometry_msgs::Vector3 vec3Sub(const geometry_msgs::Vector3 &a, const geometry_msgs::Vector3 &b)
{
  geometry_msgs::Vector3 c;
  c.x = a.x - b.x;
  c.y = a.y - b.y;
  c.z = a.z - b.z;
  return c;
}

/**
 * divide a 3d-vector by a scalar
 */
geometry_msgs::Vector3 vec3Div(const geometry_msgs::Vector3 &a, double b)
{
  geometry_msgs::Vector3 c;
  c.x = a.x / b;
  c.y = a.y / b;
  c.z = a.z / b;
  return c;
}

/**
 * rotate a 3d-vector with a quaternion 
 */
geometry_msgs::Vector3 rotVect3(const geometry_msgs::Vector3 &in, const tf::Quaternion &q)
{
  tf::Vector3 v_in(in.x, in.y, in.z);
  tf::Transform T;
  T.setOrigin(tf::Vector3(0, 0, 0));
  T.setRotation(q);
  tf::Vector3 v_out = T * v_in;
  geometry_msgs::Vector3 out;
  out.x = v_out.x();
  out.y = v_out.y();
  out.z = v_out.z();
  return out;
}

void vive_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  static bool isTfInit = false;
  double current_time_stamp = msg->header.stamp.toSec();
  static double last_time_stamp;
  geometry_msgs::Pose pose = msg->pose.pose;
  geometry_msgs::Vector3 linear_vel, delta_linear_vel;
  geometry_msgs::Vector3 angular_vel, delta_angular_vel;
  static geometry_msgs::Vector3 last_linear_vel, last_angular_vel;
  geometry_msgs::Vector3 linear_acc, angular_acc;

  static tf::Transform T_init, T_init_inv;
  tf::Transform T_curr;

  // current pose w.r.t. the vive world coordinate(the light house system)
  T_curr.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  T_curr.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

  // initialize
  if (isTfInit == false)
  {
    isTfInit = true;
    T_init.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    T_init.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
    T_init_inv = T_init.inverse();
  }

  // get pose with reference to the inital coordinate(initial pose)
  geometry_msgs::PoseStamped pub_pose;
  tf::Transform T_out = T_init.inverse() * T_curr;
  pub_pose.header.stamp = msg->header.stamp;
  pub_pose.header.frame_id = "vive_world";
  pub_pose.pose.position.x = T_out.getOrigin().getX();
  pub_pose.pose.position.y = T_out.getOrigin().getY();
  pub_pose.pose.position.z = T_out.getOrigin().getZ();
  pub_pose.pose.orientation.x = T_out.getRotation().getX();
  pub_pose.pose.orientation.y = T_out.getRotation().getY();
  pub_pose.pose.orientation.z = T_out.getRotation().getZ();
  pub_pose.pose.orientation.w = T_out.getRotation().getW();

  double delta_t = current_time_stamp - last_time_stamp;

  // ROS_INFO("current time:%f, last time:%f, delta time:%f", current_time_stamp, last_time_stamp, delta_t);

  last_time_stamp = current_time_stamp;

  // check if delta t is positive
  if (delta_t <= 0){
    ROS_ERROR("vivo_pose: non-positive, delta t = %f !!!", delta_t);
    return;
  }
  // transfrom twist into inital coordinate
  linear_vel = msg->twist.twist.linear;
  linear_vel = rotVect3(linear_vel, T_init_inv.getRotation());
  angular_vel = msg->twist.twist.angular;
  angular_vel = rotVect3(angular_vel, T_init_inv.getRotation());

  // differentiate the linear and angular velocity
  delta_linear_vel = vec3Sub(linear_vel, last_linear_vel);
  delta_angular_vel = vec3Sub(angular_vel, last_angular_vel);
  linear_acc = vec3Div(delta_linear_vel, delta_t);
  angular_acc = vec3Div(delta_angular_vel, delta_t);

  // keep current measurements for next loop
  last_linear_vel = linear_vel;
  last_angular_vel = angular_vel;

  // load UavStateStamped message
  flypulator_common_msgs::UavStateStamped uav_state_msg;
  uav_state_msg.header.stamp = msg->header.stamp;
  // pose
  uav_state_msg.pose.position.x = T_out.getOrigin().getX();
  uav_state_msg.pose.position.y = T_out.getOrigin().getY();
  uav_state_msg.pose.position.z = T_out.getOrigin().getZ();

  uav_state_msg.pose.orientation.w = T_out.getRotation().getW();
  uav_state_msg.pose.orientation.x = T_out.getRotation().getX();
  uav_state_msg.pose.orientation.y = T_out.getRotation().getY();
  uav_state_msg.pose.orientation.z = T_out.getRotation().getZ();

  // velocity
  uav_state_msg.velocity.linear.x = linear_vel.x;
  uav_state_msg.velocity.linear.y = linear_vel.y;
  uav_state_msg.velocity.linear.z = linear_vel.z;
  uav_state_msg.velocity.angular.x = angular_vel.x;
  uav_state_msg.velocity.angular.y = angular_vel.y;
  uav_state_msg.velocity.angular.z = angular_vel.z;
  // acceleration
  uav_state_msg.acceleration.linear.x = linear_acc.x;
  uav_state_msg.acceleration.linear.y = linear_acc.y;
  uav_state_msg.acceleration.linear.z = linear_acc.z;
  uav_state_msg.acceleration.angular.x = angular_acc.x;
  uav_state_msg.acceleration.angular.y = angular_acc.y;
  uav_state_msg.acceleration.angular.z = angular_acc.z;

  // publish all messages

  pub_pose_tracker.publish(pub_pose);
  // pub_linear_vel_tracker.publish(linear_vel);
  // pub_angular_vel_tracker.publish(angular_vel);
  // pub_linear_acc_tracker.publish(linear_acc);
  // pub_angular_acc_tracker.publish(angular_acc);

  pub_uav_state.publish(uav_state_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flypulator_vive_pose");

  ros::NodeHandle nh("~");

  pub_pose_tracker = nh.advertise<geometry_msgs::PoseStamped>("vive_pose", 10);
  // pub_linear_vel_tracker = nh.advertise<geometry_msgs::Vector3>("vive_vel_linear", 10);
  // pub_linear_acc_tracker = nh.advertise<geometry_msgs::Vector3>("vive_acc_linear", 10);
  // pub_angular_vel_tracker = nh.advertise<geometry_msgs::Vector3>("vive_vel_angular", 10);
  // pub_angular_acc_tracker = nh.advertise<geometry_msgs::Vector3>("vive_acc_angular", 10);

  pub_uav_state = nh.advertise<flypulator_common_msgs::UavStateStamped>("vive_uav_state", 10);
ROS_INFO("running");
ros::Subscriber sub_pose1 = nh.subscribe<nav_msgs::Odometry>("/LHR_08DDEDC9_odom", 100, vive_odom_callback);
  ros::spin();

  return 0;
}

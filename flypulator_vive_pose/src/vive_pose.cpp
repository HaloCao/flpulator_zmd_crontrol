#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <flypulator_common_msgs/UavStateStamped.h>
#include "flypulator_common_msgs/UavStateRPYStamped.h"

struct T_TWIST
{
  double vx;
  double vy;
  double vz;
  double wx;
  double wy;
  double wz;
};

// ros::Publisher pub_pose_tracker;
// ros::Publisher pub_linear_vel_tracker;
// ros::Publisher pub_linear_acc_tracker;
// ros::Publisher pub_angular_vel_tracker;
// ros::Publisher pub_angular_acc_tracker;
ros::Publisher pub_uav_state;
ros::Publisher pub_uav_state_rpy;  // publish orientation in rpy representation

/**
 * Frame abbreviation
 * W: (flypulator)world, or inital body frame.
 * L: lighthouse system, vive_world.
 * S: sensor, the tracker.
 * B: body.
 * E.G. T_LS, transformation between L and S (w.r.t. L frame).
 */

tf::Transform T_S_B;  // transformation between tracker(S) and body(B) w.r.t. tracker(S)

// default frame ID
std::string world_id = "world";
std::string tracker_id = "tracker";
std::string mcap_world_id = "vive_world";
std::string body_id = "base_link";

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
tf::Vector3 rotVect3(const tf::Quaternion &q, const tf::Vector3 &in)
{
  tf::Transform T;
  T.setOrigin(tf::Vector3(0, 0, 0));
  T.setRotation(q);
  tf::Vector3 v_out = T * in;
  return v_out;
}

/**
 * make skew-symmetric matrix(w) and multiply by a 3d-vector(a)
 */
tf::Vector3 skewSymMul(tf::Vector3 w, tf::Vector3 a)
{
  tf::Vector3 out;
  out.setX(-w.z() * a.y() + w.y() * a.z());
  out.setY(w.z() * a.x() - w.x() * a.z());
  out.setZ(-w.y() * a.x() + w.x() * a.y());

  return out;
}

// TODO: rewrite using EIGEN? and package in to a class
/**
 * transform twist from vive_world(L) to world(W)
 */
T_TWIST transfromTwist(tf::Transform &T_WL, tf::Transform &T_SB, tf::Transform &T_LS, T_TWIST twist_in)
{
  tf::Vector3 v;
  tf::Vector3 omega;
  v.setX(twist_in.vx);
  v.setY(twist_in.vy);
  v.setZ(twist_in.vz);
  omega.setX(twist_in.wx);
  omega.setY(twist_in.wy);
  omega.setZ(twist_in.wz);

  tf::Vector3 p_S_SB = T_SB.getOrigin();
  // TODO: check correctness.
  tf::Vector3 v_W_WB = rotVect3(T_WL.getRotation(), skewSymMul(omega, rotVect3(T_LS.getRotation(), p_S_SB)) + v);
  tf::Vector3 omega_W_WB = rotVect3(T_WL.getRotation(), omega);

  T_TWIST twist_out;
  twist_out.vx = v_W_WB.x();
  twist_out.vy = v_W_WB.y();
  twist_out.vz = v_W_WB.z();
  twist_out.wx = omega_W_WB.x();
  twist_out.wy = omega_W_WB.y();
  twist_out.wz = omega_W_WB.z();

  return twist_out;
}

void vive_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  static tf::TransformBroadcaster tf_br;
  static bool isTfInit = false;
  double current_time_stamp = msg->header.stamp.toSec();
  static double last_time_stamp;
  geometry_msgs::Pose pose = msg->pose.pose;
  geometry_msgs::Vector3 linear_vel, delta_linear_vel;
  geometry_msgs::Vector3 angular_vel, delta_angular_vel;
  static geometry_msgs::Vector3 last_linear_vel, last_angular_vel;
  geometry_msgs::Vector3 linear_acc, angular_acc;

  static tf::Transform T_LW, T_WL;
  tf::Transform T_LS;

  // current pose of the tracker(S) w.r.t. the vive world(L, the light house system) T_LS
  T_LS.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  T_LS.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

  // initialize
  if (isTfInit == false)
  {
    isTfInit = true;
    T_LW = T_LS * T_S_B;    // initial body pose w.r.t. vive_world, T_LW
    T_WL = T_LW.inverse();  // T_WL
  }

  // get pose with reference to the inital coordinate(initial pose)
  geometry_msgs::PoseStamped pub_pose;
  tf::Transform T_out = T_WL * T_LS * T_S_B;  // body pose w.r.t. drone's world
  // pub_pose.header.stamp = msg->header.stamp;
  // pub_pose.header.frame_id = world_id;
  // pub_pose.pose.position.x = T_out.getOrigin().getX();
  // pub_pose.pose.position.y = T_out.getOrigin().getY();
  // pub_pose.pose.position.z = T_out.getOrigin().getZ();
  // pub_pose.pose.orientation.x = T_out.getRotation().getX();
  // pub_pose.pose.orientation.y = T_out.getRotation().getY();
  // pub_pose.pose.orientation.z = T_out.getRotation().getZ();
  // pub_pose.pose.orientation.w = T_out.getRotation().getW();

  double delta_t = current_time_stamp - last_time_stamp;

  // ROS_INFO("current time:%f, last time:%f, delta time:%f", current_time_stamp, last_time_stamp, delta_t);

  last_time_stamp = current_time_stamp;

  // check if delta t is positive
  if (delta_t <= 0)
  {
    ROS_ERROR("vivo_pose: non-positive, delta t = %f !!!", delta_t);
  }
  // transfrom twist into inital coordinate
  linear_vel = msg->twist.twist.linear;
  // linear_vel = rotVect3(linear_vel, T_WL.getRotation());
  angular_vel = msg->twist.twist.angular;
  // angular_vel = rotVect3(angular_vel, T_WL.getRotation());

  // transform twist into flypulator_world
  T_TWIST twist_in, twist_transformed;
  twist_in.vx = linear_vel.x;
  twist_in.vy = linear_vel.y;
  twist_in.vz = linear_vel.z;
  twist_in.wx = angular_vel.x;
  twist_in.wy = angular_vel.y;
  twist_in.wz = angular_vel.z;
  twist_transformed = transfromTwist(T_WL, T_S_B, T_LS, twist_in);
  linear_vel.x = twist_transformed.vx;
  linear_vel.y = twist_transformed.vy;
  linear_vel.z = twist_transformed.vz;
  angular_vel.x = twist_transformed.wx;
  angular_vel.y = twist_transformed.wy;
  angular_vel.z = twist_transformed.wz;

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
  flypulator_common_msgs::UavStateRPYStamped uav_state_rpy_msg;

  uav_state_msg.header.stamp = msg->header.stamp;
  // pose
  uav_state_msg.pose.position.x = T_out.getOrigin().getX();
  uav_state_msg.pose.position.y = T_out.getOrigin().getY();
  uav_state_msg.pose.position.z = T_out.getOrigin().getZ();
  uav_state_msg.pose.orientation.w = T_out.getRotation().getW();
  uav_state_msg.pose.orientation.x = T_out.getRotation().getX();
  uav_state_msg.pose.orientation.y = T_out.getRotation().getY();
  uav_state_msg.pose.orientation.z = T_out.getRotation().getZ();

  uav_state_rpy_msg.pose.x = T_out.getOrigin().getX();
  uav_state_rpy_msg.pose.y = T_out.getOrigin().getY();
  uav_state_rpy_msg.pose.z = T_out.getOrigin().getZ();
  double current_roll, current_pitch, current_yaw;
  tf::Matrix3x3(T_out.getRotation()).getRPY(current_roll, current_pitch, current_yaw);
  uav_state_rpy_msg.pose.roll = current_roll;
  uav_state_rpy_msg.pose.pitch = current_pitch;
  uav_state_rpy_msg.pose.yaw = current_yaw;

  // velocity
  uav_state_msg.velocity.linear.x = linear_vel.x;
  uav_state_msg.velocity.linear.y = linear_vel.y;
  uav_state_msg.velocity.linear.z = linear_vel.z;
  uav_state_msg.velocity.angular.x = angular_vel.x;
  uav_state_msg.velocity.angular.y = angular_vel.y;
  uav_state_msg.velocity.angular.z = angular_vel.z;

  uav_state_rpy_msg.velocity.linear.x = linear_vel.x;
  uav_state_rpy_msg.velocity.linear.y = linear_vel.y;
  uav_state_rpy_msg.velocity.linear.z = linear_vel.z;
  uav_state_rpy_msg.velocity.angular.x = angular_vel.x;
  uav_state_rpy_msg.velocity.angular.y = angular_vel.y;
  uav_state_rpy_msg.velocity.angular.z = angular_vel.z;
  // acceleration
  uav_state_msg.acceleration.linear.x = linear_acc.x;
  uav_state_msg.acceleration.linear.y = linear_acc.y;
  uav_state_msg.acceleration.linear.z = linear_acc.z;
  uav_state_msg.acceleration.angular.x = angular_acc.x;
  uav_state_msg.acceleration.angular.y = angular_acc.y;
  uav_state_msg.acceleration.angular.z = angular_acc.z;

  uav_state_rpy_msg.acceleration.linear.x = linear_acc.x;
  uav_state_rpy_msg.acceleration.linear.y = linear_acc.y;
  uav_state_rpy_msg.acceleration.linear.z = linear_acc.z;
  uav_state_rpy_msg.acceleration.angular.x = angular_acc.x;
  uav_state_rpy_msg.acceleration.angular.y = angular_acc.y;
  uav_state_rpy_msg.acceleration.angular.z = angular_acc.z;

  // publish all messages

  // pub_pose_tracker.publish(pub_pose);
  // pub_linear_vel_tracker.publish(linear_vel);
  // pub_angular_vel_tracker.publish(angular_vel);
  // pub_linear_acc_tracker.publish(linear_acc);
  // pub_angular_acc_tracker.publish(angular_acc);

  pub_uav_state.publish(uav_state_msg);
  pub_uav_state_rpy.publish(uav_state_rpy_msg);
  tf_br.sendTransform(tf::StampedTransform(T_LW, msg->header.stamp, mcap_world_id, world_id));
  tf_br.sendTransform(tf::StampedTransform(T_out, msg->header.stamp, world_id, body_id));
  tf_br.sendTransform(tf::StampedTransform(T_S_B.inverse(), msg->header.stamp, body_id, tracker_id));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flypulator_vive_pose");

  ros::NodeHandle nh("~");

  if (!nh.param("world_id", world_id, world_id))
    ROS_WARN("[vive_pose] No World ID given, use default: %s.", world_id.c_str());

  if (!nh.param("tracker_id", tracker_id, tracker_id))
    ROS_WARN("[vive_pose] No Tracker ID given, use default: %s.", tracker_id.c_str());

  if (!nh.param("mcap_world_id", mcap_world_id, mcap_world_id))
    ROS_WARN("[vive_pose] No Mcap World ID given, use default: %s.", mcap_world_id.c_str());

  if (!nh.param("body_id", body_id, body_id))
    ROS_WARN("[vive_pose] No Body ID given, use default: %s.", body_id.c_str());

  double tx = 0, ty = 0, tz = 0, qx = 0, qy = 0, qz = 0, qw = 1;
  if (!nh.param("T_S_B_tx", tx, tx))
    ROS_WARN("[vive_pose] No T_S_B tx given, use default: %f.", tx);
  if (!nh.param("T_S_B_ty", ty, ty))
    ROS_WARN("[vive_pose] No T_S_B ty given, use default: %f.", ty);
  if (!nh.param("T_S_B_tz", tz, tz))
    ROS_WARN("[vive_pose] No T_S_B tz given, use default: %f.", tz);
  if (!nh.param("T_S_B_qx", qx, qx))
    ROS_WARN("[vive_pose] No T_S_B qx given, use default: %f.", qx);
  if (!nh.param("T_S_B_qy", qy, qy))
    ROS_WARN("[vive_pose] No T_S_B qy given, use default: %f.", qy);
  if (!nh.param("T_S_B_qz", qz, qz))
    ROS_WARN("[vive_pose] No T_S_B qz given, use default: %f.", qz);
  if (!nh.param("T_S_B_qw", qw, qw))
    ROS_WARN("[vive_pose] No T_S_B qw given, use default: %f.", qw);

  T_S_B.setOrigin(tf::Vector3(tx, ty, tz));
  T_S_B.setRotation(tf::Quaternion(qx, qy, qz, qw));
  // pub_pose_tracker = nh.advertise<geometry_msgs::PoseStamped>("vive_pose", 10);
  // pub_linear_vel_tracker = nh.advertise<geometry_msgs::Vector3>("vive_vel_linear", 10);
  // pub_linear_acc_tracker = nh.advertise<geometry_msgs::Vector3>("vive_acc_linear", 10);
  // pub_angular_vel_tracker = nh.advertise<geometry_msgs::Vector3>("vive_vel_angular", 10);
  // pub_angular_acc_tracker = nh.advertise<geometry_msgs::Vector3>("vive_acc_angular", 10);

  pub_uav_state = nh.advertise<flypulator_common_msgs::UavStateStamped>("meas_state", 10);
  pub_uav_state_rpy = nh.advertise<flypulator_common_msgs::UavStateRPYStamped>("meas_state_rpy", 10);
  ROS_INFO("running");
  // ros::Subscriber sub_pose1 = nh.subscribe<nav_msgs::Odometry>("/vive/LHR_08DDEDC9_odom", 100, vive_odom_callback);
  ros::Subscriber sub_pose1 = nh.subscribe<nav_msgs::Odometry>("odom_msg", 100, vive_odom_callback);
  ros::spin();

  return 0;
}

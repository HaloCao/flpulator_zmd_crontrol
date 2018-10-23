#include "ros/ros.h"
#include "flypulator_highlevel_command/hl_command.h"
#include "flypulator_traj_generator/linear_trajectory.h"
#include "flypulator_common_msgs/UavStateStamped.h"
#include "tf/tf.h"

ros::ServiceClient traj_client;
geometry_msgs::Pose currPose;

bool executeCommandCB(flypulator_highlevel_command::hl_command::Request &req,
                      flypulator_highlevel_command::hl_command::Response &res)
{
  flypulator_traj_generator::linear_trajectory traj_srv;

  tf::Quaternion q(currPose.orientation.x, currPose.orientation.y,
                   currPose.orientation.z, currPose.orientation.w);
  tf::Matrix3x3 R(q);
  double roll, pitch, yaw;
  R.getRPY(roll, pitch, yaw);

  // set start pose(current pose)
  traj_srv.request.x_start.x = currPose.position.x;
  traj_srv.request.x_start.y = currPose.position.y;
  traj_srv.request.x_start.z = currPose.position.z;
  traj_srv.request.rpy_start.x = roll;
  traj_srv.request.rpy_start.y = pitch;
  traj_srv.request.rpy_start.z = yaw;

  double v_speed = 0.2; // vertical speed by take off and landing [m/s]

  traj_srv.request.delta_t = currPose.position.z/v_speed; // sec

  if ("take off" == req.command)
  {
    ROS_INFO_STREAM("Got: take off command!");
    // set end pose
    traj_srv.request.x_end.x = traj_srv.request.x_start.x;
    traj_srv.request.x_end.y = traj_srv.request.x_start.y;
    traj_srv.request.x_end.z = 1.5;
    traj_srv.request.rpy_end.x = 0;
    traj_srv.request.rpy_end.y = 0;
    traj_srv.request.rpy_end.z = 0;
  }

  if ("landing" == req.command)
  {
    ROS_INFO_STREAM("Got: landing command!");
    traj_srv.request.x_end.x = traj_srv.request.x_start.x;
    traj_srv.request.x_end.y = traj_srv.request.x_start.y;
    traj_srv.request.x_end.z = 0.2;
    traj_srv.request.rpy_end.x = 0;
    traj_srv.request.rpy_end.y = 0;
    traj_srv.request.rpy_end.z = 0;
  }

  ROS_INFO("Start position: x:%5.2f, y:%5.2f, z:%5.3f",
           currPose.position.x, currPose.position.y, currPose.position.z);
  ROS_INFO("Start attitude: roll:%5.2f, pitch:%5.2f, yaw:%5.3f",
           roll, pitch, yaw);

  if (!traj_client.call(traj_srv))
    ROS_ERROR("Failed to call service linear_trajectory");

  res.finished = true;

  return true;
}

// receive state estimation message
void poseMsgCB(const flypulator_common_msgs::UavStateStamped::ConstPtr &msg)
{
  currPose = msg->pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flypulator_highlevel_command");
  ros::NodeHandle nh;

  traj_client = nh.serviceClient<flypulator_traj_generator::linear_trajectory>("linear_trajectory");

  ros::Subscriber state_sub = nh.subscribe("/drone/meas_state", 1000, poseMsgCB);

  // register services
  ros::ServiceServer good_service = nh.advertiseService("hl_command_service", executeCommandCB);
  ROS_INFO("Service hl_command ready");

  ros::spin();
  return 0;
}

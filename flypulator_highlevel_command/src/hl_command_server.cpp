#include "ros/ros.h"
#include "flypulator_highlevel_command/hl_command.h"
#include "flypulator_traj_generator/linear_trajectory.h"
#include "flypulator_common_msgs/UavStateStamped.h"
#include "tf/tf.h"

ros::ServiceClient traj_client;
geometry_msgs::Pose currPose;

double calTrajTime(tf::Vector3 p_start, tf::Vector3 p_end, double track_speed)
{
  const double pure_rotation_track_time = 2;   // trajectory time when performing pure rotaion [s]
  const double pure_rotation_threshold = 0.10; //[m]
  double traj_distance = p_start.distance(p_end);

  if (traj_distance > pure_rotation_threshold)
    return traj_distance / track_speed; // [s]
  else                                  // ~ pure rotation
    return pure_rotation_track_time;    // [s]
}

void getPoseRPY(geometry_msgs::Pose pose, double &px, double &py, double &pz,
                double &roll, double &pitch, double &yaw)
{
  // convert quaternion to roll pitch yaw angle
  tf::Quaternion q(pose.orientation.x, pose.orientation.y,
                   pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 R(q);
  R.getRPY(roll, pitch, yaw);

  px = pose.position.x;
  py = pose.position.y;
  pz = pose.position.z;
}

void getPoseRPY(geometry_msgs::Pose pose, tf::Vector3 &pos, tf::Vector3 &rpy)
{
  // convert quaternion to roll pitch yaw angle
  tf::Quaternion q(pose.orientation.x, pose.orientation.y,
                   pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 R(q);
  double roll, pitch, yaw;
  R.getRPY(roll, pitch, yaw);

  pos = tf::Vector3(pose.position.x,pose.position.y,pose.position.z);
  rpy = tf::Vector3(roll, pitch, yaw);
}

bool gotoTarget(geometry_msgs::Pose curr_pose, tf::Vector3 target_pos,
                tf::Vector3 target_rpy, double track_speed)
{
  flypulator_traj_generator::linear_trajectory traj_srv;

  // set start pose(current pose)
  getPoseRPY(curr_pose, traj_srv.request.x_start.x, traj_srv.request.x_start.y, traj_srv.request.x_start.z,
             traj_srv.request.rpy_start.x, traj_srv.request.rpy_start.y, traj_srv.request.rpy_start.z);

  // end pose
  traj_srv.request.x_end.x = target_pos.x();
  traj_srv.request.x_end.y = target_pos.y();
  traj_srv.request.x_end.z = target_pos.z();

  traj_srv.request.rpy_end.x = target_rpy.x();
  traj_srv.request.rpy_end.y = target_rpy.y();
  traj_srv.request.rpy_end.z = target_rpy.z();

  // calculate the trajectory time
  tf::Vector3 p_start(traj_srv.request.x_start.x,
                      traj_srv.request.x_start.y,
                      traj_srv.request.x_start.z);
  tf::Vector3 p_end(traj_srv.request.x_end.x,
                    traj_srv.request.x_end.y,
                    traj_srv.request.x_end.z);
  traj_srv.request.delta_t = calTrajTime(p_start, p_end, track_speed);

  // call service
  if (!traj_client.call(traj_srv))
  {
    ROS_ERROR("Failed to call service linear_trajectory");
    return false;
  }
  else
    return true;
}

bool gotoTarget(tf::Vector3 start_pos, tf::Vector3 start_rpy, tf::Vector3 target_pos,
                tf::Vector3 target_rpy, double track_speed)
{
  flypulator_traj_generator::linear_trajectory traj_srv;

  // set start pose(current pose)
  traj_srv.request.x_start.x = start_pos.x();
  traj_srv.request.x_start.y = start_pos.y();
  traj_srv.request.x_start.z = start_pos.z();
  traj_srv.request.rpy_start.x = start_rpy.x();       
  traj_srv.request.rpy_start.y = start_rpy.y();  
  traj_srv.request.rpy_start.z = start_rpy.z();     

  // end pose
  traj_srv.request.x_end.x = target_pos.x();
  traj_srv.request.x_end.y = target_pos.y();
  traj_srv.request.x_end.z = target_pos.z();

  traj_srv.request.rpy_end.x = target_rpy.x();
  traj_srv.request.rpy_end.y = target_rpy.y();
  traj_srv.request.rpy_end.z = target_rpy.z();

  // calculate the trajectory time
  tf::Vector3 p_start(traj_srv.request.x_start.x,
                      traj_srv.request.x_start.y,
                      traj_srv.request.x_start.z);
  tf::Vector3 p_end(traj_srv.request.x_end.x,
                    traj_srv.request.x_end.y,
                    traj_srv.request.x_end.z);
  traj_srv.request.delta_t = calTrajTime(p_start, p_end, track_speed);

  // call service
  if (!traj_client.call(traj_srv))
  {
    ROS_ERROR("Failed to call service linear_trajectory");
    return false;
  }
  else
    return true;
}

bool executeCommandCB(flypulator_highlevel_command::hl_command::Request &req,
                      flypulator_highlevel_command::hl_command::Response &res)
{
  const double v_speed = 0.3;          // vertical speed by take off and landing [m/s]
  const double track_speed = 0.4;      // speed while using 'go to' command [m/s]
  const double landing_heigth = 0.2; // position z when the drone is landed [m]
  const double take_off_height = 1.5;  // position z when the drone is hovering [m]

  tf::Vector3 target_pos, target_rpy;
  tf::Vector3 curr_pos, curr_rpy;
  tf::Vector3 start_pos, start_rpy;

  // ROS_INFO("Start position: x:%5.2f, y:%5.2f, z:%5.3f",
  //          currPose.position.x, currPose.position.y, currPose.position.z);
  // ROS_INFO("Start attitude: roll:%5.2f, pitch:%5.2f, yaw:%5.3f",
  //          roll, pitch, yaw);

  // command: take off
  if ("take off" == req.command)
  {
    ROS_INFO_STREAM("Got: take off command!");
    getPoseRPY(currPose, curr_pos, curr_rpy);
    target_pos = curr_pos;
    target_pos.setZ(take_off_height);
    target_rpy = tf::Vector3(0, 0, curr_rpy.z());
    gotoTarget(currPose, target_pos, target_rpy, v_speed);
  }
  // command: landing
  else if ("landing" == req.command)
  {
    ROS_INFO_STREAM("Got: landing command!");
    getPoseRPY(currPose, curr_pos, curr_rpy);
    target_pos = curr_pos;
    target_pos.setZ(landing_heigth);
    target_rpy = tf::Vector3(0, 0, curr_rpy.z());
    gotoTarget(currPose, target_pos, target_rpy, v_speed);
  }
  // command: go to
  else if ("go to" == req.command)
  {
    ROS_INFO_STREAM("Got: go to command!");
    target_pos = tf::Vector3(req.x_end.x, req.x_end.y, req.x_end.z);
    target_rpy = tf::Vector3(req.rpy_end.x, req.rpy_end.y, req.rpy_end.z);
    gotoTarget(currPose, target_pos, target_rpy, track_speed);
  }
  // command: go home
  // TODO: the current pose did not update during this callback function 
  else if ("go home" == req.command)
  {
    ROS_INFO_STREAM("Got: go home command!");
    // delay 2s
    ros::Duration duration(2);
    // Step 1:
    getPoseRPY(currPose, curr_pos, curr_rpy);
    // ROS_INFO("Start position: x:%5.2f, y:%5.2f, z:%5.3f",
    //          curr_pos.x(), curr_pos.y(), curr_pos.z());
    // ROS_INFO("Start attitude: roll:%5.2f, pitch:%5.2f, yaw:%5.3f",
            //  curr_rpy.x(), curr_rpy.y(), curr_rpy.z());
    target_pos = curr_pos;
    target_pos.setZ(take_off_height);
    target_rpy = tf::Vector3(0, 0, 0);
    gotoTarget(currPose, target_pos, target_rpy, track_speed);

    duration.sleep(); // delay 2s

    // Step 2:
    // getPoseRPY(currPose, curr_pos, curr_rpy);
    // ROS_INFO("Start position: x:%5.2f, y:%5.2f, z:%5.3f",
    //          curr_pos.x(), curr_pos.y(), curr_pos.z());
    // ROS_INFO("Start attitude: roll:%5.2f, pitch:%5.2f, yaw:%5.3f",
            //  curr_rpy.x(), curr_rpy.y(), curr_rpy.z());
    start_pos = target_pos;
    start_rpy = target_rpy;
    target_pos = tf::Vector3(0,0,take_off_height);
    target_rpy = tf::Vector3(0, 0, 0);
    gotoTarget(start_pos, start_rpy, target_pos, target_rpy, track_speed);
    
    duration.sleep(); // delay 2s

    // Step 3:
    // getPoseRPY(currPose, curr_pos, curr_rpy);
    // ROS_INFO("Start position: x:%5.2f, y:%5.2f, z:%5.3f",
    //          curr_pos.x(), curr_pos.y(), curr_pos.z());
    // ROS_INFO("Start attitude: roll:%5.2f, pitch:%5.2f, yaw:%5.3f",
            //  curr_rpy.x(), curr_rpy.y(), curr_rpy.z());
    start_pos = target_pos;
    start_rpy = target_rpy;
    target_pos = tf::Vector3(0,0,landing_heigth);
    target_rpy = tf::Vector3(0, 0, 0);
    gotoTarget(start_pos, start_rpy, target_pos, target_rpy, v_speed);
  }

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

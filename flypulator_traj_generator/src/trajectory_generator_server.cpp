#include "ros/ros.h"
#include "trajectory_generator.h"
// add additional service headers here

#include "flypulator_traj_generator/linear_trajectory.h"
#include "flypulator_traj_generator/polynomial_trajectory.h"

// global variable for trajectory generator class object
TrajectoryGenerator *g_generator_p;

// create polynomial trajectory from request and give response
bool createPolynomialTrajectoryCB(flypulator_traj_generator::polynomial_trajectory::Request &req,
                                  flypulator_traj_generator::polynomial_trajectory::Response &res)
{
  // calculate trajectory, retrieve evolution of positional and rotational accelerations for feasibility check, and if
  // demanded, publish it
  trajectory::pos_accelerations pos_accelerations;
  trajectory::euler_angle_accelerations euler_angle_accelerations;
  trajectory::euler_angle_velocities euler_angle_velocities;
  trajectory::euler_angles eulerAngles;
  geometry_msgs::Vector3 eulerAxis;
  std::vector<double> time_stamps;

  res.finished = g_generator_p->createAndSendTrajectory(req.p_start, req.p_end, req.rpy_start, req.rpy_end,
                                                        req.duration, req.start_tracking, trajectory_types::Polynomial,
                                                        pos_accelerations, euler_angle_accelerations,
                                                        euler_angle_velocities, eulerAngles, eulerAxis, time_stamps);
  // fill result
  res.p_acc = pos_accelerations;
  res.euler_angle_acc = euler_angle_accelerations;
  res.euler_angle_vel = euler_angle_velocities;
  res.euler_angle = eulerAngles;
  res.euler_axis = eulerAxis;
  res.time_stamps = time_stamps;

  return true;
}

// create linear trajectory from request and give response
bool createLinearTrajectoryCB(flypulator_traj_generator::linear_trajectory::Request &req,
                              flypulator_traj_generator::linear_trajectory::Response &res)
{
  // calculate trajectory, retrieve evolution of positional and rotational accelerations for feasibility check, and if
  // demanded, publish it
  trajectory::pos_accelerations pos_accelerations;
  trajectory::euler_angle_accelerations euler_angle_accelerations;
  trajectory::euler_angle_velocities euler_angle_velocities;
  trajectory::euler_angles eulerAngles;
  geometry_msgs::Vector3 eulerAxis;
  std::vector<double> time_stamps;
  res.finished = g_generator_p->createAndSendTrajectory(
      req.p_start, req.p_end, req.rpy_start, req.rpy_end, req.duration, req.start_tracking, trajectory_types::Linear,
      pos_accelerations, euler_angle_accelerations, euler_angle_velocities, eulerAngles, eulerAxis, time_stamps);

  // fill result
  res.p_acc = pos_accelerations;
  res.euler_angle_acc = euler_angle_accelerations;
  res.euler_angle_vel = euler_angle_velocities;
  res.euler_angle = eulerAngles;
  res.euler_axis = eulerAxis;
  res.time_stamps = time_stamps;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_generator");  // pass node name (!)
  ros::NodeHandle n;
  ros::Publisher trajectory_publisher;
  ros::Publisher traj_visualization_pub_;

  // register services
  ros::ServiceServer serviceLinTraj = n.advertiseService("linear_trajectory", createLinearTrajectoryCB);
  ROS_INFO("Service linear_trajectory ready");
  ros::ServiceServer servicePolTraj = n.advertiseService("polynomial_trajectory", createPolynomialTrajectoryCB);
  ROS_INFO("Service polynomial_trajectory ready");

  // register publisher for output message
  trajectory_publisher = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("trajectory", 1000);

  // register publisher for visualization
  traj_visualization_pub_ = n.advertise<geometry_msgs::PoseArray>("/trajectory/visualization", 1000);
  ;

  // create trajectory generator class object
  TrajectoryGenerator m_generator(trajectory_publisher, traj_visualization_pub_);
  g_generator_p = &m_generator;  // set pointer for global variable

  ros::spin();  // keep server alive and watch for requests

  return 0;
}

#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "ros/ros.h"

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

// include message structs
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

#include <eigen3/Eigen/Dense>  //TODO Why does <Eigen/Dense> not work??
#include <math.h>              //sin,cos functions, M_PI

namespace trajectory_types  // own namespace for enumeration following style guide http://wiki.ros.org/CppStyleGuide
{
enum Type
{
  Linear,
  Polynomial
  // add new trajectory types here
};
};

namespace trajectory {
    typedef std::vector<geometry_msgs::Vector3> accelerations;
}

class TrajectoryGenerator
{
public:  // constructor takes publisher to publish message
  TrajectoryGenerator(ros::Publisher& pub)
  {
    trajectory_publisher_ = pub;
  }

  // create and send trajectory with pose estimation frequency
  bool createAndSendTrajectory(const geometry_msgs::Vector3& p_start, const geometry_msgs::Vector3& p_end,
                               const geometry_msgs::Vector3& rpy_start, const geometry_msgs::Vector3& rpy_end,
                               const float duration, const bool start_tracking, const trajectory_types::Type traj_type,
                               trajectory::accelerations &pos_accs, trajectory::accelerations &rot_accs);

private:
  // message publisher for output trajectory, needs to be global to be visible to create<..>Trajectory functions
  ros::Publisher trajectory_publisher_;
  // create trajectory message
  trajectory_msgs::MultiDOFJointTrajectoryPoint generateTrajectoryMessage(
      const float p[6], const float p_dot[6], const float p_ddot[6], const Eigen::Quaternionf& q,
      const Eigen::Vector3f& omega, const Eigen::Vector3f& omega_dot, const ros::Duration& time_from_start);

  // calculate angular velocity from euler angles and its derivatives, following Fje94 p.42
  void calculateOmega(const float roll, const float roll_dot, const float pitch, const float pitch_dot, const float yaw,
                      const float yaw_dot, Eigen::Vector3f& omega);
  // calculate angular acceleration from euler angles and its derivatives, following Fje94 p.42 derivated
  void calculateOmegaDot(const float roll, const float roll_dot, const float roll_ddot, const float pitch,
                         const float pitch_dot, const float pitch_ddot, const float yaw, const float yaw_dot,
                         const float yaw_ddot, Eigen::Vector3f& omega_dot);
  // convert Euler angles to quaternions using roll-pitch-yaw sequence
  void euler2Quaternion(const float roll, const float pitch, const float yaw, Eigen::Quaternionf& q);
  // convert 2 messages of Vector3 type to 6D float array
  void convertTo6DArray(const geometry_msgs::Vector3& x_1, const geometry_msgs::Vector3& x_2, float destination[]);
  // evaluate a polynom at a given time
  inline float evaluatePolynom (float a0, float a1, float a2, float a3, float a4, float a5, float t);
};

#endif  // TRAJECTORY_GENERATOR_H

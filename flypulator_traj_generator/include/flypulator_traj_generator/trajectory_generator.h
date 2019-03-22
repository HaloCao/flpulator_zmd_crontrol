#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "ros/ros.h"

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseArray.h"

// include message structs
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

#include <Eigen/Dense>  //matrix, geometry
#include <math.h>       //sin,cos functions, M_PI

namespace trajectory_types  // own namespace for enumeration following style guide http://wiki.ros.org/CppStyleGuide
{
enum Type
{
  Linear,
  Polynomial
  // add new trajectory types here
};
};

namespace trajectory
{
typedef std::vector<geometry_msgs::Vector3> pos_accelerations;
typedef std::vector<double> euler_angle_accelerations;
typedef std::vector<double> euler_angle_velocities;
typedef std::vector<double> euler_angles;
}  // namespace trajectory

class TrajectoryGenerator
{
public:  // constructor takes publisher to publish message
  TrajectoryGenerator(ros::Publisher& traj_pub, ros::Publisher& visu_pub)
  {
    trajectory_publisher_ = traj_pub;
    traj_visualization_pub_ = visu_pub;
  }

  /**
   * \brief createAndSendTrajectory Create trajectories and send it with control frequency if desired
   * \param p_start Start position
   * \param p_end Target position
   * \param rpy_start Start attitude in RPY
   * \param rpy_end Target attitude in RPY
   * \param duration Time duration of the trajectory
   * \param start_tracking Flag if the created trajectory will be sent
   * \param traj_type Trajectory type
   * \param others other params are for visualization purpose
   * \return True if Trajectory created successfully
   */
  bool createAndSendTrajectory(const geometry_msgs::Vector3& p_start, const geometry_msgs::Vector3& p_end,
                               const geometry_msgs::Vector3& rpy_start, const geometry_msgs::Vector3& rpy_end,
                               const float duration, const bool start_tracking, const trajectory_types::Type traj_type,
                               trajectory::pos_accelerations& pos_accs,
                               trajectory::euler_angle_accelerations& euler_angle_accs,
                               trajectory::euler_angle_velocities& euler_angle_vels,
                               trajectory::euler_angles& euler_angles, geometry_msgs::Vector3& eulerAxis,
                               std::vector<double>& time_stamps);

private:
  // message publisher for output trajectory, needs to be global to be visible to create<..>Trajectory functions
  ros::Publisher trajectory_publisher_;
  // publisher for visualization purpose
  ros::Publisher traj_visualization_pub_;

  /**
   * \brief generateTrajectoryMessage Create trajectory messages
   */
  trajectory_msgs::MultiDOFJointTrajectoryPoint generateTrajectoryMessage(
      const float p[6], const float p_dot[6], const float p_ddot[6], const Eigen::Quaternionf& q,
      const Eigen::Vector3f& omega, const Eigen::Vector3f& omega_dot, const ros::Duration& time_from_start);

  /**
   * \brief evaluateAcceleration Evaluate the acceleration polynom at a given time
   * \return Desired acceleration at time t
   */
  inline float evaluateAcceleration(float a2, float a3, float a4, float a5, float t);

  /**
   * \brief evaluateVelocity Evaluate the velocity polynom at a given time
   * \return Desired velocity at time t
   */
  inline float evaluateVelocity(float a1, float a2, float a3, float a4, float a5, float t);

  /**
   * \brief evaluatePosition Evaluate the position polynom at a given time
   * \return Desired position at time t
   */
  inline float evaluatePosition(float a0, float a1, float a2, float a3, float a4, float a5, float t);

  /**
   * \brief rpyToRotMat convert RPY-Angles to rotation matrix
   * \param roll Roll angle
   * \param pitch Pitch angle
   * \param yaw Yaw angle
   * \param rotMatrix Rotation matrix converted from RPY
   */
  inline void rpyToRotMat(float roll, float pitch, float yaw, Eigen::Matrix3f& rotMatrix);

  /**
   * \brief extractStartAndTargetPose extract start and target position and extract corresponding euler parameters
   * \param p_start Input, start position
   * \param p_end Input, target position
   * \param rpy_start Input, start attitude in RPY
   * \param rpy_end Input, target attitude in RPY
   * \param pose_start[] Output, start value for position and angle of Euler parameter
   * \param pose_end[] Output, target value for position and angle of Euler parameter
   * \param euler_param Output, Euler paramter derived from start and end attitude
   * \param q_start Output, start attitude
   */
  void extractStartAndTargetPose(const geometry_msgs::Vector3& p_start, const geometry_msgs::Vector3& p_end,
                                 const geometry_msgs::Vector3& rpy_start, const geometry_msgs::Vector3& rpy_end,
                                 float pose_start[], float pose_end[], Eigen::AngleAxisf& euler_param,
                                 Eigen::Quaternionf& q_start);

  /**
   * \brief angularVelocityFromEulerParams get angular velocity and acceleration from euler angle and its derivatives
   * \param q_IA  Quaternion
   * \param kA  Euler axis
   * \param the Euler angle
   * \param dthe Derivative of Euler angle
   * \param ddthe Second order derivative of Euler angle
   * \param omeg Angular velocity
   * \param omeg_dot Derivative of angular velocity
   */
  void angularVelocityFromEulerParams(Eigen::Quaternionf q_IA, Eigen::Vector3f kA, float the, float dthe, float ddthe,
                                      Eigen::Vector3f& omeg, Eigen::Vector3f& omeg_dot);
};

#endif  // TRAJECTORY_GENERATOR_H

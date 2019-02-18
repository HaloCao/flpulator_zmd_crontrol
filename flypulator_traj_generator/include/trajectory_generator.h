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

namespace Eigen
{
struct EulerParams
{
  Eigen::Vector3f axis;
  float angle;
};
};  // namespace Eigen

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

  // create and send trajectory with pose estimation frequency
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
  // create trajectory message
  trajectory_msgs::MultiDOFJointTrajectoryPoint generateTrajectoryMessage(
      const float p[6], const float p_dot[6], const float p_ddot[6], const Eigen::Quaternionf& q,
      const Eigen::Vector3f& omega, const Eigen::Vector3f& omega_dot, const ros::Duration& time_from_start);

  // evaluate acceleration polynom at a given time
  inline float evaluateAcceleration(float a2, float a3, float a4, float a5, float t);

  // evaluate velocity polynom at a given time
  inline float evaluateVelocity(float a1, float a2, float a3, float a4, float a5, float t);

  // evaluate position polynom at a given time
  inline float evaluatePosition(float a0, float a1, float a2, float a3, float a4, float a5, float t);

  // calculate euler parameters
  inline void calculateEulerParameters(Eigen::Matrix3f rotMat, Eigen::EulerParams& eulerParams);

  // convert RPY-Angles to discrete cosine matrix
  inline void rpyToRotMat(float roll, float pitch, float yaw, Eigen::Matrix3f& rotMatrix);

  // extract start and target position and extract corresponding euler parameters
  void extractStartAndTargetPose(const geometry_msgs::Vector3& p_start, const geometry_msgs::Vector3& p_end,
                                 const geometry_msgs::Vector3& rpy_start, const geometry_msgs::Vector3& rpy_end,
                                 float pose_start[], float pose_end[], Eigen::EulerParams& eulerParams,
                                 Eigen::Quaternionf& q_start);

  // convert euler parameters to quaternion
  void eulerParamsToQuat(Eigen::Vector3f euler_axis, float euler_angle, Eigen::Quaternionf& q_AB);

  // get angular velocity and acceleration from euler angle and its derivatives
  void angularVelocityFromEulerParams(Eigen::Quaternionf q_IA, Eigen::Vector3f kA, float the, float dthe, float ddthe,
                                      Eigen::Vector3f& omeg, Eigen::Vector3f& omeg_dot);
};

#endif  // TRAJECTORY_GENERATOR_H

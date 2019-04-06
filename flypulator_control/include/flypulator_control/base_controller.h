#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include "flypulator_control/pid_parameterConfig.h"

/**
 * \struct The PoseVelocityAcceleration struct
 * \brief The PoseVelocityAcceleration struct defines UAV states, The only method print out all the states.
 */
struct PoseVelocityAcceleration
{
  Eigen::Vector3f p;
  Eigen::Quaternionf q;
  Eigen::Vector3f p_dot;
  Eigen::Vector3f omega;
  Eigen::Vector3f p_ddot;
  Eigen::Vector3f omega_dot;
  // initialize to zero (except quaternion) with default constructor
  PoseVelocityAcceleration()
    : p(Eigen::Vector3f(0, 0, 0))
    , q(Eigen::Quaternionf(1, 0, 0, 0))
    , p_dot(Eigen::Vector3f(0, 0, 0))
    , omega(Eigen::Vector3f(0, 0, 0))
    , p_ddot(Eigen::Vector3f(0, 0, 0))
    , omega_dot(Eigen::Vector3f(0, 0, 0))
  {
  }
  void printToROSINFO()
  {
    ROS_INFO("pose: \n \t x \t\t = [%f, %f, %f], \n \t x_dot \t\t = [%f, %f, %f], \n \t x_ddot \t = [%f, %f, %f], \n "
             "\t q \t\t = [%f, %f, %f, %f], \n \t omega \t\t = [%f, %f, %f], \n \t omega_dot \t = [%f, %f, %f]",
             p.x(), p.y(), p.z(), p_dot.x(), p_dot.y(), p_dot.z(), p_ddot.x(), p_ddot.y(), p_ddot.z(), q.w(), q.x(),
             q.y(), q.z(), omega.x(), omega.y(), omega.z(), omega_dot.x(), omega_dot.y(), omega_dot.z());
  }
};

/**
 * \class The BaseController class
 * \brief The BaseController class is the Superclass for all controller types, universal variable and methodes are
 * defined within this class. It is a abstract class, no objects from this class allowed, following
 * http://cpp.nope.bz/pure_virtual.html
 */
class BaseController
{
public:
  virtual ~BaseController()
  {
  }

  /**
   * \brief computeControlForceTorqueInput Calculates the desired force and torque according to the control law
   * \param x_des contains the desired pose vel and acc
   * \param x_current contains the feedback
   * \param control_force_and_torque desired body wrench as result of the control law
   */
  virtual void computeControlForceTorqueInput(const PoseVelocityAcceleration& x_des,
                                              const PoseVelocityAcceleration& x_current,
                                              Eigen::Matrix<float, 6, 1>& control_force_and_torque) = 0;

  /**
   * \brief configCallback is callback of the dynamic reconfiguration. It sets all dynamically reconfigurable parameters
   * \param config contains the new parameters
   */
  virtual void configCallback(flypulator_control::pid_parameterConfig& config, uint32_t level) = 0;
};

#endif  // BASE_CONTROLLER_H

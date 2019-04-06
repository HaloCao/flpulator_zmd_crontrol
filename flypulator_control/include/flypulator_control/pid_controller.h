#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "flypulator_control/base_controller.h"

// declare global variable
extern bool g_controller_enabled;

/**
 * \class The PidController class
 * \brief The PidController class implements the PID controller with exact feedback linearization There is basically one
 * central method which takes the desired and current value to calculate the desired force and torque with the control
 * law. Controller parameters are dynamically reconfigurable during the runtime. The controller is disabled by default.
 */
class PidController : public BaseController
{
public:
  // initialize class variables in constructor with initialization list as best practise
  PidController(const float mass, const Eigen::Matrix3f inertia, const float gravity)
    : mass_(mass)
    , inertia_(inertia)
    , inertia_inv_(inertia.inverse())
    , gravity_(Eigen::Vector3f(0, 0, gravity))
    , u_T_(Eigen::Vector3f(0, 0, 0))
    , u_R_(Eigen::Vector3f(0, 0, 0))
  {
    // initialize t_last_
    t_last_ = ros::Time::now();
    ROS_DEBUG("t_last initialized as %f", t_last_.toSec());
  }

  /**
   * \brief computeControlForceTorqueInput is an overload function which calculates the desired force and torque
   * according to the control law
   * \param x_des contains the desired pose vel and acc
   * \param x_current contains the feedback
   * \param control_force_and_torque desired body wrench as result of the control law
   */
  void computeControlForceTorqueInput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current,
                                      Eigen::Matrix<float, 6, 1>& control_force_and_torque);

  /**
   * \brief configCallback is callback of the dynamic reconfiguration. It sets all dynamically reconfigurable parameters
   * \param config contains the new parameters
   */
  void configCallback(flypulator_control::pid_parameterConfig& config, uint32_t level);

private:
  float mass_;
  float pid_I_max_T_;
  float pid_I_max_R_;
  Eigen::Matrix3f inertia_;
  Eigen::Matrix3f inertia_inv_;
  Eigen::Vector3f gravity_;
  // translational variables

  Eigen::Matrix3f K_T_P_;
  Eigen::Matrix3f K_T_I_;
  Eigen::Matrix3f K_T_D_;
  Eigen::Matrix3f K_R_P_;
  Eigen::Matrix3f K_R_I_;
  Eigen::Matrix3f K_R_D_;

  Eigen::Vector3f v_T_;
  Eigen::Vector3f v_R_;

  Eigen::Vector3f u_T_;
  Eigen::Vector3f u_R_;
  Eigen::Matrix<float, 6, 6> J;
  Eigen::Matrix<float, 6, 1> h;
  Eigen::Matrix3f I;

  Eigen::Vector3f p_err_;
  Eigen::Vector3f p_last_err_;
  Eigen::Vector3f p_dot_err_;
  Eigen::Vector3f p_integral_err_;
  Eigen::Matrix3f rotation_err_skew_;
  Eigen::Vector3f rotation_err_;
  Eigen::Vector3f rotation_integral_err_;
  Eigen::Vector3f omega_err_;

  // time variables
  ros::Time t_current_;
  ros::Time t_last_;
  ros::Duration t_delta_;

  // float anti_windup_threshold_T;
  // float anti_windup_threshold_R;
};

#endif  // PID_CONTROLLER_H

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "flypulator_control/base_controller.h"

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
    // initialize t_last_ with assumed sampling time (200Hz).
    t_last_ = ros::Time::now() - ros::Duration(0.005);
  }

  // compute Control Force and Torque
  void computeControlForceTorqueInput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current,
                                      Eigen::Matrix<float, 6, 1>& control_force_and_torque);

  // callback for dynamic reconfigure, sets dynamic parameters (controller gains)
  void configCallback(flypulator_control::pid_parameterConfig& config, uint32_t level);

private:
  float mass_;
  float pid_I_max_T;
  float pid_I_max_R;
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

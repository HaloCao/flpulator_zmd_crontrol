#ifndef SLIDING_MODE_CONTROLLER_H
#define SLIDING_MODE_CONTROLLER_H

#include "flypulator_control/base_controller.h"
#include <flypulator_control/ism_parameterConfig.h>

// declare global variable
extern bool g_controller_enabled;

/**
 * \class The SlidingModeController class
 * \brief The SlidingModeController class implements the Sliding Mode Control. There is basically one
 * central method which takes the desired and current value to calculate the desired force and torque with the control
 * law. Controller parameters are dynamically reconfigurable during the runtime. The controller is disabled by default.
 */
class SlidingModeController : public BaseController
{
public:
  // initialize class variables in constructor with initialization list as best practise
  SlidingModeController(const float mass, const Eigen::Matrix3f inertia, const float gravity)
    : mass_(mass)
    , inertia_(inertia)
    , inertia_inv_(inertia.inverse())
    , gravity_(Eigen::Vector3f(0, 0, gravity))
    , integral_T_(Eigen::Vector3f(0, 0, 0))
    , integral_R_(Eigen::Vector4f(0, 0, 0, 0))
    , control_started_(false)
    , s_T_I_(Eigen::Vector3f(0, 0, 0))
    , u_T_I_(Eigen::Vector3f(0, 0, 0))
    , s_R_I_(Eigen::Vector4f(0, 0, 0, 0))
    , u_R_I_(Eigen::Vector3f(0, 0, 0))

  {
    // initialize t_last_ with assumed sampling time (200Hz).
    t_last_ = ros::Time::now() - ros::Duration(0.005);
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
  void configCallback(flypulator_control::ism_parameterConfig& config, uint32_t level);

  float getDeadband();

private:
  // bool controller_enabled_;
  float mass_;
  Eigen::Matrix3f inertia_;
  Eigen::Matrix3f inertia_inv_;
  Eigen::Vector3f gravity_;
  // translational variables
  float lambda_T_;
  Eigen::Matrix3f K_T_;
  Eigen::Matrix3f K_T_I_;
  Eigen::Vector3f z_1_T_;
  Eigen::Vector3f z_2_T_;
  Eigen::Vector3f s_T_;
  Eigen::Vector3f s_T_I_;
  Eigen::Vector3f integral_T_;
  Eigen::Vector3f u_T_;
  Eigen::Vector3f u_T_I_;
  // rotational variables
  float lambda_R_;
  float eta_;
  float eta_d_;
  float eta_err_;
  float eta_dot_err_;
  Eigen::Matrix4f K_R_;
  Eigen::Vector3f eps_;
  Eigen::Vector3f eps_d_;
  Eigen::Vector3f eps_err_;
  Eigen::Vector3f eps_dot_err_;

  Eigen::Vector4f z_1_R_;
  Eigen::Vector3f omega_err_;
  Eigen::Matrix<float, 4, 3> matrix_g_transposed_;
  Eigen::Vector4f z_2_R_;
  Eigen::Matrix<float, 4, 3> matrix_g_dot_transposed_;
  Eigen::Vector4f s_R_;
  Eigen::Vector3f u_R_;

  Eigen::Vector4f integral_R_;
  Eigen::Matrix3f K_R_I_;
  Eigen::Vector4f s_R_I_;
  Eigen::Vector3f u_R_I_;

  // time variables
  ros::Time t_last_;
  ros::Time t_current_;
  ros::Duration t_delta_;
  bool control_started_;
  float omega_deadband_;
  float anti_windup_threshold_T;
  float anti_windup_threshold_R;
  // sign function without zero (sgn(x) <0 or sgn(x) > 0, there is NO x such that sgn(x) = 0)
  float sgn(float x)
  {
    if (x >= 0.0f)
    {
      return 1.0f;
    }
    else
    {
      return -1.0f;
    }
  }
};

#endif  // SLIDING_MODE_CONTROLLER_H

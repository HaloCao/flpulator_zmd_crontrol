#ifndef ZERO_MOMENT_DIRECTION_CONTROLLER_H
#define ZERO_MOMENT_DIRECTION_CONTROLLER_H

#include "flypulator_zmd_control/base_controller.h"
#include <flypulator_zmd_control/zmd_parameterConfig.h>

class ZeroMomentDirectionController : public BaseController
{
public:
  // initialize class variables in constructor with initialization list as best practise
  ZeroMomentDirectionController(const float mass, const Eigen::Matrix3f inertia, const float gravity)
    : mass_(mass)
    , inertia_(inertia)
    , inertia_inv_(inertia.inverse())
    , gravity_(Eigen::Vector3f(0, 0, gravity))
	  , f_(47.0f)
	  , q_d_(Eigen::Quaternionf(1, 0, 0, 0))
	  , e3_(Eigen::Vector3f(0,0,1))
    ,eps_err_sum_(Eigen::Vector3f(0,0,0))
    ,e_p_sum_(Eigen::Vector3f(0,0,0))
    //, purpose_position_(Eigen::Vector3f(1.2,1.5,1.8))
    //, purpose_velocity_(Eigen::Vector3f(0,0,0))
  {
    // initialize t_last_ with assumed sampling time (200Hz).
    t_last_ = ros::Time::now() ;
  }

  // compute Control Force and Torque
  void computeControlForceTorqueInput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current,
                                      Eigen::Matrix<float, 6, 1>& control_force_and_torque);

  // callback for dynamic reconfigure, sets dynamic parameters (controller gains)
  void configCallback(flypulator_zmd_control::zmd_parameterConfig& config, uint32_t level);

private:
  //Eigen::Vector3f purpose_position_;
  //Eigen::Vector3f purpose_velocity_;
  float mass_;
  Eigen::Matrix3f inertia_;
  Eigen::Matrix3f inertia_inv_;
  Eigen::Vector3f gravity_;
  // translational variables
  Eigen::Vector3f e_p_;
  Eigen::Vector3f e_p_sum_;
  Eigen::Vector3f e_v_;
  Eigen::Vector3f f_r_;
  Eigen::Vector3f e3_;
  float K_pp_;
  float K_pi_;
  float K_pd_;
  Eigen::Vector3f f_del_;
  float f_;
  Eigen::Vector3f _d_;
  float d_norm_;
  Eigen::Vector3f d_;
  Eigen::Matrix3f skew_d_;
  Eigen::Vector3f u_z_;
  float K_del_;
  float f_dot_;
  Eigen::Quaternionf q_d_;
  Eigen::Matrix3f skew_eps_d_;
  float eta_dot_;
  Eigen::Vector3f eps_dot_;
  float eta_;
  float eta_d_;
  float eta_r_;
  Eigen::Vector3f eps_;
  Eigen::Vector3f eps_d_;
  Eigen::Vector3f eps_r_;
  Eigen::Vector3f eps_err_;
  Eigen::Vector3f eps_rd_err_;
  Eigen::Vector3f eps_err_sum_;
  Eigen::Vector3f w_rd_;
  Eigen::Vector3f w_d_;
  float K_q_;
  float K_ap_;
  float K_ai_;
  float K_ad_;

  Eigen::Vector3f tau_r_;


  // time variables
  ros::Time t_last_;
  ros::Time t_current_;
  ros::Duration t_delta_;
  
  
};

#endif  // ZERO_MOMENT_DIRECTION_CONTROLLER_H

#include "flypulator_control/pid_controller.h"

// callback for dynamic reconfigure, sets dynamic parameters (controller gains)
void PidController::configCallback(flypulator_control::pid_parameterConfig& config, uint32_t level)
{
  // set logger level
  //   if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  //     ros::console::notifyLoggerLevelsChanged();

  ROS_INFO("Reconfigure Request: \n k_T_P = %f, \n k_T_I \t  = %f, \n k_T_D \t  = %f, \n k_R_P "
           "\t  = %f, \n k_R_I = %f, \n k_R_D  =%f",
           config.pid_k_T_P, config.pid_k_T_I, config.pid_k_T_D, config.pid_k_R_P, config.pid_k_R_I, config.pid_k_R_D);
  ROS_INFO("Reconfigure Request: \n k_T_P_z = %f, \n k_T_I_z \t  = %f, \n k_T_D_z \t  = %f, \n", config.pid_k_T_P_z,
           config.pid_k_T_I_z, config.pid_k_T_D_z);
  // set new values to class variables

  pid_I_max_T = float(config.pid_I_max_T);
  pid_I_max_R = float(config.pid_I_max_R);
  K_T_P_ << config.pid_k_T_P, 0, 0, 0, config.pid_k_T_P, 0, 0, 0, config.pid_k_T_P_z;
  K_T_I_ << config.pid_k_T_I, 0, 0, 0, config.pid_k_T_I, 0, 0, 0, config.pid_k_T_I_z;
  K_T_D_ << config.pid_k_T_D, 0, 0, 0, config.pid_k_T_D, 0, 0, 0, config.pid_k_T_D_z;
  K_R_P_ << config.pid_k_R_P, 0, 0, 0, config.pid_k_R_P, 0, 0, 0, config.pid_k_R_P_yaw;
  K_R_I_ << config.pid_k_R_I, 0, 0, 0, config.pid_k_R_I, 0, 0, 0, config.pid_k_R_I_yaw;
  K_R_D_ << config.pid_k_R_D, 0, 0, 0, config.pid_k_R_D, 0, 0, 0, config.pid_k_R_D_yaw;
}

// compute control force and torque from desired and current pose
void PidController::computeControlForceTorqueInput(const PoseVelocityAcceleration& x_des,
                                                   const PoseVelocityAcceleration& x_current,
                                                   Eigen::Matrix<float, 6, 1>& control_force_and_torque)
{
  ROS_DEBUG("Pid Mode Controller calculates control force and torque..");

  /*std::cout << "desired pose is :" << std::endl << x_des.p << std::endl;
   std::cout << "desired velocity is :" << std::endl << x_des.p_dot << std::endl;
   std::cout << "desired acceleration is :" << std::endl << x_des.p_ddot << std::endl;
   std::cout << "desired quaternion is :" << std::endl << x_des.q.coeffs() << std::endl;
   std::cout << "current rotation is :" << std::endl << x_current.q.coeffs() << std::endl;
   std::cout << "current pose is :" << std::endl << x_current.p << std::endl;
   std::cout << "current rotation is :" << std::endl << x_current.q.coeffs() << std::endl;
   std::cout << "current omega is :" << std::endl << x_current.omega << std::endl;
*/
  // compute force and torque
  // nonlinear term J and h
  J = Eigen::Matrix<float, 6, 6>::Zero();
  I = Eigen::Matrix<float, 3, 3>::Identity();
  J.block(0, 0, 3, 3) = 1 / mass_ * I;
  J.block(3, 3, 3, 3) = inertia_inv_;

  // std::cout << "J:" << J << std::endl;
  // calculate h
  Eigen::Matrix<float, 6, 1> h;
  h.block(0, 0, 3, 1) = -gravity_;
  h.block(3, 0, 3, 1) = -inertia_inv_ * (x_current.omega.cross(inertia_ * x_current.omega));
  // std::cout << "H is :" << h << std::endl;
  // ROS_DEBUG("h:", h.block(0, 0, 3, 1), h.block(3, 0, 3, 1));

  // calculate pose error
  p_err_ = x_current.p - x_des.p;
  // std::cout << "p_err_ is :" << std::endl << p_err_ << std::endl;

  Eigen::Matrix3f rotation_des(x_des.q);
  Eigen::Matrix3f rotation_current(x_current.q);
  // std::cout << "rotation desired: " << rotation_des << std::endl;
  // std::cout << "rotation current: " << rotation_current << std::endl;

  // calculate Rotation error
  Eigen::Matrix3f rotation_err_skew_ =
      rotation_des.transpose() * rotation_current - rotation_current.transpose() * rotation_des;
  // std::cout << "rotation error skew: " << std::endl << rotation_err_skew_ << std::endl;
  rotation_err_ << 0.5f * rotation_err_skew_(2, 1), 0.5f * rotation_err_skew_(0, 2), 0.5f * rotation_err_skew_(1, 0);
  // calculate omega error
  omega_err_ = x_current.omega - rotation_current * rotation_des * x_des.omega;
  // ROS_INFO("rotation_err_ = [%f, %f, %f]", rotation_err_.x(), rotation_err_.y(), rotation_err_.z());
  // std::cout << "p_dot_err_" << std::endl << p_dot_err_ << std::endl;
  // std::cout << "rotation_err_" << std::endl << rotation_err_ << std::endl;
  // std::cout << "omega_err_" << std::endl << omega_err_ << std::endl;
  // integral sliding mode: suppose zero intial conditions (?)

  // calculate pose dot error
  p_dot_err_ = x_current.p_dot - x_des.p_dot;

  t_current_ = ros::Time::now();    // get current time
  t_delta_ = t_current_ - t_last_;  // calculate time difference for integral action
  t_last_ = t_current_;

  // calculate pose integral error
  if (t_delta_.toSec() > 0.5)
  {
    p_integral_err_ = p_integral_err_ + p_err_ * 0.01;

    rotation_integral_err_ = rotation_integral_err_ + rotation_err_ * 0.01;
  }
  else
  {
    p_integral_err_ = p_integral_err_ + p_err_ * t_delta_.toSec();

    rotation_integral_err_ = rotation_integral_err_ + rotation_err_ * t_delta_.toSec();
  }
  // anti windup
  for (int i = 0; i < 3; i++)
  {
    if (p_integral_err_[i] > pid_I_max_T)
    {
      p_integral_err_[i] = pid_I_max_T;
    }
    else if (p_integral_err_[i] < (-pid_I_max_T))
    {
      p_integral_err_[i] = -pid_I_max_T;
    }

    if (rotation_integral_err_[i] > pid_I_max_R)
    {
      rotation_integral_err_[i] = pid_I_max_R;
    }
    else if (rotation_integral_err_[i] < (-pid_I_max_R))
    {
      rotation_integral_err_[i] = -pid_I_max_R;
    }
  }
  // calculate the virtual input
  v_T_ = x_des.p_ddot - K_T_D_ * p_dot_err_ - K_T_P_ * p_err_ - K_T_I_ * p_integral_err_;  //
  v_R_ = x_des.omega_dot - K_R_D_ * omega_err_ - K_R_P_ * rotation_err_ - K_R_I_ * rotation_integral_err_;
  Eigen::Matrix<float, 6, 1> v_;
  v_.block(0, 0, 3, 1) = v_T_;
  v_.block(3, 0, 3, 1) = v_R_;

  // calculate the force and torque
  control_force_and_torque = J.inverse() * (-h + v_);
  // std::cout << "j inverse:" << std::endl << J.inverse() << std::endl;
  u_T_ = control_force_and_torque.block(0, 0, 3, 1);
  u_R_ = control_force_and_torque.block(3, 0, 3, 1);
  ROS_DEBUG("u_T = [%f, %f, %f]", u_T_.x(), u_T_.y(), u_T_.z());
  ROS_DEBUG("u_R =[%f, %f, %f], ", u_R_.x(), u_R_.y(), u_R_.z());

  /*ROS_INFO("control_force_and_torque = [%f, %f, %f,%f, %f, %f]", control_force_and_torque(0, 0),
           control_force_and_torque(1, 0), control_force_and_torque(2, 0), control_force_and_torque(3, 0),
           control_force_and_torque(4, 0), control_force_and_torque(5, 0));*/
};

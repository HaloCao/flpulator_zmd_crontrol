#include "flypulator_imp_control/impedance_mode_controller.h"

// callback for dynamic reconfigure, sets dynamic parameters (controller gains)
void ImpedanceModeController::configCallback(flypulator_imp_control::imp_parameterConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: \n lambda_R = %f, \n k_R \t  = "
           "%f, \n k_R_I \t  = %f, \n Kp = %f, \n Ki \t  = %f, \n F_ext \t  =%f",
           config.imp_lambda_R, config.imp_k_R, config.imp_k_R_I,
           config.imp_Kp, config.imp_Kd, config.imp_F_ext);
  // set new values to class variables
  lambda_R_ = (float)config.imp_lambda_R;
  K_R_ << config.imp_k_R, 0, 0, 0,  // K_R_ is 4x4
      0, config.imp_k_R, 0, 0, 0, 0, config.imp_k_R, 0, 0, 0, 0, config.imp_k_R;
  K_R_I_ << config.imp_k_R_I, 0, 0, 0, config.imp_k_R_I, 0, 0, 0, config.imp_k_R_I;
  Kp_ << config.imp_Kp, 0, 0, 0, config.imp_Kp, 0, 0, 0, config.imp_Kp;
  Kd_ << config.imp_Kd, 0, 0, 0, config.imp_Kd, 0, 0, 0, config.imp_Kd;
  F_ext_ << config.imp_F_ext, config.imp_F_ext, config.imp_F_ext;
}

// compute control force and torque from desired and current pose
void ImpedanceModeController::computeControlForceTorqueInput(const PoseVelocityAcceleration& x_des,
                                                           const PoseVelocityAcceleration& x_current,
                                                           Eigen::Matrix<float, 6, 1>& control_force_and_torque)
{
  ROS_DEBUG("Impedance Mode Controller calculates control force and torque..");
  ROS_DEBUG("Impedance  Mode Controller, desired: x_des = [%f, %f, %f], q_des = [%f, %f, %f, %f]", x_des.p.x(),
            x_des.p.y(), x_des.p.z(), x_des.q.w(), x_des.q.x(), x_des.q.y(), x_des.q.z());
  ROS_DEBUG("Impedance  Mode Controller, current: x_cur = [%f, %f, %f], q_cur = [%f, %f, %f, %f]", x_current.p.x(),
            x_current.p.y(), x_current.p.z(), x_current.q.w(), x_current.q.x(), x_current.q.y(), x_current.q.z());
  ROS_DEBUG("Integral values are int_T = [%f,%f,%f], int_R = [%f,%f,%f,%f]", integral_T_.x(), integral_T_.y(),
            integral_T_.z(), integral_R_(0), integral_R_(1), integral_R_(2), integral_R_(3));
  ROS_DEBUG("omega = [%f,%f,%f], omega_des = [%f,%f,%f]", x_current.omega.x(), x_current.omega.y(), x_current.omega.z(),
            x_des.omega.x(), x_des.omega.y(), x_des.omega.z());
  // compute force and torque
  // translational control
  // Transitional controller
  z_1_T_ = -x_current.p + x_des.p;
  z_2_T_ = -x_current.p_dot + x_des.p_dot;
  s_T_ = Kp_ * z_1_T_ + Kd_ * z_2_T_ + F_ext_ + mass_ * gravity_;
  control_force_and_torque.block(0, 0, 3, 1) = s_T_;  // convert to force input by multiplying with mass (f=m*a)

  // Rotational controller
  // calculate error quaternion
  eta_ = x_current.q.w();
  eta_d_ = x_des.q.w();
  eps_ = x_current.q.vec();
  eps_d_ = x_des.q.vec();
  ROS_DEBUG("eta = %f, eta_d = %f, eps=[%f,%f,%f], eps_d = [%f,%f,%f]", eta_, eta_d_, eps_.x(), eps_.y(), eps_.z(),
            eps_d_.x(), eps_d_.y(), eps_d_.z());

  eta_err_ = eta_d_ * eta_ + eps_d_.dot(eps_);  // transposed eps_d_ times eps_ is equal to dot product
  eps_err_ = eta_d_ * eps_ - eta_ * eps_d_ -
             eps_d_.cross(eps_);  // skew symmetric matrix times vector is equal to cross product

  // calculate z1
  z_1_R_(0) = 1 - std::abs(eta_err_);  // std::abs is overloaded by math.h such that abs(float) works
  z_1_R_(1) = eps_err_.x();
  z_1_R_(2) = eps_err_.y();
  z_1_R_(3) = eps_err_.z();

  // calculate error omega
  omega_err_ = x_current.omega - x_des.omega;

  // calculate matrix GT (T.. transposed, means 4x3)
  matrix_g_transposed_.row(0) << sgn(eta_err_) * eps_err_.x(), sgn(eta_err_) * eps_err_.y(),
      sgn(eta_err_) * eps_err_.z();
  matrix_g_transposed_.block(1, 0, 3, 3) << eta_err_, -eps_err_.z(), eps_err_.y(), eps_err_.z(), eta_err_,
      -eps_err_.x(), -eps_err_.y(), eps_err_.x(), eta_err_;
  // calculate z2
  z_2_R_ = 0.5f * matrix_g_transposed_ * omega_err_;

  ROS_DEBUG("z_1_R = [%f, %f, %f,%f], z_2_R = {%f, %f, %f, %f]", z_1_R_(0), z_1_R_(1), z_1_R_(2), z_1_R_(3), z_2_R_(0),
            z_2_R_(1), z_2_R_(2), z_2_R_(3));

  // calculate first derivative of error quaternion
  eta_dot_err_ = -0.5f * (eps_err_).dot(omega_err_);
  eps_dot_err_ = 0.5f * matrix_g_transposed_.block(1, 0, 3, 3) * omega_err_;

  ROS_DEBUG("eta_dot_err_ = %f, eps_dot_err_ = [%f,%f,%f]", eta_dot_err_, eps_dot_err_.x(), eps_dot_err_.y(),
            eps_dot_err_.z());

  // calculate matrix G_dot_T (T.. transposed, means 4x3)
  matrix_g_dot_transposed_.row(0) << sgn(eta_err_) * eps_dot_err_.x(), sgn(eta_err_) * eps_dot_err_.y(),
      sgn(eta_err_) * eps_dot_err_.z();
  matrix_g_dot_transposed_.block(1, 0, 3, 3) << eta_dot_err_, -eps_dot_err_.z(), eps_dot_err_.y(), eps_dot_err_.z(),
      eta_dot_err_, -eps_dot_err_.x(), -eps_dot_err_.y(), eps_dot_err_.x(), eta_dot_err_;

  // calculate sliding surface s
  s_R_ = z_2_R_ + lambda_R_ * z_1_R_;
  ROS_DEBUG("s_R_ = [%f,%f,%f,%f]", s_R_(0), s_R_(1), s_R_(2), s_R_(3));

  // calculate output
  u_R_ = -inertia_ * matrix_g_transposed_.transpose() *
             (2 * lambda_R_ * z_2_R_ + matrix_g_dot_transposed_ * omega_err_ +
              2.0f * K_R_ * 2.0f / M_PI * (atan(s_R_.array())).matrix()) +
         inertia_ * x_des.omega_dot + x_current.omega.cross(inertia_ * x_current.omega);

  // integral sliding mode: calculate integral value
  if (control_started_)
  {
    // calculate integral sliding surface
    s_R_I_ = s_R_ + integral_R_;
    integral_R_ = integral_R_ + 2.0f / M_PI * K_R_ * (atan(s_R_.array())).matrix() * t_delta_.toSec();
    // calculate integral output
    u_R_I_ = -K_R_I_ * 2.0f / M_PI *
             (atan((0.5f * (matrix_g_transposed_ * inertia_inv_).transpose() * s_R_I_).array())).matrix();
  }
  else
  {
    control_started_ = true;
  }

  // output is the sum of both rotational outputs (with and without integral action)
  control_force_and_torque.block(3, 0, 3, 1) = u_R_ + u_R_I_;  // already torque dimension
  ROS_DEBUG("..rotational output calculated! ");
  ROS_DEBUG("u_R =[%f, %f, %f], u_R_I = [%f, %f, %f]", u_R_.x(), u_R_.y(), u_R_.z(), u_R_I_.x(), u_R_I_.y(),
            u_R_I_.z());
};

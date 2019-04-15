#include "flypulator_zmd_control/zero_moment_direction_controller.h"

using namespace std;

// callback for dynamic reconfigure, sets dynamic parameters (controller gains)
void ZeroMomentDirectionController::configCallback(flypulator_zmd_control::zmd_parameterConfig& config, uint32_t level)
{
  // set logger level
  //   if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  //     ros::console::notifyLoggerLevelsChanged();

  ROS_INFO("Reconfigure Request: \n K_pp = %f, \n K_pi \t  = %f, \n K_pd \t  = %f, \n K_del \t  = %f, \n K_q "
  	"\t  = %f , \n K_ap \t  = %f,\n K_ai \t  = %f, \n K_ad \t  = %f",
	config.zmd_K_pp, config.zmd_K_pi, config.zmd_K_pd, config.zmd_K_del, config.zmd_K_q, config.zmd_K_ap, config.zmd_K_ai, config.zmd_K_ad);
  // set new values to class variables
  K_pp_ = float(config.zmd_K_pp);
  K_pi_ = float(config.zmd_K_pi);
  K_pd_ = float(config.zmd_K_pd);
  K_del_ = float(config.zmd_K_del);
  K_q_ = float(config.zmd_K_q);
  K_ap_ = float(config.zmd_K_ap);
  K_ai_ = float(config.zmd_K_ai);
  K_ad_ = float(config.zmd_K_ad);
  
}

// compute control force and torque from desired and current pose
void ZeroMomentDirectionController::computeControlForceTorqueInput(const PoseVelocityAcceleration& x_des,
	const PoseVelocityAcceleration& x_current,
	Eigen::Matrix<float, 6, 1>& control_force_and_torque)
{
	ROS_DEBUG("Zero Moment Direction Controller calculates control force and torque..");
	ROS_DEBUG("Zero Moment Direction Controller, desired: x_des = [%f, %f, %f], q_des = [%f, %f, %f, %f]", x_des.p.x(),
		x_des.p.y(), x_des.p.z(), x_des.q.w(), x_des.q.x(), x_des.q.y(), x_des.q.z());
	ROS_DEBUG("Zero Moment Direction Controller, current: x_cur = [%f, %f, %f], q_cur = [%f, %f, %f, %f]", x_current.p.x(),
		x_current.p.y(), x_current.p.z(), x_current.q.w(), x_current.q.x(), x_current.q.y(), x_current.q.z());
	
	ROS_DEBUG("omega = [%f,%f,%f], omega_des = [%f,%f,%f]", x_current.omega.x(), x_current.omega.y(), x_current.omega.z(),
		x_des.omega.x(), x_des.omega.y(), x_des.omega.z());

	// Zero Moment Direction Controller: suppose zero intial conditions (?)
	t_current_ = ros::Time::now();    // get current time
	t_delta_ = t_current_ - t_last_;  // calculate time difference for integral action
	t_last_ = t_current_;
	ROS_DEBUG("t_delta = %f", t_delta_.toSec());

	//cout << "x_des.p = " << x_des.p.transpose() << endl;
	//cout << "x_des.q = " << x_des.q.coeffs().transpose() << endl;

	//calculate position and velocity error
	//e_p_ = x_current.p - purpose_position_;
	//e_v_ = x_current.p_dot - purpose_velocity_;
	e_p_ = x_current.p - x_des.p;
	e_p_sum_ = e_p_sum_ + e_p_ * t_delta_.toSec();
	e_v_ = x_current.p_dot - x_des.p_dot;
	//cout << "position_error = " << e_p_.transpose() << endl;

	//calculate fr
	f_r_ = mass_ * gravity_ - K_pi_ * e_p_sum_ - K_pp_ * e_p_ - K_pd_ * e_v_;

	//calculate zeromoment direction
	_d_ = (x_des.q.toRotationMatrix()).transpose() * e3_;
	d_norm_ = sqrt(_d_.x() * _d_.x() + _d_.y() * _d_.y()+_d_.z() * _d_.z());
	d_ = _d_ / d_norm_;

	//calculate force mismatch
	f_del_ = q_d_.toRotationMatrix() * d_ * f_ - f_r_;

	u_z_ = K_pd_ * K_pp_ / mass_ * e_p_ + (K_pd_ * K_pd_ / mass_ - K_pp_) * e_v_ - (K_pd_ / mass_ + K_del_) * f_del_;

	skew_d_ << 0, -d_(2), d_(1),
		d_(2), 0, -d_(0),
		-d_(1), d_(0), 0;

	eta_ = x_current.q.w();
	eta_r_ = x_des.q.w();
	eta_d_ = q_d_.w();
	eps_ = x_current.q.vec();
	eps_r_ = x_des.q.vec();
	eps_d_ = q_d_.vec();
	eps_err_ = eta_d_ * eps_ - eta_ * eps_d_ - eps_d_.cross(eps_);  // skew symmetric matrix times vector is equal to cross product
	eps_rd_err_ = eta_r_ * eps_d_ - eta_d_ * eps_r_ - eps_r_.cross(eps_d_);  // skew symmetric matrix times vector is equal to cross product

	//attitude controller
	w_rd_ = -K_q_ * d_ * d_.transpose() * eps_rd_err_;

	//w_d change the attitude
	w_d_ = 1 / f_ * skew_d_ * (q_d_.toRotationMatrix()).transpose() * u_z_ + w_rd_;

	//to calculate quaternion integral
	eta_dot_ = -0.5f * (eps_d_.transpose()) * w_d_;
	skew_eps_d_ << eta_d_, -eps_d_.z(), eps_d_.y(), eps_d_.z(), eta_d_, -eps_d_.x(), -eps_d_.y(), eps_d_.x(), eta_d_;
	eps_dot_ = 0.5f * skew_eps_d_ * w_d_;

	// force derivative
	f_dot_ = (q_d_.toRotationMatrix() * d_).transpose() * u_z_;

	//force integral
	f_ = f_ + f_dot_ * t_delta_.toSec();
	
	//quaternion integral 
	q_d_.w() = q_d_.w() + eta_dot_ * t_delta_.toSec();
	q_d_.vec()= q_d_.vec() + eps_dot_ * t_delta_.toSec();

	
	// part I glied intergrator
	eps_err_sum_ = eps_err_sum_ + eps_err_ * t_delta_.toSec();

	//calculate torque
	tau_r_ = -K_ai_ * eps_err_sum_ - K_ap_ * eps_err_ - K_ad_ * (x_current.omega - w_d_) + (x_current.omega).cross(inertia_ * x_current.omega);
	
	//calculated force is in body frame
	control_force_and_torque.block(0, 0, 3, 1) = x_current.q.toRotationMatrix() * d_ * f_;
	control_force_and_torque.block(3, 0, 3, 1) = tau_r_;

	//cout << "torque = " << tau_r_.transpose() << endl;
	//cout << "force = " << f_del_.transpose() << endl;



}
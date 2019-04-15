#include "flypulator_zmd_control/controller_interface.h"

// constructor
ControllerInterface::ControllerInterface()
{
  // default setting
  use_bidirectional_propeller_ = false;
  use_motor_ff_control_ = false;
  controller_type_ = "zmd";

  // read parameters from ros parameter server
  readParameterFromServer();

  // convert max rotor velocity from rpm to rad/s, 2*PI/60
  vel_max_ = float(drone_parameter_["rotor_vel_max"] * M_PI / 30);
  ROS_DEBUG_STREAM("Maximum rotor velocity: " << vel_max_ << " rad/s");

  // read state estimation update rate, also do if boolean false to allow future dynamic reconfiguring of boolean
  float updat_rate;
  if (ros::param::get("controller/update_rate", updat_rate))
  {
    ROS_INFO("State estimation update rate: %f Hz", double(updat_rate));
  }
  else
  {
    updat_rate = 200;  // 200 Hz
    ROS_WARN("Load control update rate failed, set to 50ms (200Hz)");
  }

  // calculate k_ff and z_p_ff, k_ff = Ts / (Ts + T_motor), z_p = T_motor / (Ts + T_motor)
  k_ff_ = 1 / (1 + updat_rate * float(drone_parameter_["t_motor"]));
  z_p_ff_ = float(drone_parameter_["t_motor"]) / (1 / updat_rate + float(drone_parameter_["t_motor"]));
  ROS_DEBUG_STREAM("k_ff = " << k_ff_ << ", "
                             << " z_p_ff = " << z_p_ff_);

  // initialized last rotor velocity with zero, right?
  spinning_rates_last_ = Eigen::Matrix<float, 6, 1>::Zero();

  // provide mass, inertia and gravity for controller
  float mass = float(drone_parameter_["mass"]);
  Eigen::Matrix3f inertia;
  inertia << float(drone_parameter_["i_xx"]), 0, 0, 0, float(drone_parameter_["i_yy"]), 0, 0, 0,
      float(drone_parameter_["i_zz"]);

  float gravity = float(drone_parameter_["gravity"]);

  // precompute mapping matrix M
  computeMappingMatrix();
  convert_force_part_to_b_ = Eigen::Matrix<float, 6, 6>::Identity();

  // create controller object depending on desired controller type (in controller_type_, read from parameter in
  // readDroneParameterFromServer())
  if (controller_type_.compare("zmd") == 0)  // controller type PID, create object of PidController class
  {
    controller_ = new ZeroMomentDirectionController(mass, inertia, gravity);  // use new, otherwise object is destroyed after
                                                              // this function and pointer is a dead pointer
                                                              // see also
    // https://stackoverflow.com/questions/6337294/creating-an-object-with-or-without-new?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
  }  // add other controller types here with if comparisons
  else
  {
    // Should not come here
    ROS_ERROR("Unknown controller type in ControllerInterface class constructor!");
  }
};

// compute the control output from desired and current pose and save to spinning_rates[6]
void ControllerInterface::computeControlOutput(const PoseVelocityAcceleration& x_des,
                                               const PoseVelocityAcceleration& x_current,
                                               Eigen::Matrix<float, 6, 1>& spinning_rates)
{
  // call controller to compute Force and Torque output
  controller_->computeControlForceTorqueInput(x_des, x_current, control_force_and_torque_);
  // map control force and torque to spinning velocities of the propellers resp. rotors
  mapControlForceTorqueInputToPropellerRates(x_current);
  // perform feedforward control
  if (use_motor_ff_control_)
  {
    motorFeedForwardControl();
  }
  // limit spinning rate
  for (int i = 0; i < 6; i++)
  {
    if (spinning_rates_current_(i, 0) > vel_max_)
    {
      ROS_WARN("Propeller[%d]: positive max spinrate reached with %f rad/s", i + 1, spinning_rates_current_(i, 0));
      spinning_rates_current_(i, 0) = vel_max_;
    }
    if (spinning_rates_current_(i, 0) < -vel_max_)
    {
      ROS_WARN("Propeller[%d]: negative max spinrate reached with %f rad/s", i + 1, spinning_rates_current_(i, 0));
      spinning_rates_current_(i, 0) = -vel_max_;
    }

    // quickfix to account for unidirectional propellers
    if ((!use_bidirectional_propeller_) && spinning_rates_current_(i, 0) < 0)
    {
      ROS_WARN("Unidirectional propeller[%d]: negative spinrate %f rad/s, set to 0", i + 1,
               spinning_rates_current_(i, 0));
      spinning_rates_current_(i, 0) = 0;
    }
  }
  spinning_rates = spinning_rates_current_;        // set output
  spinning_rates_last_ = spinning_rates_current_;  // save last value
}

void ControllerInterface::readParameterFromServer()
{
  // try to load parameter from ros parameter server, parameters on server should already be printed by launch
  if (ros::param::get("/uav", drone_parameter_))
  {
    ROS_INFO("Load drone parameter successfully from parameter server");
  }
  else
  {  // use default parameter values
    ROS_WARN("no drone parameter available, use default values...");
    drone_parameter_["mass"] = 5;
    drone_parameter_["i_xx"] = 0.4;
    drone_parameter_["i_yy"] = 0.4;
    drone_parameter_["i_zz"] = 0.8;
    drone_parameter_["alpha"] = 30;  //[degree]
    drone_parameter_["beta"] = 0;    //[degree]
    drone_parameter_["delta_h"] = 0;
    drone_parameter_["length"] = 0.5;
    drone_parameter_["gravity"] = 9.81;
    drone_parameter_["k"] = 0.000056;
    drone_parameter_["b"] = 0.0000011;
    drone_parameter_["t_motor"] = 0.05;
    drone_parameter_["rotor_vel_min"] = 0;
    drone_parameter_["rotor_vel_max"] = 5700;  // [rpm]
    // print drone parameter if not available on parameter server
    for (auto elem : drone_parameter_)
    {
      ROS_INFO_STREAM("/uav/" << elem.first << ": " << elem.second);
    }
  }

  // read bidirectional property from ros parameter server
  if (ros::param::get("/urdf/bidirectional", use_bidirectional_propeller_))
  {
    // ROS_INFO("Propellers: %s", use_bidirectional_propeller_ ? "bidirectional" : "unidirectional");
    ROS_INFO("Load propeller directional property successfully from parameter server");
  }
  else
  {
    ROS_WARN("Load propeller property failed, default: unidirectional");
  }

  // read motor feedforward bool variable (use feedforward/dont use it)

  if (ros::param::get("/controller/use_motor_ff", use_motor_ff_control_))
  {
    // ROS_INFO("Motor feedforward: %s", use_bidirectional_propeller_ ? "enabled" : "disabled");
    ROS_INFO("Load motor feedforward setting successfully from paramter server");
  }
  else
  {
    ROS_WARN("Load rotor feedforward setting failed, default: disabled");
  }

  // get controller type and ensure valid type ("ism" or "pid")
  if (ros::param::get("/controller/type", controller_type_))
  {
    ROS_INFO("Controller type = %s", controller_type_.c_str());
    if (!(controller_type_.compare("zmd") == 0))  // add new controller types here with ||
                                                  // !controller_type_.equals(<newControllerTypeAsString>)
    {
      ROS_WARN("Unknown controller type loaded from parameter server, reset to zmd");  // add "or
                                                                                       // <newControllerType>" for
                                                                                       // new controller type
      controller_type_ = "zmd";
    }
  }
  else
  {
    ROS_WARN("Load controller type from parameter server failed, default: zmd");
  }
};

// map control force and torques to propeller spinning rates
void ControllerInterface::mapControlForceTorqueInputToPropellerRates(const PoseVelocityAcceleration& x_current)
{
  ROS_DEBUG("map control forces and torques to propeller rates...");
  // [force, torqe] = ^B M * omega_spin
  // convert forces to body frame
  convert_force_part_to_b_.block(0, 0, 3, 3) = x_current.q.toRotationMatrix();
  // calculate inverse mapping matrix
  map_matrix_inverse_b_ = (convert_force_part_to_b_ * map_matrix_).inverse();
  // calculate square of spinning rates
  spinning_rates_current_ = map_matrix_inverse_b_ * control_force_and_torque_;
  // calculate spinning rates with correct sign
  for (int i = 0; i < 6; i++)
  {
    if (spinning_rates_current_(i, 0) >= 0)
    {
      spinning_rates_current_(i, 0) = sqrt(spinning_rates_current_(i, 0));
    }
    else
    {
      spinning_rates_current_(i, 0) = -sqrt(-spinning_rates_current_(i, 0));
    }
  }
}

// perform feedforward control if boolean class variable use_motor_ff_control_ is true
void ControllerInterface::motorFeedForwardControl()
{
  // U(z) / Y(z) = k_ff * z / (z - z_p_ff); Y.. output, u.. input; -> y[k] = - z_p_ff/k_ff_ * u[k-1] + 1/k_ff * u[k]
  for (int i = 0; i < 6; i++)
  {
    spinning_rates_current_(i, 0) =
        1 / k_ff_ * spinning_rates_current_(i, 0) - z_p_ff_ / k_ff_ * spinning_rates_last_(i, 0);
  }
}

// computes mapping matrix of spinning rates to forces/torques
void ControllerInterface::computeMappingMatrix()
{
  // compute thrust directions
  float alpha;
  float beta;
  float gamma;
  Eigen::Vector3f e_r;   // thrust direction
  Eigen::Vector3f mom;   // drag + thrust torque
  Eigen::Vector3f r_ti;  // vector from COM to i-th rotor
  float k = float(drone_parameter_["k"]);
  float b = float(drone_parameter_["b"]);
  float l = float(drone_parameter_["length"]);
  float dh = float(drone_parameter_["delta_h"]);
  // compute matrix
  for (int i = 0; i < 6; i++)
  {
    alpha = float(drone_parameter_["alpha"] * M_PI / 180.0 * pow(-1, i));
    beta = float(drone_parameter_["beta"] * M_PI / 180.0);
    gamma = float(i * M_PI / 3.0) - float(M_PI / 6.0);
    // compute thrust direction
    e_r = Eigen::Vector3f(cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma),
                          cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma), cos(alpha) * cos(beta));
    // compute r_ti vector;
    r_ti << l * cos(gamma), l * sin(gamma), dh;
    // compute thrust and drag torque
    mom = k * r_ti.cross(e_r) + b * pow(-1, i) * e_r;  // k * r_ti.cross(e_r) + b * pow(-1,i) * e_r;
    // save to class variable map_matrix
    map_matrix_.block(0, i, 3, 1) = k * e_r;  // k*e_r
    map_matrix_.block(3, i, 3, 1) = mom;
  }
}

Eigen::Matrix<float, 6, 1> ControllerInterface::getControlWrench()
{
  return control_force_and_torque_;
}

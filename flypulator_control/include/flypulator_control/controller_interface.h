#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

// include controller classes
#include "base_controller.h"
//#include "flypulator_control/sliding_mode_controller.h"
#include "flypulator_control/pid_controller.h"

// dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>

// include message structs
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "flypulator_common_msgs/RotorVelStamped.h"

class ControllerInterface
{
public:
  ControllerInterface();  // constructor implemented in .cpp file

  /**
   * \brief computeControlForceTorqueInput Calculates the desired rotor command
   * according to current UAV states
   * \param x_des contains the desired pose vel and acc
   * \param x_current contains the feedback
   * \param spinning_rates desired rotor command
   */
  void computeControlOutput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current,
                            Eigen::Matrix<float, 6, 1>& spinning_rates);

  /**
   * \brief getControllerReference Return a reference to controller object for dynamic reconfigure
   * \return reference to controllerobject
   */
  BaseController* getControllerReference() const;

  /**
   * \brief getControllerReference Return Return the control force and torque
   * \return control wrench
   */
  Eigen::Matrix<float, 6, 1> getControlWrench() const;

private:
  /**
   * \brief readParameterFromServer Read uav parameter from ros parameter server
   */
  void readParameterFromServer();

  /**
   * \brief mapControlForceTorqueInputToPropellerRates Map control wrench to propeller spinning rates
   * \param x_current Current states requested by transform of force to body frame
   */
  void mapControlForceTorqueInputToPropellerRates(const PoseVelocityAcceleration& x_current);

  /**
   * \brief computeMappingMatrix compute mapping matrix from spinning rates to forces/torques
   */
  void computeMappingMatrix();

  /**
   * \brief motorFeedForwardControl perform motor feedforward compensation
   */
  void motorFeedForwardControl();

  // map of drone parameters
  std::map<std::string, double> drone_parameter_;
  // controller type
  std::string controller_type_;
  // 6D- Vector of control force and torque (output of controller class)
  Eigen::Matrix<float, 6, 1> control_force_and_torque_;
  // pointer to controller;
  BaseController* controller_;
  // mapping matrix
  Eigen::Matrix<float, 6, 6> map_matrix_;
  // matrix containing R_BtoI matrix to convert body forces to inertal frame
  Eigen::Matrix<float, 6, 6> convert_force_part_to_b_;
  // inverse of mapping matrix
  Eigen::Matrix<float, 6, 6> map_matrix_inverse_b_;
  // spinning rates of current calculation,
  Eigen::Matrix<float, 6, 1> spinning_rates_current_;
  // spinning rates of last calculation
  Eigen::Matrix<float, 6, 1> spinning_rates_last_;
  // feedforward control parameters, set in constructor
  float k_ff_;
  float z_p_ff_;
  // use feedforward control parameter
  bool use_motor_ff_control_;
  bool use_bidirectional_propeller_;
  float vel_max_;
  const float ramp_increment_ = 0.002f;
  float ramp_coeff_;
};

#endif  // CONTROLLER_INTERFACE_H

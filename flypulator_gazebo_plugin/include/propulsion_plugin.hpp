#ifndef PROPULSION_PLUGIN_HPP
#define PROPULSION_PLUGIN_HPP

#include <vector>
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>
#include <algorithm>
#include <Eigen/Dense>

// include gazebo classes
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>

// include ros classes
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

// include message structs
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include "flypulator_common_msgs/Vector6dMsg.h"
#include "flypulator_common_msgs/RotorVelStamped.h"

#include <tf/transform_broadcaster.h>

// include BLDC motor model
#include "motor_model.hpp"

namespace gazebo
{
/// \brief A gazebo plugin to simulate propulsion system of the multirotor
class PropulsionPlugin : public ModelPlugin
{
  /// \brief Constructor
public:
  PropulsionPlugin();

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief The onUpdate function is called by Gazebo for every simulation
  /// iteration. Update rate = 1 KHz
  /// \param[in] _info Update infomation.
  void onUpdate(const common::UpdateInfo &_info);

  /// \brief The onRosWindMsg function is called every update of the
  /// topic for wind.
  /// \param[in] _wind_msg wind command message.
  void onRosWindMsg(const geometry_msgs::Vector3ConstPtr &_wind_msg);

  /// \brief The onControlMsg function is called every update of the
  /// topic for rotor command.
  /// \param[in] _info Update infomation.
  void onControlMsg(const flypulator_common_msgs::RotorVelStampedConstPtr &_msg);

  /// \brief sign function
  /// \param[in]
  template <typename T>
  int Sgn(T &_num);

  /// \brief Clamp a value in his limitation.
  /// \param[in] x input value
  /// \param[in]
  double clamp(double _x, double _low, double _high);

private:
  /// \brief The setVelotity function is called to set rotor velocity
  /// using the method: Joint Motors
  /// (http://gazebosim.org/tutorials?tut=set_velocity)
  void setRotorVelocity();

  /// \brief The calcGroundEffectCoeff function is called to calculate
  /// the coeffecient for ground effect.
  void calcGroundEffectCoeff();

  void queueThread();

  /// \brief The setVelotity function is called to load parameters from
  /// ros parameter server.
  void readParamsFromServer();

  /// \brief The tfPublisher function is called to publish tf message.
  void tfPublisher();

  /// \brief The jointStatePublisher function is called to publish
  /// joint states.
  void jointStatePubliher();

  /// \brief The wrenchPublisher function is called to publish force and
  /// torque on rotors
  void wrenchPublisher();

  /// \brief The streamDataToFile function is called save data to file
  /// for debugging.
  void streamDataToFile();

  // simulation configuration parameters
  bool write_data_2_file_;  // new version with more data (contains also hub force and roll moment)
  bool WRITE_CSV_FILE_;     // if save the test_data to .csv
  std::string RESULT_CSV_PATH_;
  std::string file_path_;
  bool add_wrench_to_drone_;     // if add force and torque to drone in gazebo
  bool use_ground_effect_;       // if enable ground effect
  bool use_motor_dynamic_;       // if enable motor dynamic
  bool use_simple_aerodynamic_;  // use f=k*omega² and tau = b * omega²
  bool add_dist_;                // add disturbances
  double test_data_[12];         // test data for debug

  // uav and aerodynamic parameters
  bool bidirectional_;                    // bidirectional option
  std::vector<std::string> joint_names_;  // joint names of propeller
  std::vector<std::string> link_names_;   // link names of propeller
  // if the rotor vel smaller than 325, we will get negativ CT and moment,
  // caused by the aero dynamic eq.(Hiller's 4.57)
  double vel_min_;                            // min rotor speed [rad/s]
  double vel_max_;                            // max rotor speed [rad/s]
  std::map<std::string, double> aero_param_;  // map of aerodynamic parameters
  double k_simple_aero_;                      //[N/(rad/s)^2]
  double b_simple_aero_;                      //[Nm/(rad/s)^2]

  double uav_mass_;             // drone mass
  double tilting_angle_;        // [degree] rotor tilting angle along arm axis
  double ground_effect_coeff_;  // ground effect coefficient
  double blade_pitch_angle_;    // blade pitch angle
  double wing_area_;            // wing area
  double rv;                    // rotor_axis_vertical_axis_angle cos(rv)=cos(pitch)*cos(yaw)
  double solidity_;             // rotor solidity
  double wind_vx_;              // wind velocity in global x
  double wind_vy_;              // wind velocity in global y
  double wind_vz_;              // wind velocity in global z
  double uav_vx_;               // drone velocity in global x
  double uav_vy_;               // drone velocity in global y
  double uav_vz_;               // drone velocity in global z
  double air_global_vx_;        // air velocity in global x
  double air_global_vy_;        // air velocity in global y
  double air_global_vz_;        // air velocity in global z
  double ind_vel_hover_;        // induced velocity in the hovering case

  // TODO: change all array to vector and resize with joint_names_.size()
  double rotor_vel_raw_[6] = {0};  // rotor velocity with sign, used for motor dynamic
  double rotor_vel_cmd_[6] = {0};  // blade spinning velocity commands
  double rotor_vel_[6] = {0};      // blade spinning velocity
  int dir_spin_default_[6] = {1,  -1, 1,
                              -1, 1,  -1};      // default blade rotating direction 1 counterclockwise; -1 clockwise
  int dir_thrust_[6] = {1, 1, 1, 1, 1, 1};      // thrust force direction
  int dir_vel_actual_[6] = {1, 1, 1, 1, 1, 1};  // real rotate direction

  // Eigen::Matrix3d transform_w2b_;  // transformation matrix from global coordinate to body coordinate

  Eigen::Vector3d aero_force_[6];   // aerodynamic forces
  Eigen::Vector3d aero_torque_[6];  // aerodynamic torques

  ignition::math::Vector3<double> distforce_;  // disturbance force

  // dynamic model of rotors
  flypulator::MotorModel motor1_;
  flypulator::MotorModel motor2_;
  flypulator::MotorModel motor3_;
  flypulator::MotorModel motor4_;
  flypulator::MotorModel motor5_;
  flypulator::MotorModel motor6_;

  /// \brief Pointer to the model.
  physics::ModelPtr model_;

  physics::LinkPtr ctrl_link_ptr_[6];  // pointer to control links
  physics::LinkPtr link0_;             // point to base link

  physics::JointPtr ctrl_joint_ptr_[6];  // pointer to control joints

  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode_;

  event::ConnectionPtr updateConnection_;

  ros::Subscriber rosSubWind_;

  ros::Subscriber rosSubControl_;

  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue_;

  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread_;

  ros::Publisher pub_thrust_torque_ratio_;
  ros::Publisher pub_joint_state_;
  ros::Publisher pub_link_wrench_[6];

  std::ofstream result_file;
};
}  // namespace gazebo

#endif  // PROPULSION_PLUGIN_HPP

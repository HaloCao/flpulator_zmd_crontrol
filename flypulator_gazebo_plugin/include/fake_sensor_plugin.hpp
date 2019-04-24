#ifndef FAKE_SENSOR_PLUGIN_HH
#define FAKE_SENSOR_PLUGIN_HH

#include <vector>
#include <queue>
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <gazebo_msgs/LinkStates.h>

#include <Eigen/Dense>

#include <flypulator_common_msgs/UavStateStamped.h>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/seed_seq.hpp>

#define PI (M_PI)

namespace gazebo
{
class FakeSensorPlugin : public ModelPlugin
{
public:
  FakeSensorPlugin();

  // virtual ~FakeSensorPlugin(){}; // not virtual ~FakeSensorPlugin()

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief The onUpdate function is called by Gazebo for every simulation
  /// iteration. It publishes uav state. Update rate = 1 KHz
  /// \param[in] _info Update infomation.
public:
  void OnUpdate(const common::UpdateInfo &_info);

private:
  void QueueThread();

private:
  void streamDataToFile();

  /// \brief Pointer to the model.
private:
  physics::ModelPtr model_;
  physics::WorldPtr world_;

private:
  physics::LinkPtr link0_;

  /// \brief A node use for ROS transport
private:
  std::unique_ptr<ros::NodeHandle> rosNode_;

private:
  ros::Subscriber rosSubLink_;

  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue_;

  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread_;

private:
  event::ConnectionPtr updateConnection_;

private:
  ros::Publisher pub_real_state_;
  ros::Publisher pub_meas_state_;

private:
  std::ofstream result_file_;

private:
  std::queue<flypulator_common_msgs::UavStateStamped> queue_;
  // output rate = 1000Hz/ouput_rate_divider // used if not specified in state_estimation_param_yaml
  int output_rate_divider_;
  int nr_of_msg_delay_;

  double three_sigma_p_;
  double three_sigma_v_;
  double three_sigma_phi_;
  double three_sigma_omega_;
  double sampling_time_;
};
}  // namespace gazebo

#endif  // FAKE_SENSOR_PLUGIN_HH

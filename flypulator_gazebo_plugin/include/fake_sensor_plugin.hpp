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
bool write_data_2_file = true;  // save pose data to file
std::string file_path = "/home/chao/flypulator_ws/src/flypulator/flypulator_gazebo_plugin/position_data_refstep.csv";

int g_output_rate_divider =
    10;  // output rate = 1000Hz/ouput_rate_divider // used if not specified in state_estimation_param_yaml
unsigned size_of_queue =
    2;  // number of messages which meas_state messages are delayed by if not specified in state_estimation_param_yaml

class FakeSensorPlugin : public ModelPlugin
{
public:
  FakeSensorPlugin();

public:
  virtual ~FakeSensorPlugin();

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
  physics::ModelPtr model;
  physics::WorldPtr world;

private:
  physics::LinkPtr link0;

  /// \brief A node use for ROS transport
private:
  std::unique_ptr<ros::NodeHandle> rosNode;

private:
  ros::Subscriber rosSubLink;

  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread;

private:
  event::ConnectionPtr updateConnection;

private:
  ros::Publisher pub_real_state;
  ros::Publisher pub_meas_state;

private:
  std::ofstream result_file;

private:
  std::queue<flypulator_common_msgs::UavStateStamped> queue;

  boost::variate_generator<boost::mt19937, boost::normal_distribution<>> noise_generator_x_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>> noise_generator_y_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>> noise_generator_z_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>> noise_generator_v_x_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>> noise_generator_v_y_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>> noise_generator_v_z_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>> noise_generator_roll_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>> noise_generator_pitch_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>> noise_generator_yaw_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>> noise_generator_om_x_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>> noise_generator_om_y_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>> noise_generator_om_z_;
};
}  // namespace gazebo

#endif  // FAKE_SENSOR_PLUGIN_HH

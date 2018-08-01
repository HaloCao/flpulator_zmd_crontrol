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
  /// \brief constructor
public:
  FakeSensorPlugin()
  {
  }
  /// \brief    Destructor.
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

  bool write_data_2_file = false;  // save pose data to file
  std::string file_path = "/home/jan/flypulator_ws/src/flypulator/flypulator_gazebo_plugin/"
                          "position_data_refstep.csv";

  int g_output_rate_divider =
      10;  // output rate = 1000Hz/ouput_rate_divider // used if not specified in state_estimation_param_yaml
  unsigned size_of_queue =
      2;  // number of messages which meas_state messages are delayed by if not specified in state_estimation_param_yaml

  // initial values for noise generators if no noise specified in state_estimation_param.yaml
  double g_sigma_p = 0;      // 1 / 3.0f;
  double g_sigma_v = 0;      // sqrt(2)*g_sigma_p/(g_output_rate_divider/1000) / 3.0f;
  double g_sigma_phi = 0;    // M_PI / 180.0f * 1 / 3.0f;
  double g_sigma_omega = 0;  // sqrt(2)*g_sigma_phi/(g_output_rate_divider/1000) / 3.0f;
  // https://stackoverflow.com/questions/25193991/how-to-initialize-boostmt19937-with-multiple-values-without-using-c11
  boost::random::seed_seq seed_x({1ul, 2ul, 3ul, 4ul});
  boost::random::seed_seq seed_y({5ul, 6ul, 7ul, 8ul});
  boost::random::seed_seq seed_z({9ul, 10ul, 11ul, 12ul});
  boost::random::seed_seq seed_v_x({13ul, 14ul, 15ul, 16ul});
  boost::random::seed_seq seed_v_y({17ul, 18ul, 19ul, 20ul});
  boost::random::seed_seq seed_v_z({21ul, 22ul, 23ul, 24ul});
  boost::random::seed_seq seed_roll({25ul, 26ul, 27ul, 28ul});
  boost::random::seed_seq seed_pitch({29ul, 30ul, 31ul, 32ul});
  boost::random::seed_seq seed_yaw({33ul, 34ul, 35ul, 36ul});
  boost::random::seed_seq seed_om_x({37ul, 38ul, 39ul, 40ul});
  boost::random::seed_seq seed_om_y({41ul, 42ul, 43ul, 44ul});
  boost::random::seed_seq seed_om_z({45ul, 46ul, 47ul, 48ul});

  boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
      g_noise_generator_x(boost::mt19937(seed_x), boost::normal_distribution<>(0, g_sigma_p));
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
      g_noise_generator_y(boost::mt19937(seed_y), boost::normal_distribution<>(0, g_sigma_p));
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
      g_noise_generator_z(boost::mt19937(seed_z), boost::normal_distribution<>(0, g_sigma_p));
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
      g_noise_generator_v_x(boost::mt19937(seed_v_x), boost::normal_distribution<>(0, g_sigma_v));
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
      g_noise_generator_v_y(boost::mt19937(seed_v_y), boost::normal_distribution<>(0, g_sigma_v));
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
      g_noise_generator_v_z(boost::mt19937(seed_v_z), boost::normal_distribution<>(0, g_sigma_v));
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
      g_noise_generator_roll(boost::mt19937(seed_roll), boost::normal_distribution<>(0, g_sigma_phi));
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
      g_noise_generator_pitch(boost::mt19937(seed_pitch), boost::normal_distribution<>(0, g_sigma_phi));
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
      g_noise_generator_yaw(boost::mt19937(seed_yaw), boost::normal_distribution<>(0, g_sigma_phi));
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
      g_noise_generator_om_x(boost::mt19937(seed_om_x), boost::normal_distribution<>(0, g_sigma_omega));
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
      g_noise_generator_om_y(boost::mt19937(seed_om_y), boost::normal_distribution<>(0, g_sigma_omega));
  boost::variate_generator<boost::mt19937, boost::normal_distribution<>>
      g_noise_generator_om_z(boost::mt19937(seed_om_z), boost::normal_distribution<>(0, g_sigma_omega));
};
}  // namespace gazebo

#endif  // FAKE_SENSOR_PLUGIN_HH

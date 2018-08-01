#ifndef _FAKE_SENSOR_PLUGIN_HH_
#define _FAKE_SENSOR_PLUGIN_HH_

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
std::string file_path = "/home/jan/flypulator_ws/src/flypulator/flypulator_gazebo_plugin/position_data_refstep.csv";

int g_output_rate_divider =
    10;  // output rate = 1000Hz/ouput_rate_divider // used if not specified in state_estimation_param_yaml
unsigned size_of_queue =
    2;  // number of messages which meas_state messages are delayed by if not specified in state_estimation_param_yaml

// initial values for noise generators if no noise specified in state_estimation_param.yaml
float g_sigma_p = 0;      // 1 / 3.0f;
float g_sigma_v = 0;      // sqrt(2)*g_sigma_p/(g_output_rate_divider/1000) / 3.0f;
float g_sigma_phi = 0;    // M_PI / 180.0f * 1 / 3.0f;
float g_sigma_omega = 0;  // sqrt(2)*g_sigma_phi/(g_output_rate_divider/1000) / 3.0f;
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

class FakeSensorPlugin : public ModelPlugin
{
public:
  FakeSensorPlugin()
  {
  }

public:
  ~FakeSensorPlugin()
  {
  }

public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    ROS_INFO_STREAM("Loading FakeSensorPlugin ...");
    if (_model->GetJointCount() == 0)
    {
      ROS_ERROR("Invalid joint count, plugin not loaded");
      return;
    }

    // Store the model pointer for convenience
    this->model = _model;
    this->world = _model->GetWorld();

    this->link0 = _model->GetChildLink("base_link");

    float three_sigma_p = 0;
    float three_sigma_v = 0;
    float three_sigma_phi = 0;
    float three_sigma_omega = 0;
    // take noise values from parameter server
    // and change distributions if value read from file
    // https://stackoverflow.com/questions/36289180/boostrandomvariate-generator-change-parameters-after-construction
    if (ros::param::get("state_estimation/three_sigma_p", three_sigma_p))
    {
      ROS_DEBUG("Three_sigma_p load successfully from parameter server");
      boost::normal_distribution<> new_dist(0, three_sigma_p / 3.0f);
      g_noise_generator_x.distribution() = new_dist;
      g_noise_generator_y.distribution() = new_dist;
      g_noise_generator_z.distribution() = new_dist;
    }
    if (ros::param::get("state_estimation/three_sigma_v", three_sigma_v))
    {
      ROS_DEBUG("Three_sigma_v load successfully from parameter server");
      boost::normal_distribution<> new_dist(0, three_sigma_v / 3.0f);
      g_noise_generator_v_x.distribution() = new_dist;
      g_noise_generator_v_y.distribution() = new_dist;
      g_noise_generator_v_z.distribution() = new_dist;
    }
    if (ros::param::get("state_estimation/three_sigma_phi", three_sigma_phi))
    {
      ROS_DEBUG("Three_sigma_phi load successfully from parameter server");
      boost::normal_distribution<> new_dist(0, three_sigma_phi * M_PI / 180.0f / 3.0f);
      g_noise_generator_roll.distribution() = new_dist;
      g_noise_generator_pitch.distribution() = new_dist;
      g_noise_generator_yaw.distribution() = new_dist;
    }
    if (ros::param::get("state_estimation/three_sigma_omega", three_sigma_omega))
    {
      ROS_DEBUG("Three_sigma_omega load successfully from parameter server");
      boost::normal_distribution<> new_dist(0, three_sigma_omega * M_PI / 180.0f / 3.0f);
      g_noise_generator_om_x.distribution() = new_dist;
      g_noise_generator_om_y.distribution() = new_dist;
      g_noise_generator_om_z.distribution() = new_dist;
    }

    float sampling_time;
    if (ros::param::get("state_estimation/sampling_time", sampling_time))
    {
      ROS_INFO("sampling time load successfully from parameter server");
      g_output_rate_divider = (int)(sampling_time * 1000.0);  // convert sampling time to divider
    }
    int nr_of_msg_delay;
    if (ros::param::get("state_estimation/nr_of_msg_delay", nr_of_msg_delay))
    {
      ROS_INFO("nr of message delay load successfully from parameter server");
      size_of_queue = nr_of_msg_delay;
    }

    if (write_data_2_file)
    {
      result_file.open(file_path);
      result_file << "time,x,y,z,roll,pitch,yaw" << std::endl;
    }

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      ROS_WARN_STREAM("Create gazebo_client node");
    }
    // Create our ROS node.
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
    ROS_INFO_STREAM("fake_sensor_plugin get node:" << this->rosNode->getNamespace());

    // real states of the drone
    this->pub_real_state = this->rosNode->advertise<flypulator_common_msgs::UavStateStamped>("/drone/real_state", 100);
    // measured states of the drone
    this->pub_meas_state = this->rosNode->advertise<flypulator_common_msgs::UavStateStamped>("/drone/meas_state", 100);

    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&FakeSensorPlugin::QueueThread, this));

    // This event is broadcast every simulation iteration.
    // Get pose of the drone and publish.
    // TODO: the callback should connect to WorldUpdateEnd signal not Begin
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&FakeSensorPlugin::OnUpdate, this, _1));

    ROS_INFO_STREAM("FakeSensorPlugin Loaded !");
  }

  // publish drone state
public:
  void OnUpdate(const common::UpdateInfo &_info)  // update rate = 1kHz
  {
    static int loop_cnt = 0;

    if (loop_cnt >= (g_output_rate_divider - 1))
    {
      loop_cnt = 0;  // reset loop counter
      // ROS_INFO_STREAM("I am fake sensor:"<<this->world->SimTime().Double());
      ignition::math::Pose3d drone_pose = this->link0->WorldPose();
      ignition::math::Vector3<double> drone_vel_linear = this->link0->WorldLinearVel();
      ignition::math::Vector3<double> drone_vel_angular = this->link0->RelativeAngularVel();
      ignition::math::Vector3<double> drone_acc_linear = this->link0->WorldLinearAccel();
      ignition::math::Vector3<double> drone_acc_angular = this->link0->RelativeAngularAccel();

      flypulator_common_msgs::UavStateStamped uav_state_msg;
      uav_state_msg.header.stamp = ros::Time(this->world->SimTime().Double());
      // pose
      uav_state_msg.pose.position.x = drone_pose.Pos().X();
      uav_state_msg.pose.position.y = drone_pose.Pos().Y();
      uav_state_msg.pose.position.z = drone_pose.Pos().Z();

      // Eigen::Quaterniond q_ItoB
      // (drone_pose.Rot().W(),drone_pose.Rot().X(),drone_pose.Rot().Y(),drone_pose.Rot().Z()); Eigen::Quaterniond
      // q_BtoI (q_ItoB.toRotationMatrix().transpose());
      uav_state_msg.pose.orientation.w = drone_pose.Rot().W();
      uav_state_msg.pose.orientation.x = drone_pose.Rot().X();
      uav_state_msg.pose.orientation.y = drone_pose.Rot().Y();
      uav_state_msg.pose.orientation.z = drone_pose.Rot().Z();
      // velocity
      uav_state_msg.velocity.linear.x = drone_vel_linear.X();
      uav_state_msg.velocity.linear.y = drone_vel_linear.Y();
      uav_state_msg.velocity.linear.z = drone_vel_linear.Z();
      uav_state_msg.velocity.angular.x = drone_vel_angular.X();
      uav_state_msg.velocity.angular.y = drone_vel_angular.Y();
      uav_state_msg.velocity.angular.z = drone_vel_angular.Z();
      // acceleration
      uav_state_msg.acceleration.linear.x = drone_acc_linear.X();
      uav_state_msg.acceleration.linear.y = drone_acc_linear.Y();
      uav_state_msg.acceleration.linear.z = drone_acc_linear.Z();
      uav_state_msg.acceleration.angular.x = drone_acc_angular.X();
      uav_state_msg.acceleration.angular.y = drone_acc_angular.Y();
      uav_state_msg.acceleration.angular.z = drone_acc_angular.Z();

      // ROS_INFO("x noise = %f ", g_noise_generator_x());
      // ROS_INFO("y noise = %f ", g_noise_generator_y());

      flypulator_common_msgs::UavStateStamped uav_state_meas_msg;
      // pose
      uav_state_meas_msg.pose.position.x = drone_pose.Pos().X() + g_noise_generator_x();
      uav_state_meas_msg.pose.position.y = drone_pose.Pos().Y() + g_noise_generator_y();
      uav_state_meas_msg.pose.position.z = drone_pose.Pos().Z() + g_noise_generator_z();

      // add attitude noise using roll pitch yaw representation (yaw-pitch-roll sequence)
      ignition::math::Vector3<double> eul(drone_pose.Rot().Roll(), drone_pose.Rot().Pitch(), drone_pose.Rot().Yaw());
      eul.X() = eul.X() + g_noise_generator_roll();
      eul.Y() = eul.Y() + g_noise_generator_pitch();
      eul.Z() = eul.Z() + g_noise_generator_yaw();
      ignition::math::Quaternion<double> q_n(eul);
      uav_state_meas_msg.pose.orientation.w = q_n.W();
      uav_state_meas_msg.pose.orientation.x = q_n.X();
      uav_state_meas_msg.pose.orientation.y = q_n.Y();
      uav_state_meas_msg.pose.orientation.z = q_n.Z();

      // velocity
      uav_state_meas_msg.velocity.linear.x = drone_vel_linear.X() + g_noise_generator_v_x();
      uav_state_meas_msg.velocity.linear.y = drone_vel_linear.Y() + g_noise_generator_v_y();
      uav_state_meas_msg.velocity.linear.z = drone_vel_linear.Z() + g_noise_generator_v_z();
      uav_state_meas_msg.velocity.angular.x = drone_vel_angular.X() + g_noise_generator_om_x();
      uav_state_meas_msg.velocity.angular.y = drone_vel_angular.Y() + g_noise_generator_om_y();
      uav_state_meas_msg.velocity.angular.z = drone_vel_angular.Z() + g_noise_generator_om_z();
      // acceleration // still very noisy
      uav_state_meas_msg.acceleration.linear.x = drone_acc_linear.X();
      uav_state_meas_msg.acceleration.linear.y = drone_acc_linear.Y();
      uav_state_meas_msg.acceleration.linear.z = drone_acc_linear.Z();
      uav_state_meas_msg.acceleration.angular.x = drone_acc_angular.X();
      uav_state_meas_msg.acceleration.angular.y = drone_acc_angular.Y();
      uav_state_meas_msg.acceleration.angular.z = drone_acc_angular.Z();

      uav_state_meas_msg.header.stamp = ros::Time(this->world->SimTime().Double());

      // publish real states of the simulated drone
      pub_real_state.publish(uav_state_msg);

      // delay measurement messages by multiplies of T_s
      // push message to queue
      queue.push(uav_state_meas_msg);  // pushes new messages to the back
      // at beginning, queue must be filled, so no pop
      if (queue.size() > size_of_queue)
      {
        // publish measured states of the simulated drone
        pub_meas_state.publish(queue.front());  // take first element and publish
        queue.pop();                            // delete first element
      }
    }
    else
      loop_cnt++;

    ros::spinOnce();

    // save pose data to file
    if (write_data_2_file)
    {
      streamDataToFile();
    }
  }

private:
  void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  void streamDataToFile()
  {
    result_file.setf(std::ios::fixed, std::ios::floatfield);
    result_file.precision(5);
    ignition::math::Pose3d drone_pose = this->link0->WorldPose();
    ignition::math::Vector3<double> eul(drone_pose.Rot().Roll(), drone_pose.Rot().Pitch(), drone_pose.Rot().Yaw());
    result_file << this->model->GetWorld()->SimTime().Double() << ",";
    result_file << drone_pose.Pos().X() << "," << drone_pose.Pos().Y() << "," << drone_pose.Pos().Z() << "," << eul.X()
                << "," << eul.Y() << "," << eul.Z();
    result_file << std::endl;
  }

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

  std::ofstream result_file;

  std::queue<flypulator_common_msgs::UavStateStamped> queue;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(FakeSensorPlugin)
}  // namespace gazebo

#endif /*_FAKE_SENSOR_PLUGIN_HH_*/

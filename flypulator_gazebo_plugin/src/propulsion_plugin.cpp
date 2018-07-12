#ifndef _PROPULSION_PLUGIN_HH_
#define _PROPULSION_PLUGIN_HH_

#include <vector>
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>
#include <algorithm>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/gazebo_client.hh>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>

#include <flypulator_common_msgs/Vector6dMsg.h>
#include <flypulator_common_msgs/RotorVelStamped.h>

#include "motor_model.hpp"

namespace gazebo
{
/// \brief A gazebo plugin to simulate propulsion system of the multirotor
class PropulsionPlugin : public ModelPlugin
{
  // TODO: move config param to yaml file
  bool write_data_2_file = false;  // new version with more data (contains also hub force and roll moment)
  bool WRITE_CSV_FILE = false;     // if save the test_data to .csv
  std::string RESULT_CSV_PATH = "/home/jinyao/ros_ws/flypulator/result.csv";
  std::string file_path = "/home/jan/flypulator_ws/src/flypulator/flypulator_gazebo_plugin/rotor_data.csv";
  bool add_wrench_to_drone = true;      // if add force and torque to drone in gazebo
  bool use_ground_effect = false;       // if enable ground effect
  bool use_motor_dynamic_ = true;       // if enable motor dynamic
  bool use_simple_aerodynamic = false;  // use f=k*omega² and tau = b * omega²
  bool bidirectional = true;            // bidirectional option
  // if the rotor vel smaller than 325, we will get negativ CT and moment,
  // caused by the aero dynamic eq.(Hiller's 4.57)
  double vel_min_ = 0.01;    // min rotor speed
  double vel_max_ = 100000;  // max rotor speed
  double k_simple_aero_ = 0.0138;
  double b_simple_aero_ = 0.00022;

  double test_data[12];  // test data for debug

  // internal parameters
  double uav_mass_;                           // drone mass
  double ground_effect_coeff_;                // ground effect coefficient
  double blade_pitch_angle_;                  // blade pitch angle
  double wing_area_;                          // wing area
  double rv = 13.6 * M_PI / 180.0;            // rotor_axis_vertical_axis_angle cos(rv)=cos(pitch)*cos(yaw)
  double solidity_;                           // rotor solidity
  double wind_vx_ = 1e-20;                    // wind velocity in global x
  double wind_vy_ = 1e-20;                    // wind velocity in global y
  double wind_vz_ = 1e-20;                    // wind velocity in global z
  double uav_vx_;                             // drone velocity in global x
  double uav_vy_;                             // drone velocity in global y
  double uav_vz_;                             // drone velocity in global z
  double air_global_vx_ = 1e-20;              // air velocity in global x
  double air_global_vy_ = 1e-20;              // air velocity in global y
  double air_global_vz_ = 1e-20;              // air velocity in global z
  double ind_vel_hover_;                      // induced velocity in the hovering case
  std::vector<std::string> joint_names_;      // joint names of propeller
  std::vector<std::string> link_names_;       // link names of propeller
  std::map<std::string, double> aero_param_;  // map of aerodynamic parameters

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

  gazebo::physics::LinkPtr rotor_link_ptr_[6];  // pointer to rotor links

  bool add_dist_ = true;                                                                  // add disturbances
  ignition::math::Vector3<double> distforce_ = ignition::math::Vector3<double>(0, 0, 0);  // disturbance force

  /// \brief Constructor
public:
  PropulsionPlugin()
  {
  }

public:
  ~PropulsionPlugin()
  {
  }

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    ROS_INFO_STREAM("Loading PropulsionPlugin ...");

    // load aerodynamic parameters
    this->readParamsFromServer();

    if (_model->GetJointCount() == 0)
    {
      ROS_ERROR("Invalid joint count, plugin not loaded");
      return;
    }

    // save test data
    if (WRITE_CSV_FILE)
    {
      std::ofstream fout(RESULT_CSV_PATH, std::ios::out);
      fout.close();
    }

    if (write_data_2_file)
    {
      result_file.open(file_path);
      result_file << "time,rotor,omega,force_x,force_y,force_z,torque_x,torque_y,torque_z" << std::endl;
    }

    // Store the model pointer for convenience.
    this->model_ = _model;
    // Get the first joint. We are making an assumption about the model
    // having six joints that is the rotational joint.
    this->joint1_ = _model->GetJoint(joint_names_[0]);
    this->joint2_ = _model->GetJoint(joint_names_[1]);
    this->joint3_ = _model->GetJoint(joint_names_[2]);
    this->joint4_ = _model->GetJoint(joint_names_[3]);
    this->joint5_ = _model->GetJoint(joint_names_[4]);
    this->joint6_ = _model->GetJoint(joint_names_[5]);
    // set joint velocity using joint motors to set joint velocity
    this->joint1_->SetParam("fmax", 0, 1000000.0);  // fmax: maximum joint force or torque
    this->joint2_->SetParam("fmax", 0, 1000000.0);
    this->joint3_->SetParam("fmax", 0, 1000000.0);
    this->joint4_->SetParam("fmax", 0, 1000000.0);
    this->joint5_->SetParam("fmax", 0, 1000000.0);
    this->joint6_->SetParam("fmax", 0, 1000000.0);
    this->joint1_->SetParam("vel", 0, 0);
    this->joint2_->SetParam("vel", 0, 0);
    this->joint3_->SetParam("vel", 0, 0);
    this->joint4_->SetParam("vel", 0, 0);
    this->joint5_->SetParam("vel", 0, 0);
    this->joint6_->SetParam("vel", 0, 0);

    // get the six blade link
    this->link0_ = _model->GetChildLink("base_link");
    for (int i = 0; i < aero_param_["N"]; i++)
    {
      // rotor_link_ptr_[i] = _model->GetChildLink(std::string("blade_link_") + std::to_string(i+1));
      rotor_link_ptr_[i] = _model->GetChildLink(link_names_[i]);
    }

    // calculate mass of drone from model
    uav_mass_ = this->link0_->GetInertial()->Mass();  // mass of the base_link
    // the mass of "base_link" is included the
    // mass of all components that fix to the "base_link",
    // here included 6 arm+motor and 2 leg of the drone
    for (int i = 0; i < aero_param_["N"]; i++)
    {
      uav_mass_ += rotor_link_ptr_[i]->GetInertial()->Mass();
    }
    ROS_INFO_STREAM("Mass of drone : " << uav_mass_ << "kg");  // show mass of drone

    solidity_ = (aero_param_["N"] * aero_param_["c"]) / (M_PI * aero_param_["R"]);  // rotor solidity
    wing_area_ = M_PI * pow(aero_param_["R"], 2);                                   // wing area
    // induced airflow velocity in hovering case
    // multiplied by 1/B according to Hiller eq. 4.58
    ind_vel_hover_ =
        -1 / aero_param_["B"] *
        sqrt((uav_mass_ * aero_param_["g"]) / (2 * aero_param_["N"] * aero_param_["pho"] * wing_area_ * cos(rv)));
    blade_pitch_angle_ = aero_param_["th0"] - 0.75 * aero_param_["thtw"];  // blade pitch angle

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      ROS_WARN_STREAM("Create gazebo_client node");
    }
    // Create our ROS node.
    this->rosNode_.reset(new ros::NodeHandle("gazebo_client"));
    ROS_INFO_STREAM("propulsion_plugin get node:" << this->rosNode_->getNamespace());

    this->pub_thrust_torque_ratio_ =
        this->rosNode_->advertise<flypulator_common_msgs::Vector6dMsg>("/drone/thrust_moment_ratio", 10);
    this->pub_joint_state_ = this->rosNode_->advertise<sensor_msgs::JointState>("/drone/joint_states", 50);

    for (int i = 0; i < aero_param_["N"]; i++)
    {
      this->pub_link_wrench_[i] = this->rosNode_->advertise<geometry_msgs::WrenchStamped>(
          std::string("/drone/blade_") + std::to_string(i + 1) + std::string("_wrench"), 100);
    }

    // Create a wind velocity topic and subscribe to it
    ros::SubscribeOptions sub_wind_cmd = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
        "/drone/wind_cmd", 100, boost::bind(&PropulsionPlugin::OnRosWindMsg, this, _1), ros::VoidPtr(),
        &this->rosQueue_);
    this->rosSubWind_ = this->rosNode_->subscribe(sub_wind_cmd);

    // subscribe to control signal,six global velocity for drone
    ros::SubscribeOptions sub_rotor_cmd = ros::SubscribeOptions::create<flypulator_common_msgs::RotorVelStamped>(
        "/drone/rotor_cmd", 100, boost::bind(&PropulsionPlugin::OnControlMsg, this, _1), ros::VoidPtr(),
        &this->rosQueue_);
    this->rosSubControl_ = this->rosNode_->subscribe(sub_rotor_cmd);

    // Spin up the queue helper thread.
    this->rosQueueThread_ = std::thread(std::bind(&PropulsionPlugin::QueueThread, this));

    // This event is broadcast every simulation iteration.
    this->updateConnection_ =
        event::Events::ConnectWorldUpdateBegin(boost::bind(&PropulsionPlugin::OnUpdate, this, _1));

    ROS_INFO_STREAM("PropulsionPlugin Loaded !");
  }

public:
  void OnUpdate(const common::UpdateInfo &_info)  // update rate = 1kHz
  {
    // ROS_INFO_STREAM("propulsion plugin: OnUpdate()!");

    double curr_time = this->model_->GetWorld()->SimTime().Double();
    static double last_time = curr_time;
    double dt = curr_time - last_time;
    last_time = curr_time;
    // ROS_ERROR_STREAM("dt:"<<dt);
    if (dt < 0)
    {
      ROS_WARN_STREAM("Move backward in time, reset Motor!");
      motor1_.reset();
      motor2_.reset();
      motor3_.reset();
      motor4_.reset();
      motor5_.reset();
      motor6_.reset();
      return;
    }

    // motors dynamic
    if (use_motor_dynamic_)
    {
      rotor_vel_raw_[0] = motor1_.update(rotor_vel_cmd_[0], dt);
      rotor_vel_raw_[1] = motor2_.update(rotor_vel_cmd_[1], dt);
      rotor_vel_raw_[2] = motor3_.update(rotor_vel_cmd_[2], dt);
      rotor_vel_raw_[3] = motor4_.update(rotor_vel_cmd_[3], dt);
      rotor_vel_raw_[4] = motor5_.update(rotor_vel_cmd_[4], dt);
      rotor_vel_raw_[5] = motor6_.update(rotor_vel_cmd_[5], dt);
    }
    else
    {
      for (int i = 0; i < aero_param_["N"]; i++)
        rotor_vel_raw_[i] = rotor_vel_cmd_[i];
    }

    // check direction of the rotors' velocity
    for (int i = 0; i < aero_param_["N"]; i++)
    {
      if (bidirectional)
      {  // use bi-direction
        if (rotor_vel_raw_[i] < 0)
        {
          rotor_vel_[i] = -rotor_vel_raw_[i];
          dir_vel_actual_[i] = -dir_spin_default_[i];  // inverse rotating direction
          dir_thrust_[i] = -1;
        }
        else
        {
          rotor_vel_[i] = rotor_vel_raw_[i];
          dir_vel_actual_[i] = dir_spin_default_[i];  // keep default rotating direction
          dir_thrust_[i] = 1;
        }
      }
      else
      {  // use uni-direction

        if (rotor_vel_raw_[i] < 0)
          rotor_vel_[i] = 0;
        else
          rotor_vel_[i] = rotor_vel_raw_[i];

        dir_vel_actual_[i] = dir_spin_default_[i];
        dir_thrust_[i] = 1;
      }
    }

    // clamp rotors angular velocity
    for (int i = 0; i < aero_param_["N"]; i++)
      rotor_vel_[i] = clamp(rotor_vel_[i], vel_min_, vel_max_);

    // altitude/height of the drone to the gorund
    double drone_height = this->link0_->WorldPose().Pos().Z();

    // TODO: only approximate ground effect, neglect the tilting angle of the drone and propeller
    // calculate ground effect coefficient, T_eff = T_nom/ground_effect_coeff_
    if (drone_height > 0.35 * aero_param_["R"])                                    // R = blade radius
      ground_effect_coeff_ = 1.0 - pow(aero_param_["R"] / (4 * drone_height), 2);  // min.~0.5
    else
      ground_effect_coeff_ = 0.5;

    // if not using ground effect reset the coeff
    if (!use_ground_effect)
      ground_effect_coeff_ = 1.0;

    // ROS_INFO_STREAM("k:"<<motor1_.getK()<<","<<"T:"<<motor1_.getT()<<","<<"omega:"<<motor1_.getOmega());

    uav_vx_ = this->model_->RelativeLinearVel().X();
    uav_vy_ = this->model_->RelativeLinearVel().Y();
    uav_vz_ = this->model_->RelativeLinearVel().Z();
    air_global_vx_ = wind_vx_ - uav_vx_;
    air_global_vy_ = wind_vy_ - uav_vy_;
    air_global_vz_ = wind_vz_ - uav_vz_;
    Eigen::Vector3d V_airflow(air_global_vx_, air_global_vy_, air_global_vz_);

    for (int i = 0; i < aero_param_["N"]; i++)
    {
      Eigen::Quaterniond q(rotor_link_ptr_[i]->WorldPose().Rot().W(), rotor_link_ptr_[i]->WorldPose().Rot().X(),
                           rotor_link_ptr_[i]->WorldPose().Rot().Y(), rotor_link_ptr_[i]->WorldPose().Rot().Z());
      Eigen::Matrix3d t_matrix = q.toRotationMatrix();
      Eigen::Matrix3d t_matrix_trans = t_matrix.transpose();
      Eigen::Vector3d v_local = t_matrix_trans * V_airflow;

      // now we are in local rotor frame {R}_i
      double v_z = v_local.z();
      double v_xy = sqrt(pow(v_local.x(), 2) + pow(v_local.y(), 2));
      double C = v_z / ind_vel_hover_;
      double v_i = 0;

      // calculate induced velocity, depends on climb ratio C
      // TODO fix discontinuity with Bezier Spline (Hiller, App. A.1.2)
      if (C >= 0)
      {
        // Vi1 = -(sqrt(pow((Vzz1 / 2), 2) + pow(ind_vel_hover_, 2)) + Vzz1 / 2); //Vi induced airflow velocity
        v_i = -v_z / 2.0f + sqrt(pow((v_z / 2.0f), 2) + pow(ind_vel_hover_, 2));
      }
      else if (C >= -2 && C < 0)
      {
        v_i = -ind_vel_hover_ * (aero_param_["k0"] + aero_param_["k1"] * C + aero_param_["k2"] * pow(C, 2) +
                                 aero_param_["k3"] * pow(C, 3) + aero_param_["k4"] * pow(C, 4));
      }
      else
      {
        v_i = -v_z / 2 - sqrt(pow((v_z / 2), 2) - pow(ind_vel_hover_, 2));
      }

      double lambda = (v_i + v_z) / (rotor_vel_[i] * aero_param_["R"]);  // Hiller, eq. 4.27
      double mu = v_xy / (rotor_vel_[i] * aero_param_["R"]);             // Hiller, eq. 4.40
      double CT = solidity_ * aero_param_["a"] *
                  ((blade_pitch_angle_ / 3) * (pow(aero_param_["B"], 3) + 3 * mu * mu * aero_param_["B"] / 2.0) -
                   lambda * aero_param_["B"] * aero_param_["B"] / 2.0);  // Hiller's (4.57)

      aero_force_[i].z() =
          dir_thrust_[i] * 0.5 * aero_param_["pho"] * CT * wing_area_ * pow(rotor_vel_[i] * aero_param_["R"], 2);

      double alpha = atan2(v_local.y(), v_local.x());  // orientation of Vxy in blade coordinate
      double f_hub = 0.25 * solidity_ * aero_param_["pho"] * wing_area_ * aero_param_["CD0"] *
                     pow(rotor_vel_[i] * aero_param_["R"], 2) * v_xy;  // H-force, Hiller eq. 4.63 & 4.64
      aero_force_[i].x() = f_hub * cos(alpha);                         // H-force in x direction
      aero_force_[i].y() = f_hub * sin(alpha);                         // H-force in y direction

      double CQ = aero_param_["ki"] * lambda * CT +
                  0.25 * solidity_ * aero_param_["CD0"] * (1 + aero_param_["k"] * pow(mu, 2));  // Hiller's (4.62)

      aero_torque_[i].z() = 0.5 * aero_param_["pho"] * pow((rotor_vel_[i] * aero_param_["R"]), 2) * wing_area_ *
                            aero_param_["R"] * CQ * dir_spin_default_[i];  // Hiller's (4.60)

      double moment_roll = 0.125 * solidity_ * aero_param_["a"] * aero_param_["pho"] * aero_param_["R"] * wing_area_ *
                           mu * ((4 / 3) * aero_param_["th0"] - aero_param_["thtw"] - lambda) *
                           pow((rotor_vel_[i] * aero_param_["R"]), 2);  // Hiller eq. 4.65 and 4.66
      aero_torque_[i].x() = moment_roll * cos(alpha);
      aero_torque_[i].y() = moment_roll * sin(alpha);

      // apply to uav
      if (add_wrench_to_drone)
      {
        if (use_simple_aerodynamic)
        {
          rotor_link_ptr_[i]->AddRelativeForce(
              ignition::math::Vector3<double>(0, 0, dir_thrust_[i] * k_simple_aero_ * pow(rotor_vel_[i], 2)));
          rotor_link_ptr_[i]->AddRelativeTorque(
              ignition::math::Vector3<double>(0, 0, dir_spin_default_[i] * b_simple_aero_ * pow(rotor_vel_[i], 2)));
        }
        else
        {
          rotor_link_ptr_[i]->AddRelativeForce(ignition::math::Vector3<double>(
              aero_force_[i].x(), aero_force_[i].y(), aero_force_[i].z() / ground_effect_coeff_));
          rotor_link_ptr_[i]->AddRelativeTorque(
              ignition::math::Vector3<double>(aero_torque_[i].x(), aero_torque_[i].y(), aero_torque_[i].z()));
        }
      }

    }  // end for loop

    if (add_dist_)
    {
      this->link0_->AddForce(distforce_);
      // this->link0_->AddTorque(distforce_);
      // this->link0_->AddTorque(ignition::math::Vector3<double>(1,1,1));
    }

    // print to file
    if (write_data_2_file)
    {
      streamDataToFile();
    }

    // TODO: set velocity in onUpdate() or onCtrolMsg()???
    this->SetVelocity();

    // save data for csv output
    test_data[0] = rotor_vel_[0] * dir_vel_actual_[0];
    test_data[1] = rotor_vel_[1] * dir_vel_actual_[1];
    test_data[2] = rotor_vel_[2] * dir_vel_actual_[2];
    test_data[3] = rotor_vel_[3] * dir_vel_actual_[3];
    test_data[4] = rotor_vel_[4] * dir_vel_actual_[4];
    test_data[5] = rotor_vel_[5] * dir_vel_actual_[5];
    test_data[6] = aero_torque_[0].z();
    test_data[7] = aero_torque_[1].z();
    test_data[8] = aero_torque_[2].z();
    test_data[9] = aero_torque_[3].z();
    test_data[10] = aero_torque_[4].z();
    test_data[11] = aero_torque_[5].z();

    // test_data[0] = CT1;
    // test_data[1] = CT2;
    // test_data[2] = CT3;
    // test_data[3] = CT4;
    // test_data[4] = CT5;
    // test_data[5] = CT6;
    ros::spinOnce();

    /*
    double ratio1, ratio2, ratio3, ratio4, ratio5, ratio6;
    ratio1 = -3 * aero_param_["R"] * CQ1 * Sgn(this->link1->GetRelativeAngularVel().z) / (s * aero_param_["a"] * pa *
    pow(aero_param_["B"], 3)) * dir_thrust_[0]; ratio2 = -3 * aero_param_["R"] * CQ2 *
    Sgn(this->link2->GetRelativeAngularVel().z) / (s * aero_param_["a"] * pa * pow(aero_param_["B"], 3)) *
    dir_thrust_[1]; ratio3 = -3 * aero_param_["R"] * CQ3 * Sgn(this->link3->GetRelativeAngularVel().z) / (s *
    aero_param_["a"] * pa * pow(aero_param_["B"], 3)) * dir_thrust_[2]; ratio4 = -3 * aero_param_["R"] * CQ4 *
    Sgn(this->link4->GetRelativeAngularVel().z) / (s * aero_param_["a"] * pa * pow(aero_param_["B"], 3)) *
    dir_thrust_[3]; ratio5 = -3 * aero_param_["R"] * CQ5 * Sgn(this->link5->GetRelativeAngularVel().z) / (s *
    aero_param_["a"] * pa * pow(aero_param_["B"], 3)) * dir_thrust_[4]; ratio6 = -3 * aero_param_["R"] * CQ6 *
    Sgn(this->link6->GetRelativeAngularVel().z) / (s * aero_param_["a"] * pa * pow(aero_param_["B"], 3)) *
    dir_thrust_[5];

    flypulator_common_msgs::Vector6dMsg _msg;
    _msg.x1 = ratio1;
    _msg.x2 = ratio2;
    _msg.x3 = ratio3;
    _msg.x4 = ratio4;
    _msg.x5 = ratio5;
    _msg.x6 = ratio6;

    this->pub_thrust_torque_ratio_.publish(_msg);

    */

    // publish tf base_link ---> world
    static int publish_cnt = 0;
    // publish rate = 1000/(10) = 100 [Hz]
    if (publish_cnt >= (10 - 1))
    {
      publish_cnt = 0;
      tfPublisher();
      jointStatePubliher();
      wrenchPublisher();

      if (WRITE_CSV_FILE)
      {
        // if (rotor_vel_[0] < 2500 && rotor_vel_[0] >= 100)
        {
          std::ofstream result_file(RESULT_CSV_PATH, std::ios::app);
          result_file.setf(std::ios::fixed, std::ios::floatfield);
          result_file.precision(5);
          result_file << this->model_->GetWorld()->SimTime().Double() << "," << test_data[0] << "," << test_data[1]
                      << "," << test_data[2] << "," << test_data[3] << "," << test_data[4] << "," << test_data[5] << ","
                      << test_data[6] << "," << test_data[7] << "," << test_data[8] << "," << test_data[9] << ","
                      << test_data[10] << "," << test_data[11] << "," << std::endl;
          result_file.close();
        }
      }
    }
    else
    {
      publish_cnt++;
    }
  }

public:
  void OnRosWindMsg(const geometry_msgs::Vector3ConstPtr &_wind_msg)
  {
    // wind_vx_ = _wind_msg->x;
    // wind_vy_ = _wind_msg->y;
    // wind_vz_ = _wind_msg->z;
    ROS_INFO("add wind disturbance");
    distforce_ = ignition::math::Vector3<double>(_wind_msg->x, _wind_msg->y, _wind_msg->z);
  }

public:
  void OnControlMsg(const flypulator_common_msgs::RotorVelStampedConstPtr &_msg)
  {
    // TODO: replace N with sizeof(joint_names_)
    if (_msg->velocity.size() != size_t(aero_param_["N"]))
    {
      ROS_ERROR("Dimention of rotor cmd does not match joint number!");
      return;
    }

    for (int i = 0; i < int(aero_param_["N"]); i++)
      rotor_vel_cmd_[i] = _msg->velocity[i];

    // ROS_INFO_STREAM("aero:"<<_msg->velocity[0]<<","<<_msg->velocity[1]<<","<<_msg->velocity[2]<<","<<_msg->velocity[3]<<","<<_msg->velocity[4]<<","<<_msg->velocity[5]);
    // ROS_INFO_STREAM("aero:"<<rotor_vel_[0]<<","<<rotor_vel_[1]<<","<<rotor_vel_[2]<<","<<rotor_vel_[3]<<","<<rotor_vel_[4]<<","<<rotor_vel_[5]);
  }

private:
  void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode_->ok())
    {
      this->rosQueue_.callAvailable(ros::WallDuration(timeout));
    }
  }
  // return sgn information
public:
  int Sgn(const double &num)
  {
    if (num < 0)
      return -1;
    else if (num > 0)
      return 1;
    else
      return 0;
  }

public:
  double clamp(double x, double low, double high)
  {
    if (x > high)
      return high;
    if (x < low)
      return low;
    return x;
  }

  // apply velocity to joints with 3 methods
public:
  void SetVelocity()
  {
    this->joint1_->SetParam("vel", 0, rotor_vel_[0] * dir_vel_actual_[0]);
    this->joint2_->SetParam("vel", 0, rotor_vel_[1] * dir_vel_actual_[1]);
    this->joint3_->SetParam("vel", 0, rotor_vel_[2] * dir_vel_actual_[2]);
    this->joint4_->SetParam("vel", 0, rotor_vel_[3] * dir_vel_actual_[3]);
    this->joint5_->SetParam("vel", 0, rotor_vel_[4] * dir_vel_actual_[4]);
    this->joint6_->SetParam("vel", 0, rotor_vel_[5] * dir_vel_actual_[5]);
    // ROS_DEBUG_STREAM("aero:"<<rotor_vel_[0]<<","<<rotor_vel_[1]<<","<<rotor_vel_[2]<<","<<rotor_vel_[3]<<","<<rotor_vel_[4]<<","<<rotor_vel_[5]);
    // ROS_DEBUG_STREAM("aero plugin: SetVelocity()!");
  }

  // load parameter from ROS parameter server
private:
  void readParamsFromServer()
  {
    // try to read joint names from ros parameter server
    if (ros::param::get("urdf/controller_joint_names", joint_names_))
    {
      for (auto i : joint_names_)
      {
        ROS_DEBUG_STREAM("propulsion_plugin: loaded control joint names : " << i);
      }
      ROS_INFO_STREAM("propulsion_plugin: joint names loaded.");
    }
    else
    {
      ROS_ERROR_STREAM("Can't load joint names from parameter server!");
    }

    // read links names from ros parameter server
    if (ros::param::get("urdf/controller_link_names", link_names_))
    {
      for (auto i : link_names_)
      {
        ROS_DEBUG_STREAM("propulsion_plugin: loaded control link names : " << i);
      }
      ROS_INFO_STREAM("propulsion_plugin: link names loaded.");
    }
    else
    {
      ROS_ERROR_STREAM("Can't load link names from parameter server!");
    }

    // try to read aerodynamic parameter from ros parameter server
    if (ros::param::get("/aero_param", aero_param_))
    {
      ROS_INFO_STREAM("propulsion_plugin: aerodynamic parameters loaded.");
      for (auto elem : aero_param_)
      {
        ROS_DEBUG_STREAM("propulsion_plugin: aeroparam/" << elem.first << ": " << elem.second);
      }
    }
    else
    {
      ROS_WARN_STREAM("No aerodynamic parameters availiable, use default values...");
      aero_param_["N"] = 6;
      aero_param_["c"] = 0.016;
      aero_param_["R"] = 0.75;
      aero_param_["a"] = 5.7;
      aero_param_["th0"] = 0.7;
      aero_param_["thtw"] = 0.5;
      aero_param_["B"] = 0.98;
      aero_param_["pho"] = 1.2;
      aero_param_["ki"] = 1.15;
      aero_param_["k"] = 4.65;
      aero_param_["k0"] = 1.15;
      aero_param_["k1"] = -1.125;
      aero_param_["k2"] = -1.372;
      aero_param_["k3"] = -1.718;
      aero_param_["k4"] = -0.655;
      aero_param_["CD0"] = 0.04;
      // TODO: move g to uav_parameter.yaml
      aero_param_["g"] = 9.81;
    }
  }

private:
  void tfPublisher()
  {
    static tf::TransformBroadcaster T_br;

    tf::Transform T_tmp;
    ignition::math::Pose3d pose_tmp;

    pose_tmp = this->link0_->WorldPose();
    T_tmp.setOrigin(tf::Vector3(pose_tmp.Pos().X(), pose_tmp.Pos().Y(), pose_tmp.Pos().Z()));
    T_tmp.setRotation(tf::Quaternion(pose_tmp.Rot().X(), pose_tmp.Rot().Y(), pose_tmp.Rot().Z(), pose_tmp.Rot().W()));
    T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "world", "base_link"));

    // pose_tmp = this->link1->GetRelativePose();
    // T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    // T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x,pose_tmp.rot.y,pose_tmp.rot.z,pose_tmp.rot.w));
    // T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "base_link", link_names_[0]));

    // pose_tmp = this->link2->GetRelativePose();
    // T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    // T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x,pose_tmp.rot.y,pose_tmp.rot.z,pose_tmp.rot.w));
    // T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "base_link", "link_names_[1]"));

    // pose_tmp = this->link3->GetRelativePose();
    // T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    // T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x,pose_tmp.rot.y,pose_tmp.rot.z,pose_tmp.rot.w));
    // T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "base_link", "link_names_[2]"));

    // pose_tmp = this->link4->GetRelativePose();
    // T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    // T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x,pose_tmp.rot.y,pose_tmp.rot.z,pose_tmp.rot.w));
    // T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "base_link", "link_names_[3]"));

    // pose_tmp = this->link5->GetRelativePose();
    // T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    // T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x,pose_tmp.rot.y,pose_tmp.rot.z,pose_tmp.rot.w));
    // T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "base_link", "link_names_[4]"));

    // pose_tmp = this->link6->GetRelativePose();
    // T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    // T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x,pose_tmp.rot.y,pose_tmp.rot.z,pose_tmp.rot.w));
    // T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "base_link", "link_names_[5]"));
  }
  // publish joint state
private:
  void jointStatePubliher()
  {
    // publish joint state
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name.resize(6);
    joint_state_msg.position.resize(6);
    joint_state_msg.velocity.resize(6);
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.header.frame_id = "base_link";
    joint_state_msg.name[0] = this->joint1_->GetName();
    joint_state_msg.name[1] = this->joint2_->GetName();
    joint_state_msg.name[2] = this->joint3_->GetName();
    joint_state_msg.name[3] = this->joint4_->GetName();
    joint_state_msg.name[4] = this->joint5_->GetName();
    joint_state_msg.name[5] = this->joint6_->GetName();

    joint_state_msg.position[0] = this->joint1_->Position(0);
    joint_state_msg.position[1] = this->joint2_->Position(0);
    joint_state_msg.position[2] = this->joint3_->Position(0);
    joint_state_msg.position[3] = this->joint4_->Position(0);
    joint_state_msg.position[4] = this->joint5_->Position(0);
    joint_state_msg.position[5] = this->joint6_->Position(0);

    pub_joint_state_.publish(joint_state_msg);
  }

private:
  void wrenchPublisher()
  {
    geometry_msgs::WrenchStamped wrench_msg_tmp;
    wrench_msg_tmp.header.stamp = ros::Time::now();

    for (int i = 0; i < 6; i++)
    {
      wrench_msg_tmp.header.frame_id = std::string("motor_link_") + std::to_string(i + 1);
      wrench_msg_tmp.wrench.force.x = aero_force_[i].x();
      wrench_msg_tmp.wrench.force.y = aero_force_[i].y();
      wrench_msg_tmp.wrench.force.z = aero_force_[i].z();
      wrench_msg_tmp.wrench.torque.x = aero_torque_[i].x();
      wrench_msg_tmp.wrench.torque.y = aero_torque_[i].y();
      wrench_msg_tmp.wrench.torque.z = aero_torque_[i].z();
      this->pub_link_wrench_[i].publish(wrench_msg_tmp);
    }
  }

  void streamDataToFile()
  {
    result_file.setf(std::ios::fixed, std::ios::floatfield);
    result_file.precision(5);
    result_file << this->model_->GetWorld()->SimTime().Double() << ",";
    for (int i = 0; i < 6; i++)
    {
      result_file << "rotor" << i << "," << rotor_vel_[i] * rotor_vel_raw_[i] << "," << aero_force_[i].x() << ","
                  << aero_force_[i].y() << "," << aero_force_[i].z() << "," << aero_torque_[i].x() << ","
                  << aero_torque_[i].y() << "," << aero_torque_[i].z() << ",";
    }
    result_file << std::endl;

    // result_file.close();
  }

  // dynamic model of the motors
private:
  flypulator::MotorModel motor1_;
  flypulator::MotorModel motor2_;
  flypulator::MotorModel motor3_;
  flypulator::MotorModel motor4_;
  flypulator::MotorModel motor5_;
  flypulator::MotorModel motor6_;

  /// \brief Pointer to the model.
private:
  physics::ModelPtr model_;

private:
  physics::JointPtr joint1_;

private:
  physics::JointPtr joint2_;

private:
  physics::JointPtr joint3_;

private:
  physics::JointPtr joint4_;

private:
  physics::JointPtr joint5_;

private:
  physics::JointPtr joint6_;

private:
  physics::LinkPtr link0_;

  /// \brief A node use for ROS transport
private:
  std::unique_ptr<ros::NodeHandle> rosNode_;

private:
  event::ConnectionPtr updateConnection_;

private:
  ros::Subscriber rosSubWind_;

private:
  ros::Subscriber rosSubControl_;

  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue_;

  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread_;

private:
  ros::Publisher pub_thrust_torque_ratio_;
  ros::Publisher pub_joint_state_;
  ros::Publisher pub_link_wrench_[6];

  std::ofstream result_file;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(PropulsionPlugin)
}  // namespace gazebo

#endif /*_PROPULSION_PLUGIN_HH_*/

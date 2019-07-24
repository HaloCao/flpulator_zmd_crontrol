#include "propulsion_plugin.hpp"

namespace gazebo
{
PropulsionPlugin::PropulsionPlugin()
{
  // set logger level
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  // default configuration
  write_data_2_file_ = false;
  WRITE_CSV_FILE_ = false;
  RESULT_CSV_PATH_ = "/home/jinyao/ros_ws/flypulator/result.csv";
  file_path_ = "/home/jan/flypulator_ws/src/flypulator/flypulator_gazebo_plugin/rotor_data.csv";
  add_wrench_to_drone_ = true;
  use_ground_effect_ = false;
  use_motor_dynamic_ = true;
  use_simple_aerodynamic_ = true;
  add_dist_ = false;
  dist_force_ = ignition::math::Vector3<double>(0, 0, 0);
  dist_torque_ = ignition::math::Vector3<double>(0, 0, 0);
  // if the rotor vel smaller than 325, we will get negativ CT and moment,
  // caused by the aero dynamic eq.(Hiller's 4.57)
  vel_min_ = 0.01;
  vel_max_ = 100000;
  k_simple_aero_ = 0.0000427;
  b_simple_aero_ = 0.0000022;
  tilting_angle_ = 13.6;
  bidirectional_ = true;
  joint_cnt_ = 6;

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
  aero_param_["g"] = 9.81;

  wind_vx_ = 1e-20;
  wind_vy_ = 1e-20;
  wind_vz_ = 1e-20;

  air_global_vx_ = 1e-20;
  air_global_vy_ = 1e-20;
  air_global_vz_ = 1e-20;
}

void PropulsionPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO_STREAM("Loading PropulsionPlugin ...");

  // load aerodynamic parameters
  this->readParamsFromServer();

  if (_model->GetJointCount() == 0 || joint_cnt_ != 6)  // this plugin is only valid for Hexarotor for now!
  {
    ROS_ERROR("Invalid joint count, plugin not loaded");
    return;
  }

  // save test data
  if (WRITE_CSV_FILE_)
  {
    std::ofstream fout(RESULT_CSV_PATH_, std::ios::out);
    fout.close();
  }

  if (write_data_2_file_)
  {
    result_file.open(file_path_);
    result_file << "time,rotor,omega,force_x,force_y,force_z,torque_x,torque_y,torque_z" << std::endl;
  }

  // Store the model pointer for convenience.
  this->model_ = _model;

  for (unsigned int i = 0; i < joint_cnt_; i++)
  {
    // get control related joints
    this->ctrl_joint_ptr_[i] = _model->GetJoint(joint_names_[i]);

    // set joint velocity using joint motors: http://gazebosim.org/tutorials?tut=set_velocity#SetVelocityWithJointMotors
    this->ctrl_joint_ptr_[i]->SetParam("fmax", 0, 1000000.0);
    this->ctrl_joint_ptr_[i]->SetParam("vel", 0, 0);
  }

  // get control related links
  this->link0_ = _model->GetChildLink("base_link");
  for (unsigned int i = 0; i < joint_cnt_; i++)
  {
    this->ctrl_link_ptr_[i] = _model->GetChildLink(link_names_[i]);
  }

  // calculate mass of drone from model
  uav_mass_ = this->link0_->GetInertial()->Mass();  // mass of the base_link
  // the mass of "base_link" is included the
  // mass of all components that fix to the "base_link",
  // here included 6 arm+motor and 2 leg of the drone
  for (unsigned int i = 0; i < joint_cnt_; i++)
  {
    uav_mass_ += ctrl_link_ptr_[i]->GetInertial()->Mass();
  }
  ROS_INFO_STREAM("UAV Mass: " << uav_mass_ << "kg");  // show mass of drone

  rv = tilting_angle_ * M_PI / 180.0;
  // TODO: wrong equation!!!
  solidity_ = (double(joint_cnt_) * aero_param_["c"]) / (M_PI * aero_param_["R"]);  // rotor solidity
  wing_area_ = M_PI * pow(aero_param_["R"], 2);                                     // wing area
  // induced airflow velocity in hovering case
  // multiplied by 1/B according to Hiller eq. 4.58
  ind_vel_hover_ =
      -1 / aero_param_["B"] *
      sqrt((uav_mass_ * aero_param_["g"]) / (2 * double(joint_cnt_) * aero_param_["pho"] * wing_area_ * cos(rv)));
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

  // this->pub_thrust_torque_ratio_ =
  //   this->rosNode_->advertise<flypulator_common_msgs::Vector6dMsg>("/drone/thrust_moment_ratio", 10);
  this->pub_joint_state_ = this->rosNode_->advertise<sensor_msgs::JointState>("/drone/joint_states", 50);

  for (unsigned int i = 0; i < joint_cnt_; i++)
  {
    this->pub_link_wrench_[i] = this->rosNode_->advertise<geometry_msgs::WrenchStamped>(
        std::string("/drone/blade_") + std::to_string(i + 1) + std::string("_wrench"), 100);
  }

  // Create a wind velocity topic and subscribe to it (invalid for simple aerodynamics mode)
  if (!use_simple_aerodynamic_)
  {
    ros::SubscribeOptions sub_wind_cmd = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
        "/drone/wind_cmd", 100, boost::bind(&PropulsionPlugin::onRosWindMsg, this, _1), ros::VoidPtr(),
        &this->rosQueue_);
    this->rosSubWind_ = this->rosNode_->subscribe(sub_wind_cmd);
  }

  // Create a external wrench topic and subscribe to it (user defined external force and torque)
  ros::SubscribeOptions sub_ext_dist = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
      "/drone/dist_wrench", 100, boost::bind(&PropulsionPlugin::onRosDistMsg, this, _1), ros::VoidPtr(),
      &this->rosQueue_);
  this->rosSubDist_ = this->rosNode_->subscribe(sub_ext_dist);

  // subscribe to control signal,six global velocity for drone
  ros::SubscribeOptions sub_rotor_cmd = ros::SubscribeOptions::create<flypulator_common_msgs::RotorVelStamped>(
      "/drone/rotor_cmd", 100, boost::bind(&PropulsionPlugin::onControlMsg, this, _1), ros::VoidPtr(),
      &this->rosQueue_);
  this->rosSubControl_ = this->rosNode_->subscribe(sub_rotor_cmd);

  // Spin up the queue helper thread.
  this->rosQueueThread_ = std::thread(std::bind(&PropulsionPlugin::queueThread, this));

  // This event is broadcast every simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&PropulsionPlugin::onUpdate, this, _1));

  ROS_INFO("Loading PropulsionPlugin finished successfully!");
}

void PropulsionPlugin::onUpdate(const common::UpdateInfo &_info)  // update rate = 1kHz
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
    for (unsigned int i = 0; i < joint_cnt_; i++)
      rotor_vel_raw_[i] = rotor_vel_cmd_[i];
  }
  // ROS_INFO_STREAM("k:"<<motor1_.getK()<<","<<"T:"<<motor1_.getT()<<","<<"omega:"<<motor1_.getOmega());

  // check direction of the rotors' velocity
  for (unsigned int i = 0; i < joint_cnt_; i++)
  {
    if (bidirectional_)
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
      {
        rotor_vel_[i] = 0;
        ROS_WARN("Invalid negative rotor_cmd[%d] for unidirectional propeller! Set to 0", i);
      }
      else
      {
        rotor_vel_[i] = rotor_vel_raw_[i];
      }

      dir_vel_actual_[i] = dir_spin_default_[i];
      dir_thrust_[i] = 1;
    }
  }

  // clamp rotors angular velocity
  for (unsigned int i = 0; i < joint_cnt_; i++)
    rotor_vel_[i] = clamp(rotor_vel_[i], vel_min_, vel_max_);

  if (use_ground_effect_)
  {
    calcGroundEffectCoeff();
  }
  else
  {
    ground_effect_coeff_ = 1.0;
  }

  // why not  this->model_->WorldLinearVel().X();
  uav_vx_ = this->model_->RelativeLinearVel().X();
  uav_vy_ = this->model_->RelativeLinearVel().Y();
  uav_vz_ = this->model_->RelativeLinearVel().Z();
  air_global_vx_ = wind_vx_ - uav_vx_;
  air_global_vy_ = wind_vy_ - uav_vy_;
  air_global_vz_ = wind_vz_ - uav_vz_;
  Eigen::Vector3d V_airflow(air_global_vx_, air_global_vy_, air_global_vz_);

  for (unsigned int i = 0; i < joint_cnt_; i++)
  {
    if (add_wrench_to_drone_)
    {
      if (use_simple_aerodynamic_)
      {
        ctrl_link_ptr_[i]->AddRelativeForce(ignition::math::Vector3<double>(
            0, 0, dir_thrust_[i] * k_simple_aero_ * pow(rotor_vel_[i], 2) / ground_effect_coeff_));
        ctrl_link_ptr_[i]->AddRelativeTorque(
            ignition::math::Vector3<double>(0, 0, -dir_vel_actual_[i] * b_simple_aero_ * pow(rotor_vel_[i], 2)));
      }
      else
      {
        Eigen::Quaterniond q(ctrl_link_ptr_[i]->WorldPose().Rot().W(), ctrl_link_ptr_[i]->WorldPose().Rot().X(),
                             ctrl_link_ptr_[i]->WorldPose().Rot().Y(), ctrl_link_ptr_[i]->WorldPose().Rot().Z());
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
          v_i = -v_z / 2.0 + sqrt(pow((v_z / 2.0), 2) + pow(ind_vel_hover_, 2));
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
        ctrl_link_ptr_[i]->AddRelativeForce(ignition::math::Vector3<double>(aero_force_[i].x(), aero_force_[i].y(),
                                                                            aero_force_[i].z() / ground_effect_coeff_));
        ctrl_link_ptr_[i]->AddRelativeTorque(
            ignition::math::Vector3<double>(aero_torque_[i].x(), aero_torque_[i].y(), aero_torque_[i].z()));
      }
    }

  }  // end for loop

  if (add_dist_)
  {
    this->link0_->AddForce(dist_force_);
    this->link0_->AddTorque(dist_torque_);

    // std::cout<< "force: " << dist_force_.X() << "," << dist_force_.Y() << "," << dist_force_.Z() << "\n";
    // std::cout<< "torque: " << dist_torque_.X() << "," << dist_torque_.Y() << "," << dist_torque_.Z() << "\n";
  }

  // print to file
  if (write_data_2_file_)
  {
    streamDataToFile();
  }

  this->setRotorVelocity();

  // save data for csv output
  for (unsigned int i = 0; i < joint_cnt_; i++)
  {
    test_data_[i] = rotor_vel_[i] * dir_vel_actual_[i];
    test_data_[i + 6] = aero_torque_[i].z();
  }

  // test_data_[0] = CT1;
  // test_data_[1] = CT2;
  // test_data_[2] = CT3;
  // test_data_[3] = CT4;
  // test_data_[4] = CT5;
  // test_data_[5] = CT6;
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

    if (WRITE_CSV_FILE_)
    {
      // if (rotor_vel_[0] < 2500 && rotor_vel_[0] >= 100)
      {
        std::ofstream result_file(RESULT_CSV_PATH_, std::ios::app);
        result_file.setf(std::ios::fixed, std::ios::floatfield);
        result_file.precision(5);
        result_file << this->model_->GetWorld()->SimTime().Double() << "," << test_data_[0] << "," << test_data_[1]
                    << "," << test_data_[2] << "," << test_data_[3] << "," << test_data_[4] << "," << test_data_[5]
                    << "," << test_data_[6] << "," << test_data_[7] << "," << test_data_[8] << "," << test_data_[9]
                    << "," << test_data_[10] << "," << test_data_[11] << "," << std::endl;
        result_file.close();
      }
    }
  }
  else
  {
    publish_cnt++;
  }
}

void PropulsionPlugin::onRosWindMsg(const geometry_msgs::Vector3ConstPtr &_wind_msg)
{
  wind_vx_ = _wind_msg->x;
  wind_vy_ = _wind_msg->y;
  wind_vz_ = _wind_msg->z;
  // ROS_INFO("add wind disturbance");
}

void PropulsionPlugin::onRosDistMsg(const geometry_msgs::WrenchConstPtr &_dist_msg)
{
  dist_force_ = ignition::math::Vector3<double>(_dist_msg->force.x, _dist_msg->force.y, _dist_msg->force.z);
  dist_torque_ = ignition::math::Vector3<double>(_dist_msg->torque.x, _dist_msg->torque.y, _dist_msg->torque.z);
}

void PropulsionPlugin::onControlMsg(const flypulator_common_msgs::RotorVelStampedConstPtr &_msg)
{
  if (_msg->velocity.size() != joint_cnt_)
  {
    ROS_ERROR_STREAM("Dimention mismatch rotor_cmd and joint number!");
    return;
  }

  for (unsigned int i = 0; i < joint_cnt_; i++)
    rotor_vel_cmd_[i] = _msg->velocity[i];

  // ROS_INFO_STREAM("aero:"<<_msg->velocity[0]<<","<<_msg->velocity[1]<<","<<_msg->velocity[2]<<","<<_msg->velocity[3]<<","<<_msg->velocity[4]<<","<<_msg->velocity[5]);
  // ROS_INFO_STREAM("aero:"<<rotor_vel_[0]<<","<<rotor_vel_[1]<<","<<rotor_vel_[2]<<","<<rotor_vel_[3]<<","<<rotor_vel_[4]<<","<<rotor_vel_[5]);
}

void PropulsionPlugin::queueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode_->ok())
  {
    this->rosQueue_.callAvailable(ros::WallDuration(timeout));
  }
}

template <typename T>
int PropulsionPlugin::Sgn(T &_num)
{
  return (T(0) < _num) - (_num < T(0));
}

double PropulsionPlugin::clamp(double _x, double _lower, double _upper)
{
  return std::min(_upper, std::max(_x, _lower));
}

void PropulsionPlugin::setRotorVelocity()
{
  for (unsigned int i = 0; i < joint_cnt_; i++)
  {
    this->ctrl_joint_ptr_[i]->SetParam("vel", 0, rotor_vel_[i] * dir_vel_actual_[i]);
  }
  // ROS_DEBUG_STREAM("aero:"<<rotor_vel_[0]<<","<<rotor_vel_[1]<<","<<rotor_vel_[2]<<","<<rotor_vel_[3]<<","<<rotor_vel_[4]<<","<<rotor_vel_[5]);
  // ROS_DEBUG_STREAM("aero plugin: SetVelocity()!");
}

void PropulsionPlugin::calcGroundEffectCoeff()
{
  // altitude/height of the drone to the gorund
  double drone_height = this->link0_->WorldPose().Pos().Z();

  // TODO: only approximate ground effect, neglect the tilting angle of the drone and propeller
  // calculate ground effect coefficient, T_eff = T_nom/ground_effect_coeff_
  if (drone_height > 0.35 * aero_param_["R"])                                    // R = blade radius
    ground_effect_coeff_ = 1.0 - pow(aero_param_["R"] / (4 * drone_height), 2);  // min.~0.5
  else
    ground_effect_coeff_ = 0.5;
}

void PropulsionPlugin::readParamsFromServer()
{
  // try to read joint names from ros parameter server
  if (ros::param::get("urdf/controller_joint_names", joint_names_))
  {
    for (auto i : joint_names_)
    {
      ROS_DEBUG_STREAM("propulsion_plugin: loaded control joint names : " << i);
    }
    ROS_INFO("propulsion_plugin: joint names loaded.");
    joint_cnt_ = (unsigned int)(joint_names_.size());
  }
  else
  {
    ROS_ERROR("Load joint names from parameter server failed!");
  }
  // ROS_DEBUG_STREAM("control joint number: " << joint_cnt_);

  // try to read links names from ros parameter server
  if (ros::param::get("urdf/controller_link_names", link_names_))
  {
    for (auto i : link_names_)
    {
      ROS_DEBUG_STREAM("propulsion_plugin: loaded control link names : " << i);
    }
    ROS_INFO("propulsion_plugin: link names loaded.");
  }
  else
  {
    ROS_ERROR("Load link names from parameter server failed!");
  }

  // try to read model parameter from ros parameter server
  if (ros::param::get("/aero_param", aero_param_) && ros::param::get("uav/rotor_vel_min", vel_min_))
  {
    ros::param::get("uav/rotor_vel_max", vel_max_);
    ros::param::get("uav/k", k_simple_aero_);
    ros::param::get("uav/b", b_simple_aero_);
    ros::param::get("uav/alpha", tilting_angle_);
    ros::param::get("urdf/bidirectional", bidirectional_);

    // convert actuator boundaries from rpm to rad/s
    vel_min_ *= M_PI / 30;
    vel_max_ *= M_PI / 30;

    ROS_INFO("propulsion_plugin: drone parameters loaded.");

    ROS_DEBUG_STREAM("propulsion_plugin: uav/rotor_vel_min: " << vel_min_ << " rad/s");
    ROS_DEBUG_STREAM("propulsion_plugin: uav/rotor_vel_max: " << vel_max_ << " rad/s");
    ROS_DEBUG_STREAM("propulsion_plugin: uav/k: " << k_simple_aero_ << " N/(rad/s)^2");
    ROS_DEBUG_STREAM("propulsion_plugin: uav/b: " << b_simple_aero_ << " N/(rad/s)^2");
    ROS_DEBUG_STREAM("propulsion_plugin: uav/alpha: " << tilting_angle_ << " degree");
    ROS_DEBUG_STREAM("propulsion_plugin: uav/bidirectional: " << bidirectional_);
  }
  else
  {
    ROS_WARN("No model parameters availiable, use default values...");
    // default values in constructor
    ROS_INFO_STREAM("propulsion_plugin: uav/rotor_vel_min: " << vel_min_ << " rad/s");
    ROS_INFO_STREAM("propulsion_plugin: uav/rotor_vel_max: " << vel_max_ << " rad/s");
    ROS_INFO_STREAM("propulsion_plugin: uav/k: " << k_simple_aero_ << " N/(rad/s)^2");
    ROS_INFO_STREAM("propulsion_plugin: uav/b: " << b_simple_aero_ << " N/(rad/s)^2");
    ROS_INFO_STREAM("propulsion_plugin: uav/alpha: " << tilting_angle_ << " degree");
    ROS_INFO_STREAM("propulsion_plugin: uav/bidirectional: " << bidirectional_);
  }

  // try to read simulation configuration from ros parameter server
  if (ros::param::get("sim/write_data_2_file", write_data_2_file_))
  {
    ros::param::get("sim/WRITE_CSV_FILE", WRITE_CSV_FILE_);
    ros::param::get("sim/RESULT_CSV_PATH", RESULT_CSV_PATH_);
    ros::param::get("sim/file_path", file_path_);
    ros::param::get("sim/add_wrench_to_drone", add_wrench_to_drone_);
    ros::param::get("sim/use_ground_effect", use_ground_effect_);
    ros::param::get("sim/use_motor_dynamic", use_motor_dynamic_);
    ros::param::get("sim/use_simple_aerodynamic", use_simple_aerodynamic_);
    ros::param::get("sim/add_dist", add_dist_);

    ROS_INFO("propulsion_plugin: simulation setting loaded.");

    ROS_DEBUG_STREAM("propulsion_plugin: sim/add_wrench_to_drone:" << add_wrench_to_drone_);
    ROS_DEBUG_STREAM("propulsion_plugin: sim/use_ground_effect:" << use_ground_effect_);
    ROS_DEBUG_STREAM("propulsion_plugin: sim/use_motor_dynamic:" << use_motor_dynamic_);
    ROS_DEBUG_STREAM("propulsion_plugin: sim/use_simple_aerodynamic:" << use_simple_aerodynamic_);
    ROS_DEBUG_STREAM("propulsion_plugin: sim/add_dist:" << add_dist_);
  }
  else
  {
    ROS_WARN_STREAM("Load simulation setting failed, use default values...");
    ROS_INFO_STREAM("propulsion_plugin: sim/add_wrench_to_drone:" << add_wrench_to_drone_);
    ROS_INFO_STREAM("propulsion_plugin: sim/use_ground_effect:" << use_ground_effect_);
    ROS_INFO_STREAM("propulsion_plugin: sim/use_motor_dynamic:" << use_motor_dynamic_);
    ROS_INFO_STREAM("propulsion_plugin: sim/use_simple_aerodynamic:" << use_simple_aerodynamic_);
    ROS_INFO_STREAM("propulsion_plugin: sim/add_dist:" << add_dist_);
  }
}

void PropulsionPlugin::tfPublisher()
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

void PropulsionPlugin::jointStatePubliher()
{
  // publish joint state
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.name.resize(joint_cnt_);
  joint_state_msg.position.resize(joint_cnt_);
  joint_state_msg.velocity.resize(joint_cnt_);
  joint_state_msg.effort.resize(joint_cnt_);
  joint_state_msg.header.stamp = ros::Time::now();
  joint_state_msg.header.frame_id = "base_link";
  for (unsigned int i = 0; i < joint_cnt_; i++)
  {
    joint_state_msg.name[i] = this->ctrl_joint_ptr_[i]->GetName();
    joint_state_msg.position[i] = this->ctrl_joint_ptr_[i]->Position(0);
    joint_state_msg.velocity[i] = this->ctrl_joint_ptr_[i]->GetVelocity(0);
  }

  pub_joint_state_.publish(joint_state_msg);
}

void PropulsionPlugin::wrenchPublisher()
{
  geometry_msgs::WrenchStamped wrench_msg_tmp;
  wrench_msg_tmp.header.stamp = ros::Time::now();

  for (int i = 0; i < 6; i++)
  {
    // TODO: check link name, publish another variable for simple aero model!!!
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

void PropulsionPlugin::streamDataToFile()
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

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(PropulsionPlugin)
}  // namespace gazebo

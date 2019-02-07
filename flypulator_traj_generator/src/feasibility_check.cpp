/*
 * @author Nils Dunkelberg
 */

#include "feasibility_check.h"

FeasibilityCheck::FeasibilityCheck() : actuator_simulation_(new ActuatorSimulation())
{
  // load required simulation parameters
  ros::param::param<float>("/trajectory/newton_stepsize", newt_stepsize_, 0.01f);
  ros::param::param<double>("/trajectory/newton_epsilon", newt_epsilon_, 10);
  ros::param::param<double>("/trajectory/boundary_buffer", rotvel_buffer_, 150);

  // actuator velocity boundaries
  ros::param::param<double>("/uav/rotor_vel_max", upper_vel_limit_, 5700);
  ros::param::param<double>("/uav/rotor_vel_min", lower_vel_limit_, 0);

  // store actuator boundaries as rad²/s² as well
  upper_vel_limit_squ_ = pow(upper_vel_limit_ * M_PI / 30, 2);
  lower_vel_limit_squ_ = pow(lower_vel_limit_ * M_PI / 30, 2);

  // register ros service client for polynomial trajectory generation service+
  polynomial_traj_client_ = nh_.serviceClient<polynomial_trajectory>("polynomial_trajectory");
}

void FeasibilityCheck::updateSimulationParameters(flypulator_traj_generator::traj_parameterConfig &config)
{
  // update parameters of actuator simulation through dynamic reconfigure
  actuator_simulation_->updateDroneParameters(config);

  // updating feasibility parameters through dynamic reconfigure
  rotvel_buffer_ = (double)config.boundary_buffer;

  upper_vel_limit_ = (float)config.rotor_vel_max;
  lower_vel_limit_ = (float)config.rotor_vel_min;

  // store actuator boundaries as rad²/s² as well
  upper_vel_limit_squ_ = pow(upper_vel_limit_ * M_PI / 30, 2);
  lower_vel_limit_squ_ = pow(lower_vel_limit_ * M_PI / 30, 2);
}

bool FeasibilityCheck::makeFeasible(Eigen::Vector6f &start_pose, Eigen::Vector6f &target_pose, double &duration)
{
  // retrieve squared rotor velocities for start and target pose
  Eigen::Vector6f rot_vel_squ_start = actuator_simulation_->getSteadyStateRotorVelocities(start_pose);
  Eigen::Vector6f rot_vel_squ_target = actuator_simulation_->getSteadyStateRotorVelocities(target_pose);

  // check start pose feasibility. If not feasible, user has to modify it.
  if (!isFeasible(rot_vel_squ_start))
  {
    ROS_INFO("Start pose not feasible. Please adjust!");
    return false;
  }

  // check target pose feasibility. If not feasible, calculate closest feasible alternative.
  if (!isFeasible(rot_vel_squ_target))
  {
    ROS_INFO("Target pose not feasible. Retrieving feasible alternative...");
    retrieveFeasibleEndpose(target_pose, rot_vel_squ_target);
  }

  // check if trajectory is feasible
  callTrajectoryGenerator(start_pose, target_pose, duration, false);
  if (cur_traj_data_->rot_vel_squ_max_ > upper_vel_limit_squ_ ||
      cur_traj_data_->rot_vel_squ_min_ < lower_vel_limit_squ_)
  {
    // trajectory not feasible -> calculate feasible duration
    ROS_INFO("Trajectory duration not feasible. Retrieving feasible alternative...");
    retrieveFeasibleDuration(duration);
  }

  ROS_INFO("The feasible duration is %f ", duration);
  return true;
}

bool FeasibilityCheck::isFeasible(Eigen::Vector6f rotor_velocities)
{
  // retrieve maximum and minimum rotor squared velocities
  double rot_vel_min = rotor_velocities.minCoeff();
  double rot_vel_max = rotor_velocities.maxCoeff();

  // feasibility check
  return rot_vel_min >= lower_vel_limit_squ_ && rot_vel_max <= upper_vel_limit_squ_;
}

void FeasibilityCheck::retrieveFeasibleEndpose(Eigen::Vector6f &target_pose, Eigen::Vector6f initial_rotor_velocities)
{
  //######### setup ###################
  // retrieve the squared rotor velocities needed for horizontal hovering (zero orientation)
  Eigen::Vector6f steady_state_rotvels = actuator_simulation_->getSteadyStateRotorVelocities(Eigen::Vector6f::Zero());
  double steady_state_rotvel = steady_state_rotvels[0];

  // set updated actuator limits (decreased/increased by buffer velocity) and converted to rad²/s²
  double ulimit = pow((upper_vel_limit_ - rotvel_buffer_) * M_PI / 30, 2);
  double llimit = pow((lower_vel_limit_ + rotvel_buffer_) * M_PI / 30, 2);

  // since yaw angle has no influence on euler angles, set it to zero
  Eigen::Vector6f target_pose_tmp = target_pose;
  target_pose_tmp[5] = 0;

  // retrieve euler parameters for target pose
  Eigen::Vector3f euler_axis;
  double euler_angle;
  actuator_simulation_->poseToEulerParams(target_pose_tmp, euler_axis, euler_angle);

  // retrieve rotor indices with the maximum and minimum velocities#
  int i_max;
  int i_min;
  initial_rotor_velocities.maxCoeff(&i_max);
  initial_rotor_velocities.minCoeff(&i_min);

  //######## newton approach ############
  double feasible_upper_euler_angle = findFeasbileEulerAngle(steady_state_rotvel, ulimit, euler_axis, i_max);
  double feasible_lower_euler_angle = findFeasbileEulerAngle(steady_state_rotvel, llimit, euler_axis, i_min);

  double feasible_euler_angle = std::min(feasible_upper_euler_angle, feasible_lower_euler_angle);

  //######### result #####################
  // apply new target pose orientation to the original target pose (only roll and pitch angle considered)
  Eigen::Vector3f rpy_angles = actuator_simulation_->eulerParamsToYPR(euler_axis, feasible_euler_angle);
  target_pose.segment(3, 2) =
      rpy_angles.head(2);  // segment (starting at coefficient 3, containing 2 successive entrys (roll and pitch))
}

double FeasibilityCheck::findFeasbileEulerAngle(double rotvel_init, double rotvel_limit, Eigen::Vector3f euler_axis,
                                                int index)
{
  double rotvel_k = rotvel_init;  // holds the resulting rotor velocities after each step of the newton approach
  double theta_k = 0;             // holds the corresponding euler angle of each step which will be optimized
  double newt_epsilon = pow(newt_epsilon_ * M_PI / 10, 2);  // terminating condition per rad²/s²

  // loop until terminating condition is met
  while (abs(rotvel_limit - rotvel_k) > newt_epsilon)
  {
    // derivative at current euler angle
    double theta_tmp = theta_k + newt_stepsize_;
    Eigen::Vector6f rotvel_tmp = actuator_simulation_->getSteadyStateRotorVelocities(euler_axis, theta_tmp);
    double d_rotvel = (rotvel_tmp[index] - rotvel_k) / newt_stepsize_;

    // calculate euler angle and rotor velocity at intersection with actuator limit (corresponds to next step k+1)
    theta_k += (rotvel_limit - rotvel_k) / d_rotvel;
    Eigen::Vector6f rotvels_k = actuator_simulation_->getSteadyStateRotorVelocities(euler_axis, theta_k);
    rotvel_k = rotvels_k[index];
  }
  return theta_k;
}

void FeasibilityCheck::retrieveFeasibleDuration(double &duration)
{
  // the actuator limit, which gets exceeded most and the indices of the corresponding rotor velocities
  double critical_limit;
  indices critical_indices;
  double critical_rotor_velocity;

  // the factor by which the new time gets dilated
  double dilation_factor = 1.001;

  // maximum amount of iterations
  while (!withinActuatorLimits(critical_limit, critical_indices, critical_rotor_velocity))
  {
    // retrieve the orientation at the time the critical rotor velocity occur
    double crit_euler_angle = cur_traj_data_->euler_angles_[critical_indices.second];
    Eigen::Quaternionf q_krit = actuator_simulation_->eulerParamsToQuat(cur_traj_data_->start_pose_.tail(3),
                                                                        cur_traj_data_->euler_axis_, crit_euler_angle);

    // retrieve gravitational component of the critical rotor velocity (W_inv(crit, 3) * mg)
    double grav_rotvel_component =
        actuator_simulation_->getGravitationalVelocityComponent(q_krit, critical_indices.first);

    duration *= dilation_factor *
                sqrt((critical_rotor_velocity - grav_rotvel_component) / (critical_limit - grav_rotvel_component));

    callTrajectoryGenerator(cur_traj_data_->start_pose_, cur_traj_data_->target_pose_, duration, false);
  }
}

bool FeasibilityCheck::withinActuatorLimits(double &critical_limit, indices &critical_indices,
                                            double &critical_rotor_velocity)
{
  // get amount (per rad²/s²) by which the actuator boundaries get exceeded
  double upper_exceeding = cur_traj_data_->rot_vel_squ_max_ - upper_vel_limit_squ_;
  double lower_exceeding = lower_vel_limit_squ_ - cur_traj_data_->rot_vel_squ_min_;

  // if exceeding amounts are negative, all rotor velocities are within the feasible range -> trajectory is feasible
  if (upper_exceeding <= 0 && lower_exceeding <= 0)
  {
    return true;
  }
  else if (upper_exceeding > lower_exceeding)
  {
    critical_limit = upper_vel_limit_squ_;
    critical_indices = cur_traj_data_->i_max_;
    critical_rotor_velocity = cur_traj_data_->rot_vel_squ_max_;
  }
  else
  {
    critical_limit = lower_vel_limit_squ_;
    critical_indices = cur_traj_data_->i_min_;
    critical_rotor_velocity = cur_traj_data_->rot_vel_squ_min_;
  }

  // trajectory not feasible otherwise
  return false;
}

void FeasibilityCheck::callTrajectoryGenerator(Eigen::Vector6f start_pose, Eigen::Vector6f target_pose, double duration,
                                               bool start_tracking)
{
  // create service
  polynomial_trajectory pt_srv;

  // fill service parameters
  pt_srv.request.p_start.x = start_pose[0];
  pt_srv.request.p_start.y = start_pose[1];
  pt_srv.request.p_start.z = start_pose[2];
  pt_srv.request.rpy_start.x = start_pose[3];
  pt_srv.request.rpy_start.y = start_pose[4];
  pt_srv.request.rpy_start.z = start_pose[5];

  pt_srv.request.p_end.x = target_pose[0];
  pt_srv.request.p_end.y = target_pose[1];
  pt_srv.request.p_end.z = target_pose[2];
  pt_srv.request.rpy_end.x = target_pose[3];
  pt_srv.request.rpy_end.y = target_pose[4];
  pt_srv.request.rpy_end.z = target_pose[5];

  pt_srv.request.duration = duration;

  pt_srv.request.start_tracking = start_tracking;

  if (!polynomial_traj_client_.call(pt_srv))
  {
    ROS_ERROR("[flypulator_traj_generator] Failed to call polynomial trajectory service.");
    return;
  }

  // Create Vector to store rotor velocity evolution and its timestamps (preparation for later plotting)
  size_t size = pt_srv.response.p_acc.size();
  trajectory::rotor_velocities_rpm rotor_velocities_rpm(6, QVector<double>(size));
  QVector<double> time_stamps = QVector<double>::fromStdVector(pt_srv.response.time_stamps);

  // convert eulerAxis from geometry message to Vector3f
  Eigen::Vector3f euler_axis(pt_srv.response.euler_axis.x, pt_srv.response.euler_axis.y, pt_srv.response.euler_axis.z);

  // initialize trajectory data struct and fill it
  delete cur_traj_data_;
  cur_traj_data_ =
      new trajectory::TrajectoryData(start_pose, target_pose, pt_srv.response.p_acc, pt_srv.response.euler_angle_acc, pt_srv.response.euler_angle_vel,
                                     pt_srv.response.euler_angle, euler_axis, rotor_velocities_rpm, time_stamps);

  // simulate the rotor velocities for the current trajectory
  actuator_simulation_->simulateActuatorVelocities(*cur_traj_data_);

  // perform feasibility check
  if (cur_traj_data_->rot_vel_squ_max_ > upper_vel_limit_squ_ ||
      cur_traj_data_->rot_vel_squ_min_ < lower_vel_limit_squ_)
  {
    cur_traj_data_->feasible_ = false;
  }
}

void FeasibilityCheck::getPlotData(trajectory::rotor_velocities_rpm &rotor_velocities_rpm, QVector<double> &time_stamps,
                                   bool &feasible)
{
  // write the references with the current trajectory data
  rotor_velocities_rpm = cur_traj_data_->rot_vel_rpm_;
  time_stamps = cur_traj_data_->time_stamps_;
  feasible = cur_traj_data_->feasible_;
}

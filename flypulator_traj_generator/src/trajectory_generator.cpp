#include "flypulator_traj_generator/trajectory_generator.h"
#include <iostream>

// create Trajectory and send it periodically
bool TrajectoryGenerator::createAndSendTrajectory(
    const geometry_msgs::Vector3& p_start, const geometry_msgs::Vector3& p_end, const geometry_msgs::Vector3& rpy_start,
    const geometry_msgs::Vector3& rpy_end, const float duration, const bool start_tracking,
    const trajectory_types::Type traj_type, trajectory::pos_accelerations& pos_accs,
    trajectory::euler_angle_accelerations& euler_angle_accs, trajectory::euler_angle_velocities& euler_angle_vels,
    trajectory::euler_angles& euler_angles, geometry_msgs::Vector3& eulerAxis, std::vector<double>& time_stamps)
{
  // save input values in 4D array (px, py, pz, euler_angle)
  float pose_start[4];
  float pose_end[4];

  Eigen::EulerParams eulerParams;
  Eigen::Quaternionf q_start;
  // assign positional start and target components to 4d arrays (pose_start and pose_end). Calculate euler parameters
  // and retrieve them via reference
  extractStartAndTargetPose(p_start, p_end, rpy_start, rpy_end, pose_start, pose_end, eulerParams, q_start);

  float pose_current[4];  // array for 4d poses (three positional components, one euler angle)
  float vel_current[4];   // array for 4D velocities
  float acc_current[4];   // array for 4D accelerations
  // create math objects for quaternion and omega calculations using Eigen library
  Eigen::Quaternionf q_current;
  Eigen::Vector3f omega_current;
  Eigen::Vector3f omega_dot_current;

  // assign euler axis to service response
  eulerAxis.x = eulerParams.axis[0];
  eulerAxis.y = eulerParams.axis[1];
  eulerAxis.z = eulerParams.axis[2];

  float a[4][6];  // polynomial coefficients for trajectory, first dimension: pose component, second dimension:
                  // coefficient (can be more than 6 for higher order polynoms)

  // calculate polynomial coeffiencts for linear and polynomial trajectory
  switch (traj_type)
  {
    case trajectory_types::Linear:
      // calculate constant linear velocity and acceleration (=0)
      for (int dim = 0; dim < 4; dim++)
      {
        a[dim][0] = pose_start[dim];
        a[dim][1] = (pose_end[dim] - pose_start[dim]) / duration;
        a[dim][2] = 0;
        a[dim][3] = 0;
        a[dim][4] = 0;
        a[dim][5] = 0;
      }
      // ROS_INFO("Start linear trajectory..");
      break;

    case trajectory_types::Polynomial:
      // calculate polynomial coefficients
      for (int dim = 0; dim < 4; dim++)
      {
        a[dim][0] = pose_start[dim];
        a[dim][1] = 0.0f;
        a[dim][2] = 0.0f;
        a[dim][3] = 10 * (pose_end[dim] - pose_start[dim]) / pow(duration, 3);
        a[dim][4] = -15 * (pose_end[dim] - pose_start[dim]) / pow(duration, 4);
        a[dim][5] = 6 * (pose_end[dim] - pose_start[dim]) / pow(duration, 5);
      }
      // ROS_INFO("Start polynomial trajectory..");
      break;

    default:
      ROS_ERROR("Polynom type not well defined! Must follow enumeration");
  }

  // retrieve simulation step size from parameter server
  float sim_dt;
  if (!ros::param::get("/trajectory/simulation_step_size", sim_dt))
  {
    ROS_DEBUG("[TrajectoryGenService] Step size for simulation not specified: Using default of 10ms instead.");
    sim_dt = 0.01;
  }

  // retrieve path poses and their orientation for visualization purpose
  geometry_msgs::PoseArray path;
  std::vector<geometry_msgs::Pose> poses;

  for (double t = 0; t <= duration; t += sim_dt)
  {
    geometry_msgs::Vector3 pos_acc;
    float euler_angle_acc;
    float euler_angle_vel;
    float euler_angle;

    pos_acc.x = evaluateAcceleration(a[0][2], a[0][3], a[0][4], a[0][5], t);
    pos_acc.y = evaluateAcceleration(a[1][2], a[1][3], a[1][4], a[1][5], t);
    pos_acc.z = evaluateAcceleration(a[2][2], a[2][3], a[2][4], a[2][5], t);

    euler_angle_acc = evaluateAcceleration(a[3][2], a[3][3], a[3][4], a[3][5], t);
    euler_angle_vel = evaluateVelocity(a[3][1], a[3][2], a[3][3], a[3][4], a[3][5], t);
    euler_angle = evaluatePosition(a[3][0], a[3][1], a[3][2], a[3][3], a[3][4], a[3][5], t);

    pos_accs.push_back(pos_acc);
    euler_angle_accs.push_back(euler_angle_acc);
    euler_angle_vels.push_back(euler_angle_vel);
    euler_angles.push_back(euler_angle);
    time_stamps.push_back(t);

    geometry_msgs::Pose pose;
    pose.position.x = evaluatePosition(a[0][0], a[0][1], a[0][2], a[0][3], a[0][4], a[0][5], t);
    pose.position.y = evaluatePosition(a[1][0], a[1][1], a[1][2], a[1][3], a[1][4], a[1][5], t);
    pose.position.z = evaluatePosition(a[2][0], a[2][1], a[2][2], a[2][3], a[2][4], a[2][5], t);

    // Each arrow of the pose array visualitation is supposed to point to the direction of the z-Axis, therefore a 90°
    // rotation against the y-axis is performed after all.
    Eigen::Quaternionf q_rot_y;
    q_rot_y = Eigen::AngleAxisf(-0.5 * M_PI, Eigen::Vector3f::UnitY());

    // retrieve orientation of {B} w.r.t. {A} via euler angle
    float cur_euler_angle = evaluatePosition(a[3][0], a[3][1], a[3][2], a[3][3], a[3][4], a[3][5], t);
    // convert to quaternion
    Eigen::Quaternionf q_AB;
    eulerParamsToQuat(eulerParams.axis, cur_euler_angle, q_AB);

    // resulting orientaiton
    Eigen::Quaternionf q_res = q_start * q_AB * q_rot_y;

    pose.orientation.x = q_res.x();
    pose.orientation.y = q_res.y();
    pose.orientation.z = q_res.z();
    pose.orientation.w = q_res.w();

    poses.push_back(pose);
  }

  path.poses = poses;
  path.header.frame_id = "world";
  traj_visualization_pub_.publish(path);

  // only start the trajectory tracking, if demanded
  if (!start_tracking)
  {
    return true;
  }

  const ros::Time t_start = ros::Time::now();  // Take ros time - sync with "use_sim_time" parameter over /clock topic
  const ros::Duration trajDuration(duration);

  ros::Time t(t_start.toSec());  // running time variable

  ROS_DEBUG("Start Time: %f", t.toSec());

  // take update rate from parameter file
  float update_rate;
  if (ros::param::get("/trajectory/update_rate", update_rate))
  {
    ROS_DEBUG("Update Rate = %f from parameter server", update_rate);
  }
  else
  {
    ROS_DEBUG("Update Rate = 10, default value (/trajectory/update_rate not available on parameter server)");
    update_rate = 10.0f;
  }

  ros::Rate r(update_rate);

  // start continous message publishing
  while (t <= t_start + trajDuration)
  {
    // calculate trajectory for position, velocity, acceleration, and euler angle trajectory and its derivatives (not
    // equal to omega and omega_dot!!)
    if (traj_type == trajectory_types::Linear || traj_type == trajectory_types::Polynomial)
    {
      // linear trajectory is also a polynomial trajectory with different coeffiencts (set before)
      float dt = (float)(t.toSec() - t_start.toSec());
      for (int dim = 0; dim < 4; dim++)
      {
        pose_current[dim] = evaluatePosition(a[dim][0], a[dim][1], a[dim][2], a[dim][3], a[dim][4], a[dim][5], dt);
        vel_current[dim] = evaluateVelocity(a[dim][1], a[dim][2], a[dim][3], a[dim][4], a[dim][5], dt);
        acc_current[dim] = evaluateAcceleration(a[dim][2], a[dim][3], a[dim][4], a[dim][5], dt);
      }
    }
    // add other trajectory types here

    // calculate rotational trajectory from euler angle trajectory
    // calculate quaternion from euler parameters
    Eigen::Quaternionf q_AB;
    float theta = pose_current[3];
    eulerParamsToQuat(eulerParams.axis, theta, q_AB);
    q_current = q_start * q_AB;

    // calculate omega from euler angles and their derivative
    float theta_dot = vel_current[3];
    float theta_ddot = acc_current[3];
    angularVelocityFromEulerParams(q_start, eulerParams.axis, theta, theta_dot, theta_ddot, omega_current,
                                   omega_dot_current);

    // Concatenate informations to trajectory message
    trajectory_msgs::MultiDOFJointTrajectoryPoint msg =
        generateTrajectoryMessage(pose_current, vel_current, acc_current, q_current, omega_current, omega_dot_current,
                                  ros::Duration(t - t_start));
    this->trajectory_publisher_.publish(msg);  // publish message to topic /trajectory

    ros::spinOnce();       // not necessary (?) but good measure
    r.sleep();             // keep frequency, so sleep until next timestep
    t = ros::Time::now();  // update time
    ROS_DEBUG("Current Time: %f", t.toSec());
  }

  // send final message with zero velocities and accelerations
  for (int i = 0; i < 6; i++)
  {
    vel_current[i] = 0;
    acc_current[i] = 0;
  }
  Eigen::Vector3f zeros(0, 0, 0);
  trajectory_msgs::MultiDOFJointTrajectoryPoint msg = generateTrajectoryMessage(
      pose_current, vel_current, acc_current, q_current, zeros, zeros, ros::Duration(t - t_start));
  this->trajectory_publisher_.publish(msg);  // publish message to topic /trajectory

  ROS_INFO("trajectory finished!");

  return true;
}

void TrajectoryGenerator::extractStartAndTargetPose(const geometry_msgs::Vector3& p_start,
                                                    const geometry_msgs::Vector3& p_end,
                                                    const geometry_msgs::Vector3& rpy_start,
                                                    const geometry_msgs::Vector3& rpy_end, float pose_start[],
                                                    float pose_end[], Eigen::EulerParams& eulerParams,
                                                    Eigen::Quaternionf& q_start)
{
  // assign positional parts
  pose_start[0] = p_start.x;
  pose_start[1] = p_start.y;
  pose_start[2] = p_start.z;

  pose_end[0] = p_end.x;
  pose_end[1] = p_end.y;
  pose_end[2] = p_end.z;

  // extract start and target rpy angles (per rad!)
  float roll_start = rpy_start.x * M_PI / 180;
  float pitch_start = rpy_start.y * M_PI / 180;
  float yaw_start = rpy_start.z * M_PI / 180;

  float roll_end = rpy_end.x * M_PI / 180;
  float pitch_end = rpy_end.y * M_PI / 180;
  float yaw_end = rpy_end.z * M_PI / 180;

  // ####### calculation of euler parameters ###########
  // rotation matrix from start and end pose
  Eigen::Matrix3f rot_mat_start;
  rpyToRotMat(roll_start, pitch_start, yaw_start, rot_mat_start);
  Eigen::Matrix3f rot_mat_end;
  rpyToRotMat(roll_end, pitch_end, yaw_end, rot_mat_end);

  // retrieve difference rotation
  Eigen::Matrix3f rot_mat_diff = rot_mat_start.transpose() * rot_mat_end;

  // get euler axis and angle
  calculateEulerParameters(rot_mat_diff, eulerParams);

  // assign euler angles
  pose_start[3] = 0;
  pose_end[3] = eulerParams.angle;

  // assign start orientation (auto conversion from matrix to quat)
  q_start = rot_mat_start;
}

void TrajectoryGenerator::eulerParamsToQuat(Eigen::Vector3f euler_axis, float euler_angle, Eigen::Quaternionf& q_AB)
{
  // attitude of {B} w.r.t. the start orientation
  q_AB.x() = euler_axis[0] * sin(euler_angle / 2);
  q_AB.y() = euler_axis[1] * sin(euler_angle / 2);
  q_AB.z() = euler_axis[2] * sin(euler_angle / 2);
  q_AB.w() = cos(euler_angle / 2);
}

void TrajectoryGenerator::angularVelocityFromEulerParams(Eigen::Quaternionf q_IA, Eigen::Vector3f kA, float the,
                                                         float dthe, float ddthe, Eigen::Vector3f& omeg,
                                                         Eigen::Vector3f& omeg_dot)
{
  // abbreviations
  double sthe = sin(the);
  double cthe = cos(the);

  double k_x = kA[0];
  double k_y = kA[1];
  double k_z = kA[2];

  // rotation matrix of starting orientation
  Eigen::Matrix3f R_IA = q_IA.matrix();

  // current orientation of {B} w.r.t. the starting orientation {A}
  Eigen::Matrix3f R_AB;
  R_AB(0, 0) = cthe - (k_x * k_x) * (cthe - 1.0);
  R_AB(0, 1) = -k_z * sthe - k_x * k_y * (cthe - 1.0);
  R_AB(0, 2) = k_y * sthe - k_x * k_z * (cthe - 1.0);
  R_AB(1, 0) = k_z * sthe - k_x * k_y * (cthe - 1.0);
  R_AB(1, 1) = cthe - (k_y * k_y) * (cthe - 1.0);
  R_AB(1, 2) = -k_x * sthe - k_y * k_z * (cthe - 1.0);
  R_AB(2, 0) = -k_y * sthe - k_x * k_z * (cthe - 1.0);
  R_AB(2, 1) = k_x * sthe - k_y * k_z * (cthe - 1.0);
  R_AB(2, 2) = cthe - (k_z * k_z) * (cthe - 1.0);

  // first derivative of R_AB
  Eigen::Matrix3f dR_AB;
  dR_AB(0, 0) = -dthe * sthe + dthe * (k_x * k_x) * sthe;
  dR_AB(0, 1) = -cthe * dthe * k_z + dthe * k_x * k_y * sthe;
  dR_AB(0, 2) = cthe * dthe * k_y + dthe * k_x * k_z * sthe;
  dR_AB(1, 0) = cthe * dthe * k_z + dthe * k_x * k_y * sthe;
  dR_AB(1, 1) = -dthe * sthe + dthe * (k_y * k_y) * sthe;
  dR_AB(1, 2) = -cthe * dthe * k_x + dthe * k_y * k_z * sthe;
  dR_AB(2, 0) = -cthe * dthe * k_y + dthe * k_x * k_z * sthe;
  dR_AB(2, 1) = cthe * dthe * k_x + dthe * k_y * k_z * sthe;
  dR_AB(2, 2) = -dthe * sthe + dthe * (k_z * k_z) * sthe;

  // second derivative of R_AB
  Eigen::Matrix3f ddR_AB;
  ddR_AB(0, 0) = -ddthe * sthe - cthe * (dthe * dthe) + ddthe * (k_x * k_x) * sthe + cthe * (dthe * dthe) * (k_x * k_x);
  ddR_AB(0, 1) =
      (dthe * dthe) * k_z * sthe - cthe * ddthe * k_z + ddthe * k_x * k_y * sthe + cthe * (dthe * dthe) * k_x * k_y;
  ddR_AB(0, 2) =
      -(dthe * dthe) * k_y * sthe + cthe * ddthe * k_y + ddthe * k_x * k_z * sthe + cthe * (dthe * dthe) * k_x * k_z;
  ddR_AB(1, 0) =
      -(dthe * dthe) * k_z * sthe + cthe * ddthe * k_z + ddthe * k_x * k_y * sthe + cthe * (dthe * dthe) * k_x * k_y;
  ddR_AB(1, 1) = -ddthe * sthe - cthe * (dthe * dthe) + ddthe * (k_y * k_y) * sthe + cthe * (dthe * dthe) * (k_y * k_y);
  ddR_AB(1, 2) =
      (dthe * dthe) * k_x * sthe - cthe * ddthe * k_x + ddthe * k_y * k_z * sthe + cthe * (dthe * dthe) * k_y * k_z;
  ddR_AB(2, 0) =
      (dthe * dthe) * k_y * sthe - cthe * ddthe * k_y + ddthe * k_x * k_z * sthe + cthe * (dthe * dthe) * k_x * k_z;
  ddR_AB(2, 1) =
      -(dthe * dthe) * k_x * sthe + cthe * ddthe * k_x + ddthe * k_y * k_z * sthe + cthe * (dthe * dthe) * k_y * k_z;
  ddR_AB(2, 2) = -ddthe * sthe - cthe * (dthe * dthe) + ddthe * (k_z * k_z) * sthe + cthe * (dthe * dthe) * (k_z * k_z);

  // transform to inertial frame {I}
  Eigen::Matrix3f R_IB = R_IA * R_AB;
  Eigen::Matrix3f dR_IB = R_IA * dR_AB;
  Eigen::Matrix3f ddR_IB = R_IA * ddR_AB;

  // retrieve skew matrices
  Eigen::Matrix3f omega_skew = dR_IB * R_IB.transpose();
  Eigen::Matrix3f domega_skew = ddR_IB * R_IB.transpose() + dR_IB * dR_IB.transpose();

  // retrieve angular acceleration and velocity
  omeg << omega_skew(2, 1), omega_skew(0, 2), omega_skew(0, 1);
  omeg_dot << domega_skew(2, 1), domega_skew(0, 2), domega_skew(0, 1);
}

inline float TrajectoryGenerator::evaluateAcceleration(float a2, float a3, float a4, float a5, float t)
{
  return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);
}

inline float TrajectoryGenerator::evaluateVelocity(float a1, float a2, float a3, float a4, float a5, float t)
{
  return a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4);
}

inline float TrajectoryGenerator::evaluatePosition(float a0, float a1, float a2, float a3, float a4, float a5, float t)
{
  return a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5);
}

inline void TrajectoryGenerator::calculateEulerParameters(Eigen::Matrix3f rotMat, Eigen::EulerParams& eulerParams)
{
  // get euler parameters from rotation matrix
  eulerParams.angle = acos(0.5 * (rotMat(0, 0) + rotMat(1, 1) + rotMat(2, 2) - 1));

  // todo: handle singularities properly
  if (eulerParams.angle == 0)
  {
    eulerParams.axis(0) = 1;
  }
  else if (eulerParams.angle == M_PI)
  {
    eulerParams.axis(0) = -1;
  }
  else
  {
    Eigen::Vector3f v;
    v(0) = rotMat(2, 1) - rotMat(1, 2);
    v(1) = rotMat(0, 2) - rotMat(2, 0);
    v(2) = rotMat(1, 0) - rotMat(0, 1);

    eulerParams.axis = 0.5 / sin(eulerParams.angle) * v;
  }
}

inline void TrajectoryGenerator::rpyToRotMat(float roll, float pitch, float yaw, Eigen::Matrix3f& rotMatrix)
{
  // convert rpy angles to rotation matix (yaw pitch roll sequence with consecutive axes)
  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

  Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;

  rotMatrix = q.matrix();
}

// create trajectory message
trajectory_msgs::MultiDOFJointTrajectoryPoint TrajectoryGenerator::generateTrajectoryMessage(
    const float p[6], const float p_dot[6], const float p_ddot[6], const Eigen::Quaternionf& q,
    const Eigen::Vector3f& omega, const Eigen::Vector3f& omega_dot, const ros::Duration& time_from_start)
{
  // TODO Maybe pass reference to output to avoid copying of msg_trajectory

  // set linear position, velocity and acceleration in geometry_msgs::Vector3 structs
  geometry_msgs::Vector3 p_msg;
  p_msg.x = p[0];
  p_msg.y = p[1];
  p_msg.z = p[2];

  geometry_msgs::Vector3 p_dot_msg;
  p_dot_msg.x = p_dot[0];
  p_dot_msg.y = p_dot[1];
  p_dot_msg.z = p_dot[2];

  geometry_msgs::Vector3 p_ddot_msg;
  p_ddot_msg.x = p_ddot[0];
  p_ddot_msg.y = p_ddot[1];
  p_ddot_msg.z = p_ddot[2];

  geometry_msgs::Quaternion q_msg;  // set quaternion struct
  q_msg.w = q.w();
  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();

  geometry_msgs::Vector3 omega_msg;  // set omega struct
  omega_msg.x = omega.x();
  omega_msg.y = omega.y();
  omega_msg.z = omega.z();

  geometry_msgs::Vector3 omega_dot_msg;  // set omega_dot struct
  omega_dot_msg.x = omega_dot.x();
  omega_dot_msg.y = omega_dot.y();
  omega_dot_msg.z = omega_dot.z();

  geometry_msgs::Transform msg_transform;  // Pose
  msg_transform.translation = p_msg;
  msg_transform.rotation = q_msg;

  geometry_msgs::Twist msg_velocities;  // Velocities
  msg_velocities.linear = p_dot_msg;
  msg_velocities.angular = omega_msg;

  geometry_msgs::Twist msg_accelerations;  // Accelerations
  msg_accelerations.linear = p_ddot_msg;
  msg_accelerations.angular = omega_dot_msg;

  trajectory_msgs::MultiDOFJointTrajectoryPoint msg_trajectory;
  msg_trajectory.transforms.push_back(msg_transform);  // in C++ Ros Message Arrays are implemented as std::vector
  msg_trajectory.velocities.push_back(msg_velocities);
  msg_trajectory.accelerations.push_back(msg_accelerations);
  msg_trajectory.time_from_start = time_from_start;

  return msg_trajectory;
}

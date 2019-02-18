#include "trajectory_generator.h"
#include <iostream>

// create Trajectory and send it periodically
bool TrajectoryGenerator::createAndSendTrajectory(
    const geometry_msgs::Vector3& p_start, const geometry_msgs::Vector3& p_end, const geometry_msgs::Vector3& rpy_start,
    const geometry_msgs::Vector3& rpy_end, const float duration, const bool start_tracking,
    const trajectory_types::Type traj_type, trajectory::pos_accelerations& pos_accs,
    trajectory::euler_angle_accelerations& euler_angle_accs, trajectory::euler_angle_velocities& euler_angle_vels, trajectory::euler_angles& euler_angles,
    geometry_msgs::Vector3& eulerAxis, std::vector<double>& time_stamps)
{
  // save input values in 6D array
  float pose_start[7];
  convertTo6DArray(p_start, rpy_start, pose_start);
  float pose_end[7];
  convertTo6DArray(p_end, rpy_end, pose_end);

  float pose_current[6];  // array for 6D pose
  float vel_current[6];   // array for 6D velocities
  float acc_current[6];   // array for 6D accelerations
  // create math objects for quaternion and omega calculations using Eigen library
  Eigen::Quaternionf q_current;
  Eigen::Vector3f omega_current;
  Eigen::Vector3f omega_dot_current;

  // ####### calculation of euler parameters ###########
  // rotation matrix from start and end pose
  Eigen::Matrix3d rot_mat_start;
  rpyToRotMat(pose_start[3], pose_start[4], pose_start[5], rot_mat_start);
  Eigen::Matrix3d rot_mat_end;
  rpyToRotMat(pose_end[3], pose_end[4], pose_end[5], rot_mat_end);

  // retrieve difference rotation
  Eigen::Matrix3d rot_mat_diff = rot_mat_start.transpose() * rot_mat_end;

  // get euler axis and angle
  Eigen::EulerParams eulerParams;
  calculateEulerParameters(rot_mat_diff, eulerParams);

  // assign euler axis to service response
  eulerAxis.x = eulerParams.axis[0];
  eulerAxis.y = eulerParams.axis[1];
  eulerAxis.z = eulerParams.axis[2];

  // temporarely store starting and ending euler angles at 7th position of start and end pose
  pose_start[6] = 0;
  pose_end[6] = eulerParams.angle;

  float a[7][6];  // polynomial coefficients for trajectory, first dimension: axis, second dimension: coefficient (can
                  // be more than 6 for higher order polynoms)

  // calculate polynomial coeffiencts for linear and polynomial trajectory
  switch (traj_type)
  {
    case trajectory_types::Linear:
      // calculate constant linear velocity and acceleration (=0)
      for (int dim = 0; dim < 7; dim++)
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
      for (int dim = 0; dim < 7; dim++)
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

    euler_angle_acc = evaluateAcceleration(a[6][2], a[6][3], a[6][4], a[6][5], t);
    euler_angle_vel = evaluateVelocity(a[6][1], a[6][2], a[6][3], a[6][4], a[6][5], t);
    euler_angle = evaluatePosition(a[6][0], a[6][1], a[6][2], a[6][3], a[6][4], a[6][5], t);

    pos_accs.push_back(pos_acc);
    euler_angle_accs.push_back(euler_angle_acc);
    euler_angle_vels.push_back(euler_angle_vel);
    euler_angles.push_back(euler_angle);
    time_stamps.push_back(t);

    geometry_msgs::Pose pose;
    pose.position.x = evaluatePosition(a[0][0], a[0][1], a[0][2], a[0][3], a[0][4], a[0][5], t);
    pose.position.y = evaluatePosition(a[1][0], a[1][1], a[1][2], a[1][3], a[1][4], a[1][5], t);
    pose.position.z = evaluatePosition(a[2][0], a[2][1], a[2][2], a[2][3], a[2][4], a[2][5], t);

    float roll = evaluatePosition(a[3][0], a[3][1], a[3][2], a[3][3], a[3][4], a[3][5], t);
    float pitch = evaluatePosition(a[4][0], a[4][1], a[4][2], a[4][3], a[4][4], a[4][5], t);
    float yaw = evaluatePosition(a[5][0], a[5][1], a[5][2], a[5][3], a[5][4], a[5][5], t);

    // Each arrow of the pose array visualitation is supposed to point to the direction of the z-Axis, there a 90°
    // rotation against the y-axis is performed after all.
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-0.5 * M_PI, Eigen::Vector3f::UnitY());

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

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
    // calculate trajectory for position, velocity, acceleration, and euler angles trajectory and its derivatives (not
    // equal to omega and omega_dot!!)
    if (traj_type == trajectory_types::Linear || traj_type == trajectory_types::Polynomial)
    {
      // linear trajectory is also a polynomial trajectory with different coeffiencts (set before)
      float dt = (float)(t.toSec() - t_start.toSec());
      for (int dim = 0; dim < 6; dim++)
      {
        pose_current[dim] = a[dim][0] + a[dim][1] * dt + a[dim][2] * pow(dt, 2) + a[dim][3] * pow(dt, 3) +
                            a[dim][4] * pow(dt, 4) + a[dim][5] * pow(dt, 5);
        vel_current[dim] = a[dim][1] + 2 * a[dim][2] * dt + 3 * a[dim][3] * pow(dt, 2) + 4 * a[dim][4] * pow(dt, 3) +
                           5 * a[dim][5] * pow(dt, 4);
        acc_current[dim] =
            2 * a[dim][2] + 6 * a[dim][3] * dt + 12 * a[dim][4] * pow(dt, 2) + 20 * a[dim][5] * pow(dt, 3);
      }
    }
    // add other trajectory types here

    // calculate rotational trajectory from euler angles trajectory
    // calculate quaternion from euler angles
    euler2Quaternion(pose_current[3], pose_current[4], pose_current[5], q_current);
    // calculate omega from euler angles and their derivative
    calculateOmega(pose_current[3], vel_current[3], pose_current[4], vel_current[4], pose_current[5], vel_current[5],
                   omega_current);
    // calculate omega_dot from euler angles and their derivatives
    calculateOmegaDot(pose_current[3], vel_current[3], acc_current[3], pose_current[4], vel_current[4], acc_current[4],
                      pose_current[5], vel_current[5], acc_current[5], omega_dot_current);

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

// convert 2 messages of Vector3 type to 6D float array
void TrajectoryGenerator::convertTo6DArray(const geometry_msgs::Vector3& x_1, const geometry_msgs::Vector3& x_2,
                                           float destination[])
{
  destination[0] = x_1.x;
  destination[1] = x_1.y;
  destination[2] = x_1.z;
  destination[3] = x_2.x * M_PI / 180.0f;  // convert to rad!
  destination[4] = x_2.y * M_PI / 180.0f;  // convert to rad!
  destination[5] = x_2.z * M_PI / 180.0f;  // convert to rad!
}

// convert Euler angles to quaternions using roll-pitch-yaw sequence
void TrajectoryGenerator::euler2Quaternion(const float roll, const float pitch, const float yaw, Eigen::Quaternionf& q)
{
  // following
  // https://stackoverflow.com/questions/21412169/creating-a-rotation-matrix-with-pitch-yaw-roll-using-eigen/21414609
  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

  q = yawAngle * pitchAngle * rollAngle;  //(yaw-pitch-roll sequence) with consequtive axes, q B to I
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

inline void TrajectoryGenerator::calculateEulerParameters(Eigen::Matrix3d rotMat, Eigen::EulerParams& eulerParams)
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

inline void TrajectoryGenerator::rpyToRotMat(float roll, float pitch, float yaw, Eigen::Matrix3d& rotMatrix)
{
  // convert rpy angles to rotation matix (yaw pitch roll sequence with consecutive axes)
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;

  rotMatrix = q.matrix();
}

// calculate angular velocity from euler angles and its derivatives, following Fje94 p.42
void TrajectoryGenerator::calculateOmega(const float roll, const float roll_dot, const float pitch,
                                         const float pitch_dot, const float yaw, const float yaw_dot,
                                         Eigen::Vector3f& omega)
{
  omega.x() = roll_dot - yaw_dot * sin(pitch);
  omega.y() = pitch_dot * cos(roll) + yaw_dot * sin(roll) * cos(pitch);
  omega.z() = -pitch_dot * sin(roll) + yaw_dot * cos(roll) * cos(pitch);
}

// calculate angular acceleration from euler angles and its derivatives, following Fje94 p.42 derivated
void TrajectoryGenerator::calculateOmegaDot(const float roll, const float roll_dot, const float roll_ddot,
                                            const float pitch, const float pitch_dot, const float pitch_ddot,
                                            const float yaw, const float yaw_dot, const float yaw_ddot,
                                            Eigen::Vector3f& omega_dot)
{
  omega_dot.x() = roll_ddot - (yaw_ddot * sin(pitch) + yaw_dot * pitch_dot * cos(pitch));
  omega_dot.y() = pitch_ddot * cos(roll) - pitch_dot * roll_dot * sin(roll) + yaw_ddot * sin(roll) * cos(pitch) +
                  yaw_dot * (roll_dot * cos(roll) * cos(pitch) - pitch_dot * sin(roll) * sin(pitch));
  omega_dot.z() = -(pitch_ddot * sin(roll) + roll_dot * pitch_dot * cos(roll)) + yaw_ddot * cos(roll) * cos(pitch) -
                  yaw_dot * (roll_dot * sin(roll) * cos(pitch) + pitch_dot * cos(roll) * sin(pitch));
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

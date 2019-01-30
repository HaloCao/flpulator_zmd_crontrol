/*
 * @author Nils Dunkelberg
 */

#include <limits>
#include "actuator_simulation.h"

using namespace Eigen;

ActuatorSimulation::ActuatorSimulation()
{
  // retrieve relevant parameters from parameter server
  // todo Inform user about undefined parameters
  ros::param::param<float>("/trajectory/simulation_step_size", dt_, 0.01f);
  ros::param::param<float>("/uav/mass", mass_, 2.5f);
  ros::param::param<float>("/uav/gravity", gravity_, 9.81f);
  ros::param::param<float>("/uav/i_xx", i_xx_, 0.5f);
  ros::param::param<float>("/uav/i_yy", i_yy_, 0.5f);
  ros::param::param<float>("/uav/i_zz", i_zz_, 2.0f);

  ros::param::param<float>("/uav/alpha", alpha_, 13.6f);
  ros::param::param<float>("/uav/beta", beta_, 0.0f);
  ros::param::param<float>("/uav/length", length_, 0.6f);
  ros::param::param<float>("/uav/delta_h", dh_, 0.0f);
  ros::param::param<float>("/uav/k", k_, 0.000056);
  ros::param::param<float>("/uav/b", b_, 0.0000011);

  // actuator velocity boundaries
  ros::param::param<double>("/uav/rotor_vel_max", upper_vel_limit_, 5700);
  ros::param::param<double>("/uav/rotor_vel_min", lower_vel_limit_, 0);

  //store actuator boundaries as rad²/s² as well
  upper_vel_limit_squ_ = pow(upper_vel_limit_ * M_PI / 30, 2);
  lower_vel_limit_squ_ = pow(lower_vel_limit_ * M_PI / 30, 2);
}

void ActuatorSimulation::updateDroneParameters(flypulator_traj_generator::traj_parameterConfig& config)
{
    // updating geometric drone parameters through dynamic reconfigure
    k_ = (float) config.k;
    b_ = (float) config.b;
    dh_ = (float) config.delta_h;
    length_ = (float) config.length;
    alpha_ = (float) config.alpha;
    beta_ = (float) config.beta;
    mass_ = (float) config.mass;
    i_xx_ = (float) config.i_xx;
    i_yy_ = (float) config.i_yy;
    i_zz_ = (float) config.i_zz;
    upper_vel_limit_ = (float) config.rotor_vel_max;
    lower_vel_limit_ = (float) config.rotor_vel_min;    

    //store actuator boundaries as rad²/s² as well
    upper_vel_limit_squ_ = pow(upper_vel_limit_ * M_PI / 30, 2);
    lower_vel_limit_squ_ = pow(lower_vel_limit_ * M_PI / 30, 2);
}

Eigen::Vector6f ActuatorSimulation::getSteadyStateRotorVelocities(Eigen::Vector6f pose)
{
    // retrieve pose orientation as radiation
    Vector3f pose_ori = pose.tail(3) * M_PI / 180;

    // orientation as quaternion
    Quaternionf q = AngleAxisf(pose_ori[2], Vector3f::UnitZ()) * AngleAxisf(pose_ori[1], Vector3f::UnitY()) *
        AngleAxisf(pose_ori[0], Vector3f::UnitX());

    // calculate rotor velocities from quaternion and return them
    return quatToSteadyStateRotorVelocities(q);


}

Eigen::Vector6f ActuatorSimulation::getSteadyStateRotorVelocities(Eigen::Vector3f euler_axis, double euler_angle)
{
    Quaternionf q = eulerParamsToQuat(Vector3f::Zero(), euler_axis, euler_angle);

    // calculate rotor velocities from quaternion and return them
    return quatToSteadyStateRotorVelocities(q);
}

Eigen::Vector6f ActuatorSimulation::quatToSteadyStateRotorVelocities(Eigen::Quaternionf q)
{
    // force/torque input vector, only fz-component for steady state
    Vector6f u = Vector6f::Zero();
    u[2] = mass_ * gravity_;

    // retrieve mapping matrix and calculate squared rotor velocities
    Matrix6f W;
    getMappingMatrix(W, q);

    // get squared rotor velocities and return them
    return W.inverse() * u;
}

void ActuatorSimulation::simulateActuatorVelocities(trajectory::TrajectoryData &traj_data)
{
  // Uav attitude over time (initialize referring to the start pose)
  Quaternionf q;

  // starting orientation in rad
  Vector3f start_ori = traj_data.start_pose_.tail(3) * M_PI / 180;

  q = AngleAxisf(start_ori[2], Vector3f::UnitZ()) * AngleAxisf(start_ori[1], Vector3f::UnitY()) *
      AngleAxisf(start_ori[0], Vector3f::UnitX());

  // Derivation of UAV attitude
  Quaternionf q_dot;

  // Angular velocity over time
  Vector3f omeg(0.0f, 0.0f, 0.0f);

  // static euler axis w.r.t. to the starting orientation {A}
  Vector3f euler_axis_A = traj_data.euler_axis_;

  // track the indices of the maximum and minimum rotor velocities
  double rot_vel_max = std::numeric_limits<double>::min();
  double rot_vel_min = std::numeric_limits<double>::max();

  for (size_t i = 0; i < traj_data.pos_accs_.size(); i++)
  {
    // input vector holding the three force and the three torque components
    Vector6f u;

    // current translational accelerations
    geometry_msgs::Vector3 pos_acc = traj_data.pos_accs_[i];

    // ######### translational (force) components ##############
    u[0] = mass_ * pos_acc.x;
    u[1] = mass_ * pos_acc.y;
    u[2] = mass_ * (pos_acc.z + gravity_);

    // ######### rotational (torque) components ################
    // retrieve orientation of {B} w.r.t. {SA} where A is the starting frame, where the euler axis is defined
    Matrix3f R_AB;
    eulerParamsToRotMatrix(euler_axis_A, traj_data.euler_angles_[i], R_AB);

    // current orientation of the body frame is the tranposed matrix
    Matrix3f R_BA = R_AB.transpose();

    // retrieve euler axis w.r.t. the current body frame
    Vector3f eulerAxis_B = R_BA * euler_axis_A;

    // the angular acceleration w.r.t. the body frame points to the same direction, as the euler axis w.r.t. to the body
    // frame it consequently equals the scalar euler angle acceleration multiplied by the euler axis w.r.t. {B}
    Vector3f omeg_dot = eulerAxis_B * traj_data.euler_angle_accs_[i];

    // retrieve corresponding torques referring the state space model
    u[3] = (omeg_dot[0] - omeg_dot[1] * omeg_dot[2] * (i_yy_ - i_zz_ / i_xx_)) * i_xx_;
    u[4] = (omeg_dot[1] - omeg_dot[0] * omeg_dot[2] * (i_zz_ - i_xx_ / i_yy_)) * i_yy_;
    u[5] = (omeg_dot[2] - omeg_dot[0] * omeg_dot[1] * (i_xx_ - i_yy_ / i_zz_)) * i_zz_;

    // retrieve inverse mapping matrix
    Matrix6f W;
    getMappingMatrix(W, q);

    // get squared rotor velocities and store them
    Vector6f rot_vel_square = W.inverse() * u;
    traj_data.rot_vel_squ_.push_back(rot_vel_square);

    // convert to array (to apply element-wise operations) and retrieve the square root while keeping sign
    ArrayXf rot_vel = rot_vel_square.array();
    rot_vel = rot_vel.sign() * rot_vel.abs().sqrt();

    // convert from rad/s to rpm and write result to reference
    for (size_t j = 0; j < rot_vel.size(); j++)
    {
      double rot_vel_rpm = rot_vel[j] * 60 / (2 * M_PI);
      traj_data.rot_vel_rpm_[j][i] = rot_vel_rpm;

      // new minimum/maximum value?
      if (rot_vel_square[j] > rot_vel_max)
      {
          traj_data.i_max_.first = j;
          traj_data.i_max_.second = i;
          rot_vel_max = rot_vel_square[j];
      }

      else if (rot_vel_square[j] < rot_vel_min)
      {
          traj_data.i_min_.first = j;
          traj_data.i_min_.second = i;
          rot_vel_min = rot_vel_square[j];
      }
    }

    // todo: impelement runge kutta or other low error integration method
    // get derivation of attitude referring to state space model
    q_dot.x() = 0.5 * (q.w() * omeg[0] - q.z() * omeg[1] + q.y() * omeg[2]);
    q_dot.y() = 0.5 * (q.z() * omeg[0] + q.w() * omeg[1] - q.x() * omeg[2]);
    q_dot.z() = 0.5 * (-q.y() * omeg[0] + q.x() * omeg[1] + q.w() * omeg[2]);
    q_dot.w() = -0.5 * (q.x() * omeg[0] + q.y() * omeg[1] + q.z() * omeg[2]);

    // integrate
    q.x() += q_dot.x() * dt_;
    q.y() += q_dot.y() * dt_;
    q.z() += q_dot.z() * dt_;
    q.w() += q_dot.w() * dt_;

    omeg += omeg_dot * dt_;
  }

  // set minimum and maximum (squared) rotor velocities
  traj_data.rot_vel_squ_min_ = rot_vel_min;
  traj_data.rot_vel_squ_max_ = rot_vel_max;
}

void ActuatorSimulation::poseToEulerParams(Eigen::Vector6f pose, Eigen::Vector3f &euler_axis, double &euler_angle)
{
    // convert pose orientation to radiation
    Vector3f pose_ori = pose.tail(3) * M_PI / 180;

    // calculate rotation matrix according to ypr-sequence with consecutive axes
    AngleAxisd roll_angle(pose_ori[0], Vector3d::UnitX());
    AngleAxisd pitch_angle(pose_ori[1], Vector3d::UnitY());
    AngleAxisd yaw_angle(pose_ori[2], Vector3d::UnitZ());

    Quaternion<double> q = yaw_angle * pitch_angle * roll_angle;

    Matrix3d rot_mat = q.matrix();

    // retrieve euler parameters from rotation matrix
    euler_angle = acos(0.5 * (rot_mat(0, 0) + rot_mat(1, 1) + rot_mat(2, 2) - 1));

    // todo: handle singularities
    Vector3f v;
    v(0) = rot_mat(2, 1) - rot_mat(1, 2);
    v(1) = rot_mat(0, 2) - rot_mat(2, 0);
    v(2) = rot_mat(1, 0) - rot_mat(0, 1);

    euler_axis = 0.5 / sin(euler_angle) * v;
}

Eigen::Quaternionf ActuatorSimulation::eulerParamsToQuat(Eigen::Vector3f start_frame, Eigen::Vector3f euler_axis, double euler_angle)
{
    Quaternionf q_IA = AngleAxisf(start_frame[2], Vector3f::UnitZ()) * AngleAxisf(start_frame[1], Vector3f::UnitY()) *
        AngleAxisf(start_frame[0], Vector3f::UnitX());

    Quaternionf q_AB;
    q_AB.x() = euler_axis[0] * sin(euler_angle/2);
    q_AB.y() = euler_axis[1] * sin(euler_angle/2);
    q_AB.z() = euler_axis[2] * sin(euler_angle/2);
    q_AB.w() = cos(euler_angle/2);

    return q_IA * q_AB;
}

inline void ActuatorSimulation::eulerParamsToRotMatrix(Eigen::Vector3f euler_axis, float euler_angle,
                                                       Eigen::Matrix3f &rotMat)
{
  // store single components for better readibility
  float kx = euler_axis[0];
  float ky = euler_axis[1];
  float kz = euler_axis[2];

  float ce = cos(euler_angle);
  float se = sin(euler_angle);
  float ve = 1 - ce;

  rotMat(0, 0) = kx * kx * ve + ce;
  rotMat(0, 1) = kx * ky * ve - kz * se;
  rotMat(0, 2) = kx * kz * ve + ky * se;

  rotMat(1, 0) = kx * ky * ve + kz * se;
  rotMat(1, 1) = ky * ky * ve + ce;
  rotMat(1, 2) = ky * kz * ve - kx * se;

  rotMat(2, 0) = kx * kz * ve - ky * se;
  rotMat(2, 1) = ky * kz * ve + kx * se;
  rotMat(2, 2) = kz * kz * ve + ce;
}

Vector3f ActuatorSimulation::eulerParamsToYPR(Eigen::Vector3f euler_axis, double euler_angle)
{
    // convert euler parameter to rotation matrix
    Matrix3f rot_mat;
    eulerParamsToRotMatrix(euler_axis, euler_angle, rot_mat);

    // retrieve roll pitch yaw angles following ypr-sequence with consecutive axes
    float roll = atan2(rot_mat(2,1), rot_mat(2,2));
    float pitch = -asin(rot_mat(2,0));
    float yaw = atan2(rot_mat(1,0), rot_mat(0,0));

    Vector3f rpy_angles;
    rpy_angles << roll, pitch, yaw;

    return rpy_angles * 180 / M_PI;

}

double ActuatorSimulation::getGravitationalVelocityComponent(Eigen::Quaternionf q, uint rotor_index)
{
    // retrieve inverse mapping matrix for the given orientation q
    Matrix6f W;
    getMappingMatrix(W, q);
    Matrix6f W_inv = W.inverse();

    //return the contribution to the rotor velocity by gravitational force
    return W_inv(rotor_index, 3) * mass_ * gravity_;
}


void ActuatorSimulation::getMappingMatrix(Eigen::Matrix6f &map_matrix, Quaternionf q)
{
    //todo Hardcode inverse mapping matrix
    // compute thrust directions
    Vector3f e_r;   // thrust direction
    Vector3f mom;   // drag + thrust torque
    Vector3f r_ti;  // vector from COM to i-th rotor

    // compute matrix
    for (int i = 0; i < 6; i++)
    {
      float alpha = alpha_ * M_PI / 180.0 * pow(-1, i);
      float beta = beta_;
      float gamma = ((float)i) * M_PI / 3.0f - M_PI/6;   //todo: why -pi/6 in controller_interface?
      // compute thrust direction
      e_r = Eigen::Vector3f(cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma),
                            cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma), cos(alpha) * cos(beta));
      // compute r_ti vector;
      r_ti << length_ * cos(gamma), length_ * sin(gamma), dh_;
      // compute thrust and drag torque
      mom = k_ * r_ti.cross(e_r) + b_ * pow(-1, i) * e_r;
      // save to variable map_matrix
      map_matrix.block(0, i, 3, 1) = k_ * q.matrix() *  e_r;
      map_matrix.block(3, i, 3, 1) = mom;
    }
}

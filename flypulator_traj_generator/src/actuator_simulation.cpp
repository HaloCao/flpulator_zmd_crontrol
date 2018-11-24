/*
 * @todo Description
 * @author Nils Dunkelberg
 */

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

  // actuator velocity boundaries
  ros::param::param<double>("/uav/rotor_vel_max", upper_vel_limit_, 5700);
  ros::param::param<double>("/uav/rotor_vel_min", lower_vel_limit_, 0);
}

void ActuatorSimulation::simulateActuatorVelocities(Eigen::Vector6f &start_pose, trajectory::accelerations &pos_accs,
                                                    trajectory::accelerations &rot_accs,
                                                    trajectory::RotorEvolution &rotor_velocities, bool &feasible)
{
  // Uav attitude over time (initialize referring to the start pose)
  Quaternionf q;
  q = AngleAxisf(start_pose[3], Vector3f::UnitX()) * AngleAxisf(start_pose[4], Vector3f::UnitY()) *
      AngleAxisf(start_pose[5], Vector3f::UnitZ());

  // Derivation of UAV attitude
  Quaternionf q_dot;

  // Angular velocity over time
  Vector3f omeg(0.0f, 0.0f, 0.0f);

  for (size_t i = 0; i < pos_accs.size(); i++)
  {
    // input vector holding the three force and the three torque components
    Vector6f u;

    // current translational and rotational accelerations
    geometry_msgs::Vector3 pos_acc = pos_accs[i];
    geometry_msgs::Vector3 rot_acc = rot_accs[i];

    // ######### translational (force) components ##############
    u[0] = mass_ * pos_acc.x;
    u[1] = mass_ * pos_acc.y;
    u[2] = mass_ * (pos_acc.z + gravity_);

    // ######### rotational (torque) components ################
    // retrieve orientation of {I} w.r.t. {B}
    // todo check euler convention (rpy)
    Matrix3f R_BI = q.toRotationMatrix().transpose();

    // angular acceleration w.r.t. initial frame
    Vector3f omeg_dot_i(rot_acc.x, rot_acc.y, rot_acc.z);

    // transform to body frame
    Vector3f omeg_dot = R_BI * omeg_dot_i;

    // retrieve corresponding torques referring the state space model
    u[3] = (omeg_dot[0] - omeg_dot[1] * omeg_dot[2] * (i_yy_ - i_zz_ / i_xx_)) * i_xx_;
    u[4] = (omeg_dot[1] - omeg_dot[0] * omeg_dot[2] * (i_zz_ - i_xx_ / i_yy_)) * i_yy_;
    u[5] = (omeg_dot[2] - omeg_dot[0] * omeg_dot[1] * (i_xx_ - i_yy_ / i_zz_)) * i_zz_;

    // retrieve inverse mapping matrix
    Matrix6f W_inv;
    getInverseMappingMatrix(W_inv, q);

    // get squared rotor velocities
    Vector6f rot_vel_square = W_inv * u;

    // convert to array (to apply element-wise operations) and retrieve the square root while keeping sign
    ArrayXf rot_vel = rot_vel_square.array();
    rot_vel = rot_vel.sign() * rot_vel.abs().sqrt();

    // convert from rad/s to rpm and write result to reference
    for (size_t j = 0; j < rot_vel.size(); j++)
    {
      double rot_vel_rpm = rot_vel[j] * 60 / (2 * M_PI);
      rotor_velocities[j][i] = rot_vel_rpm;

      // feasibility check
      feasible &= rot_vel_rpm <= upper_vel_limit_;
      feasible &= rot_vel_rpm >= lower_vel_limit_;
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
}

inline void ActuatorSimulation::getInverseMappingMatrix(Matrix6f &map_mat, Quaternionf attitude)
{
  // todo Mapping Matrix is hardcoded for now (referring to fixed uav_params) . It should be calculated programmatically
  // depending on actual uav geometry from parameter server.
  // The hardcoding originates from a matlab script.
  // Quaternion definition from Matlab (q1,q2,q3,q4) = (x, y, z, w),
  // Quaternion definiton from Eigen (q0,q1,q2,q3) = (w, x, y, z)
  // todo unifiy!

  // convert quaternion depiction
  float q1 = attitude.x();
  float q2 = attitude.y();
  float q3 = attitude.z();
  float q4 = attitude.w();

  map_mat(0, 0) =
      (962.1860511 * (-2.429738275e161 * pow(q1, 2) - 2.708843346e177 * q1 * q2 + 4.929691737e176 * q1 * q3 +
                      2.429738275e161 * pow(q2, 2) + 4.929691737e176 * q2 * q4 + 2.429738275e161 * pow(q3, 2) +
                      2.708843346e177 * q3 * q4 - 2.429738275e161 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(0, 1) =
      -(962.1860511 * (-1.354421673e177 * pow(q1, 2) + 4.859476551e161 * q1 * q2 + 4.929691737e176 * q1 * q4 +
                       1.354421673e177 * pow(q2, 2) - 4.929691737e176 * q2 * q3 - 1.354421673e177 * pow(q3, 2) +
                       4.859476551e161 * q3 * q4 + 1.354421673e177 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(0, 2) =
      -(481.0930255 * (4.929691737e176 * pow(q1, 2) + 9.718953101e161 * q1 * q3 + 5.417686692e177 * q1 * q4 +
                       4.929691737e176 * pow(q2, 2) + 5.417686692e177 * q2 * q3 - 9.718953101e161 * q2 * q4 -
                       4.929691737e176 * pow(q3, 2) - 4.929691737e176 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(0, 3) = -1.91674902 * pow(10, -12);

  map_mat(0, 4) = -10684.63398;

  map_mat(0, 5) = -13306.1476;

  map_mat(1, 0) =
      (655.2818796 * (-1.722326264e177 * pow(q1, 2) + 1.988771064e177 * q1 * q2 + 7.238534702e176 * q1 * q3 +
                      1.722326264e177 * pow(q2, 2) + 7.238534702e176 * q2 * q4 + 1.722326264e177 * pow(q3, 2) -
                      1.988771064e177 * q3 * q4 - 1.722326264e177 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(1, 1) =
      -(1310.563759 * (4.971927661e176 * pow(q1, 2) + 1.722326264e177 * q1 * q2 + 3.619267351e176 * q1 * q4 -
                       4.971927661e176 * pow(q2, 2) - 3.619267351e176 * q2 * q3 + 4.971927661e176 * pow(q3, 2) +
                       1.722326264e177 * q3 * q4 - 4.971927661e176 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(1, 2) =
      (1310.563759 * (-1.809633675e176 * pow(q1, 2) - 1.722326264e177 * q1 * q3 + 9.943855322e176 * q1 * q4 -
                      1.809633675e176 * pow(q2, 2) + 9.943855322e176 * q2 * q3 + 1.722326264e177 * q2 * q4 +
                      1.809633675e176 * pow(q3, 2) + 1.809633675e176 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(1, 3) = 9253.164458;

  map_mat(1, 4) = -5342.316991;

  map_mat(1, 5) = 13306.1476;

  map_mat(2, 0) =
      (120.2732564 * (9.383708611e177 * pow(q1, 2) + 1.083537338e178 * q1 * q2 + 3.943753389e177 * q1 * q3 -
                      9.383708611e177 * pow(q2, 2) + 3.943753389e177 * q2 * q4 - 9.383708611e177 * pow(q3, 2) -
                      1.083537338e178 * q3 * q4 + 9.383708611e177 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(2, 1) =
      (120.2732564 * (-5.417686692e177 * pow(q1, 2) + 1.876741722e178 * q1 * q2 - 3.943753389e177 * q1 * q4 +
                      5.417686692e177 * pow(q2, 2) + 3.943753389e177 * q2 * q3 - 5.417686692e177 * pow(q3, 2) +
                      1.876741722e178 * q3 * q4 + 5.417686692e177 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(2, 2) =
      (60.13662819 * (-3.943753389e177 * pow(q1, 2) + 3.753483444e178 * q1 * q3 + 2.167074677e178 * q1 * q4 -
                      3.943753389e177 * pow(q2, 2) + 2.167074677e178 * q2 * q3 - 3.753483444e178 * q2 * q4 +
                      3.943753389e177 * pow(q3, 2) + 3.943753389e177 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(2, 3) = 9253.164458;

  map_mat(2, 4) = 5342.316991;

  map_mat(2, 5) = -13306.1476;

  map_mat(3, 0) =
      (2.39423885e50 * (-1.063750641e114 * pow(q1, 2) - 1.088617906e130 * q1 * q2 + 1.981122571e129 * q1 * q3 +
                        1.063750641e114 * pow(q2, 2) + 1.981122571e129 * q2 * q4 + 1.063750641e114 * pow(q3, 2) +
                        1.088617906e130 * q3 * q4 - 1.063750641e114 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(3, 1) =
      -(2.39423885e50 * (-5.443089528e129 * pow(q1, 2) + 2.127501283e114 * q1 * q2 + 1.981122571e129 * q1 * q4 +
                         5.443089528e129 * pow(q2, 2) - 1.981122571e129 * q2 * q3 - 5.443089528e129 * pow(q3, 2) +
                         2.127501283e114 * q3 * q4 + 5.443089528e129 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(3, 2) =
      -(1.197119425e50 * (1.981122571e129 * pow(q1, 2) + 4.255002566e114 * q1 * q3 + 2.177235811e130 * q1 * q4 +
                          1.981122571e129 * pow(q2, 2) + 2.177235811e130 * q2 * q3 - 4.255002566e114 * q2 * q4 -
                          1.981122571e129 * pow(q3, 2) - 1.981122571e129 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(3, 3) = -2.088113046 * pow(10, -12);

  map_mat(3, 4) = 10684.63398;

  map_mat(3, 5) = 13306.1476;

  map_mat(4, 0) =
      (120.2732564 * (-9.383708611e177 * pow(q1, 2) + 1.083537338e178 * q1 * q2 + 3.943753389e177 * q1 * q3 +
                      9.383708611e177 * pow(q2, 2) + 3.943753389e177 * q2 * q4 + 9.383708611e177 * pow(q3, 2) -
                      1.083537338e178 * q3 * q4 - 9.383708611e177 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(4, 1) =
      -(120.2732564 * (5.417686692e177 * pow(q1, 2) + 1.876741722e178 * q1 * q2 + 3.943753389e177 * q1 * q4 -
                       5.417686692e177 * pow(q2, 2) - 3.943753389e177 * q2 * q3 + 5.417686692e177 * pow(q3, 2) +
                       1.876741722e178 * q3 * q4 - 5.417686692e177 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(4, 2) =
      (60.13662819 * (-3.943753389e177 * pow(q1, 2) - 3.753483444e178 * q1 * q3 + 2.167074677e178 * q1 * q4 -
                      3.943753389e177 * pow(q2, 2) + 2.167074677e178 * q2 * q3 + 3.753483444e178 * q2 * q4 +
                      3.943753389e177 * pow(q3, 2) + 3.943753389e177 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(4, 3) = -9253.164458;

  map_mat(4, 4) = 5342.316991;

  map_mat(4, 5) = -13306.1476;

  map_mat(5, 0) =
      (655.2818796 * (1.722326264e177 * pow(q1, 2) + 1.988771064e177 * q1 * q2 + 7.238534702e176 * q1 * q3 -
                      1.722326264e177 * pow(q2, 2) + 7.238534702e176 * q2 * q4 - 1.722326264e177 * pow(q3, 2) -
                      1.988771064e177 * q3 * q4 + 1.722326264e177 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(5, 1) =
      (1310.563759 * (-4.971927661e176 * pow(q1, 2) + 1.722326264e177 * q1 * q2 - 3.619267351e176 * q1 * q4 +
                      4.971927661e176 * pow(q2, 2) + 3.619267351e176 * q2 * q3 - 4.971927661e176 * pow(q3, 2) +
                      1.722326264e177 * q3 * q4 + 4.971927661e176 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(5, 2) =
      (1310.563759 * (-1.809633675e176 * pow(q1, 2) + 1.722326264e177 * q1 * q3 + 9.943855322e176 * q1 * q4 -
                      1.809633675e176 * pow(q2, 2) + 9.943855322e176 * q2 * q3 - 1.722326264e177 * q2 * q4 +
                      1.809633675e176 * pow(q3, 2) + 1.809633675e176 * pow(q4, 2))) /
      (7.488139347e175 * pow(q1, 4) + 1.497627869e176 * pow(q1, 2) * pow(q2, 2) +
       1.497627869e176 * pow(q1, 2) * pow(q3, 2) + 1.497627869e176 * pow(q1, 2) * pow(q4, 2) +
       7.488139347e175 * pow(q2, 4) + 1.497627869e176 * pow(q2, 2) * pow(q3, 2) +
       1.497627869e176 * pow(q2, 2) * pow(q4, 2) + 7.488139347e175 * pow(q3, 4) +
       1.497627869e176 * pow(q3, 2) * pow(q4, 2) + 7.488139347e175 * pow(q4, 4));

  map_mat(5, 3) = -9253.164458;

  map_mat(5, 4) = -5342.316991;

  map_mat(5, 5) = 13306.1476;
}

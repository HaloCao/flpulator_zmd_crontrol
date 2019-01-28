/*
 * @author Nils Dunkelberg
 */

#ifndef ACUTATORSIMULATION_H
#define ACUTATORSIMULATION_H

#include <QVector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/Vector3.h"

#include "ros/ros.h"

#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <flypulator_traj_generator/traj_parameterConfig.h>

/**
 * \typedef Vector6f Eigen/Matrix which holds the individual rotor velocities.
 * \typedef Matrix6f Eigen/Matrix which holds the Mapping Matrix (mapping from forces/torques to rot. velocities)
 */
namespace Eigen
{
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
}  // namespace Eigen

namespace trajectory
{
/**
 *\typedef pos_accelerations Vector which holds the course of positional accelerations over time.
 *\typedef euler_angle_accelerations Vector which holds the course of rotational accelerations (euler angle only) over
 *time. \typedef RotorEvolution Vector holding the rotor velocities of the hexacopter over time
 */
typedef std::vector<geometry_msgs::Vector3> pos_accelerations;
typedef std::vector<double> euler_angle_accelerations;
typedef std::vector<double> euler_angles;
typedef std::vector<QVector<double>> RotorEvolution;
}  // namespace trajectory

/**
 * \class The ActuatorSimulation class
 * \brief The ActuatorSimulation class implements the actual mathematics to simulate the rotor velocity's course over
 * time. There is basically one central method which takes the required initial accelerations of the trajectory and uses
 * the state space model of the hexacopter and the kinematic model from the ros parameter server to retrieve a solid
 * prediction of the expected rotor velocities. Proceeding from the given forces and torques with regard to the
 * navigation frame, a mapping matrix is used to solve for the rotor velocities. It contains the geometric
 * characteristics as well as the current attitude of the hexacopter.
 */
class ActuatorSimulation
{
public:
  ActuatorSimulation();

  /**
   * \brief simulateActuatorVelocities Calculates the evolution of the rotor rotational velocities based on the
   * trajectory's pose accelerations.
   * \param start_pose Start pose of the trajectory to retrieve initial attitude
   * \param pos_accs Vector of positional accelerations over time
   * \param rot_accs Vector of rotational accelerations over time
   * \param euler_axis constant euler axis of the whole rotational manoeuvre w.r.t the starting frame
   * \param rotor_velocities Reference to a vector of 6D-vectors to store the resulting rotor velocities over time.
   * \param feasible True if given trajectory doesn't exceed the actuator boundaries
   *
   */
  void simulateActuatorVelocities(Eigen::Vector6f &start_pose, trajectory::pos_accelerations &pos_accs,
                                  trajectory::euler_angle_accelerations &rot_accs,
                                  trajectory::euler_angles &euler_angles, geometry_msgs::Vector3 &euler_axis,
                                  trajectory::RotorEvolution &rotor_velocities, bool &feasible);

  /**
   * \brief configCallback Callback for dynamic reconfigure of trajectory parameters
   */
  void updateDroneParameters(flypulator_traj_generator::traj_parameterConfig& config);

  /**
   * \brief getSteadyStateRotorVelocities Calculates the squared rotor velocites for the given 6D-pose and returns them
   * \param pose The pose to calculate rotor velocities for (implies orientation described throug ypr-angles in degree)
   * \return Rotor velocities per rad²/s²
   */
  Eigen::Vector6f getSteadyStateRotorVelocities(Eigen::Vector6f pose);

  /**
   * \brief getSteadyStateRotorVelocities Calculates the squared rotor velocites for the given orientation (representated through euler parameters) and returns them
   * \param euler_axis The euler axis of the steady state orientation
   * \param euler_angle The euler angle of the steady state orientation
   * \return Rotor velocities per rad²/s²
   */
  Eigen::Vector6f getSteadyStateRotorVelocities(Eigen::Vector3f euler_axis, double euler_angle);

  /**
   * \brief poseToEulerParams Calculates the euler parameters for a given pose
   * \param pose Pose to calculate euler parameters for (orientation as degree)
   * \param euler_axis Reference to the resulting euler axis
   * \param euler_angle Reference to the resulting euler angle
   */
  void poseToEulerParams(Eigen::Vector6f pose, Eigen::Vector3f &euler_axis, double &euler_angle);

  /**
   * @brief eulerParamsToYPR Converts from euler parameters to yaw-pitch-roll - angles
   * @param euler_axis The euler axis of the given orientation
   * @param euler_angle The euler angle of the given rotation
   * @return The resulting roll pitch yaw angles following y-p-r-Sequence with consecutive axes (in degrees)
   */
  Eigen::Vector3f eulerParamsToYPR(Eigen::Vector3f euler_axis, double euler_angle);


protected:
  /**
   * \brief eulerParamsToRotMatrix Calculates a rotation matrix from given euler axis and euler angle
   * \param euler_axis  A 3D unit vector corresponding to the euler axis
   * \param euler_angle The angle, indicating the current rotation around the euler axis
   * \param rotMat  The resulting rotation matrix
   */
  inline void eulerParamsToRotMatrix(Eigen::Vector3f euler_axis, float euler_angle, Eigen::Matrix3f &rotMat);

  /**
   * @brief getMappingMatrix Writes the mapping matrix (mapping from rotor velocities to forces/torques) to a given reference
   * \param map_matrix Reference to write resulting mapping matrix to
   * \param q Quaternion indicating the hexacopter's current orientation
   */
  void getMappingMatrix(Eigen::Matrix6f &map_matrix, Eigen::Quaternionf q);

  /**
   * \brief quatToSteadyStateRotorVelocities
   * \param q Quaternion indicating the orientation to calculate rotor velocities for
   * \return 6D-Vector of rotor velocities per rad²/s²
   */
  Eigen::Vector6f quatToSteadyStateRotorVelocities(Eigen::Quaternionf q);

private:
  // simulation parameters
  float dt_;       ///< step size for numerical integrations.
  float gravity_;  ///< gravitational acceleration [m/s²].

  // uav geometric constants
  float mass_;  ///< mass of the hexacopter [kg].
  float i_xx_;  ///< inertia w.r.t. x-axis of the hexacopter [kg*m²].
  float i_yy_;  ///< inertia w.r.t. y-axis of the hexacopter [kg*m²].
  float i_zz_;  ///< inertia w.r.t. z-axis of the hexacopter [kg*m²].

  float k_;
  float b_;
  float alpha_;
  float beta_;
  float length_;
  float dh_;

  // actuator boundaries for feasibility check
  double upper_vel_limit_;  ///< The maximum feasible rotational velocity of a propeller per rpm.
  double lower_vel_limit_;  ///< The minimum feasible rotational velocity of a propeller per rpm.
  double upper_vel_limit_squ_; ///< The maximum feasible velocity per rad²/s²
  double lower_vel_limit_squ_; ///< The minimum feasible velocity per rad²/s²
};

#endif  // ACUTATORSIMULATION_H

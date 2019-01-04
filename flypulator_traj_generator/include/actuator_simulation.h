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

protected:
  /**
   * \brief eulerParamsToRotMatrix Calculates a rotation matrix from given euler axis and euler angle
   * \param euler_axis  A 3D unit vector corresponding to the euler axis
   * \param euler_angle The angle, indicating the current rotation around the euler axis
   * \param rotMat  The resulting rotation matrix
   */
  inline void eulerParamsToRotMatrix(Eigen::Vector3f euler_axis, float euler_angle, Eigen::Matrix3f &rotMat);

  /**
   * \brief getInverseMappingMatrix Writes the mapping matrix (mapping from forces/torques to rotor velocities) to a
   * provided reference.   * \param map_mat Reference to the mapping matrix. \param attitude Current attitude of the
   * hexacopter, since mapping matrix depends on it.
   */
  inline void getInverseMappingMatrix(Eigen::Matrix6f &map_mat, Eigen::Quaternionf attitude);

private:
  // simulation parameters
  float dt_;       ///< step size for numerical integrations.
  float gravity_;  ///< gravitational acceleration [m/s²].

  // uav geometric constants
  float mass_;  ///< mass of the hexacopter [kg].
  float i_xx_;  ///< inertia w.r.t. x-axis of the hexacopter [kg*m²].
  float i_yy_;  ///< inertia w.r.t. y-axis of the hexacopter [kg*m²].
  float i_zz_;  ///< inertia w.r.t. z-axis of the hexacopter [kg*m²].

  // actuator boundaries for feasibility check
  double upper_vel_limit_;  ///< The maximum feasible rotational velocity of a propeller.
  double lower_vel_limit_;  ///< The minimum feasible rotational velocity of a propeller.
};

#endif  // ACUTATORSIMULATION_H

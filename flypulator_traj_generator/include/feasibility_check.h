/*
 * @author Nils Dunkelberg
 */

#ifndef FEASIBILITYCHECK_H
#define FEASIBILITYCHECK_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ros/ros.h"

#include "feasibility_check.h"
#include "actuator_simulation.h"

//#include <dynamic_reconfigure/server.h>
//#include <flypulator_traj_generator/traj_parameterConfig.h>

/**
* \typedef Vector6f Eigen/Matrix which holds the individual rotor velocities.
*/
namespace Eigen
{
typedef Eigen::Matrix<float, 6, 1> Vector6f;
}  // namespace Eigen

//namespace trajectory
//{
///**
// *\typedef pos_accelerations Vector which holds the course of positional accelerations over time.
// *\typedef euler_angle_accelerations Vector which holds the course of rotational accelerations (euler angle only) over
// *time. \typedef RotorEvolution Vector holding the rotor velocities of the hexacopter over time
// */
//typedef std::vector<geometry_msgs::Vector3> pos_accelerations;
//typedef std::vector<double> euler_angle_accelerations;
//typedef std::vector<double> euler_angles;
//typedef std::vector<QVector<double>> RotorEvolution;
//}  // namespace trajectory

/**
 * \class The FeasibilityCheck class
 * \brief The FeasibilityCheck class inspects a given trajectory for feasibility. In case of missing steady state feasibility, it calculates the next feasible endpose. Furthermore it performs a reparametrization of trajectory's duration in order to obtain in-flight feasibility.
 */
class FeasibilityCheck
{
public:
  FeasibilityCheck(ActuatorSimulation* actuator_simulation);

  /**
   * \brief makeFeasible Checks the given start and target pose for feasibility and modifies in terms of feasible target pose and trajectory duration when required.
   * \param start_pose reference to the start pose of the trajectory (orientation in degree)
   * \param target_pose reference to the start pose of the trajectory (orientation in degree)
   * \param duration reference to the start pose of the trajectory (orientation in degree)
   * \return false if failed. This implies the start pose being infeasible
   */
  bool makeFeasible(Eigen::Vector6f &start_pose, Eigen::Vector6f &target_pose, double &duration);

  /**
   * @brief isFeasible Checks the given rotor velocities for feasibility
   * @param rotor_velocities Rotor velocities to check for feasibility
   * @return True if velocities are feasible
   */
  bool isFeasible(Eigen::Vector6f rotor_velocities);


protected:

  /**
   * \brief retrieveFeasibleEndpose Checks forwarded target pose for feasiblity
   * \param target_pose Reference to the target pose of the trajectory (orientation in degree)
   * \param initial_rotor_velocities Squared rotor velocities of the infeasible inital endpose (rad²/s²)
   */
  void retrieveFeasibleEndpose(Eigen::Vector6f &target_pose, Eigen::Vector6f initial_rotor_velocities);

  /**
   * \brief findFeasbileEulerAngle Calculates the maximum feasible euler angle for a pose orientation with the specified euler axis following a newton approach
   * \param rotvel_init Rotor velocity to start newton approach at
   * \param rotvel_limit Upper or lower actuator limit
   * \param euler_axis  Euler axis indicating the "direction" of rotation
   * \param index The index of the critical rotor
   * \return The maximum feasible euler angle
   */
  double findFeasbileEulerAngle(double rotvel_init, double rotvel_limit, Eigen::Vector3f euler_axis, int index);

private:  
  ActuatorSimulation* actuator_simulation_;  ///< Responsible for simulation of the course of rotor speeds based on a given trajectory

  float newt_stepsize_; ///< Stepsize for calculation of first-order divided difference quotient
  double rotvel_buffer_; ///< Buffer value by which the actuator boundaries get tightened for feasible target pose calculation
  double newt_epsilon_; ///< Terminating condition for newton approach, used for feasible target pose calculation

  // actuator boundaries for feasibility calculation
  double upper_vel_limit_;  ///< The maximum feasible rotational velocity of a propeller per rpm.
  double lower_vel_limit_;  ///< The minimum feasible rotational velocity of a propeller per rpm.
  double upper_vel_limit_squ_; ///< The maximum feasible velocity per rad²/s²
  double lower_vel_limit_squ_; ///< The minimum feasible velocity per rad²/s²

};

#endif  // FEASIBILITYCHECK_H

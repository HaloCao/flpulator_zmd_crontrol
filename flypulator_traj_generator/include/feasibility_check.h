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


protected:

  /**
   * \brief retrieveFeasibleEndpose Checks forwarded target pose for feasiblity
   * \param target_pose Reference to the target pose of the trajectory (orientation in degree)
   */
  void retrieveFeasibleEndpose(Eigen::Vector6f &target_pose);

private:  
  ActuatorSimulation* actuator_simulation_;  ///< Responsible for simulation of the course of rotor speeds based on a given trajectory

};

#endif  // FEASIBILITYCHECK_H

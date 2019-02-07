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

typedef unsigned int uint;
typedef std::pair<uint, uint> indices;

namespace trajectory
{
/**
 *\typedef pos_accelerations Vector which holds the course of positional accelerations over time.
 *\typedef euler_angle_accelerations Vector which holds the course of rotational accelerations (euler angle only) over
 *time.
 * \typedef euler_angles Vector which holds the course of the euler angles
 * \typedef rotor_velocities_rpm Vector which holds the rotor velocities in the unit of rev. per minute (QVector for
 *later plotting) \typedef rotor_velocities_squared Vector which holds the rotor velocities in the unit of rad²/s²
 */
typedef std::vector<geometry_msgs::Vector3> pos_accelerations;
typedef std::vector<double> euler_angle_accelerations;
typedef std::vector<double> euler_angle_velocities;
typedef std::vector<double> euler_angles;
typedef std::vector<QVector<double>> rotor_velocities_rpm;
typedef std::vector<Eigen::Vector6f> rotor_velocities_squared;

/**
 * \brief The TrajectoryData struct This structure contains all the relevent input and output trajectory data as well as
 * the QVectors necessary for plotting.
 */
struct TrajectoryData
{
  // input
  Eigen::Vector6f start_pose_;
  Eigen::Vector6f target_pose_;
  trajectory::pos_accelerations pos_accs_;
  trajectory::euler_angle_accelerations euler_angle_accs_;
  trajectory::euler_angle_velocities euler_angle_vels_;
  trajectory::euler_angles euler_angles_;
  Eigen::Vector3f euler_axis_;

  // output
  QVector<double> time_stamps_;
  trajectory::rotor_velocities_rpm rot_vel_rpm_;
  trajectory::rotor_velocities_squared rot_vel_squ_;
  indices i_min_;
  indices i_max_;
  double rot_vel_squ_min_;
  double rot_vel_squ_max_;
  bool feasible_;

  TrajectoryData(Eigen::Vector6f start_pose, Eigen::Vector6f target_pose, trajectory::pos_accelerations pos_accs,
                 trajectory::euler_angle_accelerations euler_angle_accs, trajectory::euler_angle_velocities euler_angle_vels, trajectory::euler_angles euler_angles,
                 Eigen::Vector3f euler_axis, trajectory::rotor_velocities_rpm rotor_velocities_rpm,
                 QVector<double> time_stamps)
    : start_pose_(start_pose)
    , target_pose_(target_pose)
    , pos_accs_(pos_accs)
    , euler_angle_accs_(euler_angle_accs)
    , euler_angle_vels_(euler_angle_vels)
    , euler_angles_(euler_angles)
    , euler_axis_(euler_axis)
    , rot_vel_rpm_(rotor_velocities_rpm)
    , time_stamps_(time_stamps)
    , feasible_(true)
  {
  }
};
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
   * \param trajectory_data contains the relevant inputs (startpose, accelerations...) and used to store the outputs
   * (rotor velocities, location of maximum, minimum etc) (see trajectory::TrajectoryData)
   */
  void simulateActuatorVelocities(trajectory::TrajectoryData &traj_data);

  /**
   * \brief configCallback Callback for dynamic reconfigure of trajectory parameters
   */
  void updateDroneParameters(flypulator_traj_generator::traj_parameterConfig &config);

  /**
   * \brief getSteadyStateRotorVelocities Calculates the squared rotor velocites for the given 6D-pose and returns them
   * \param pose The pose to calculate rotor velocities for (implies orientation described throug ypr-angles in degree)
   * \return Rotor velocities per rad²/s²
   */
  Eigen::Vector6f getSteadyStateRotorVelocities(Eigen::Vector6f pose);

  /**
   * \brief getSteadyStateRotorVelocities Calculates the squared rotor velocites for the given orientation
   * (representated through euler parameters) and returns them \param euler_axis The euler axis of the steady state
   * orientation \param euler_angle The euler angle of the steady state orientation \return Rotor velocities per rad²/s²
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

  /**
   * @brief eulerParamsToQuat Calculates a quaternion from euler parameters, considering the startin frame, in which the
   * euler parameters are defined
   * @param start_frame Orientation of the frame, where the euler parameters are defined w.r.t. the global frame {I}
   * @param euler_axis The euler axis describing the rotation w.r.t the start frame
   * @param euler_angle The euler angle describing the rotation w.r.t the start frame
   * @return Quaternion describing the orientation of the body frame {B} w.r.t. the global frame {I}
   */
  Eigen::Quaternionf eulerParamsToQuat(Eigen::Vector3f start_frame, Eigen::Vector3f euler_axis, double euler_angle);

  /**
   * @brief getGravitationalVelocityComponent Calculates the contribution of gravitational influence to the rotor
   * velocity of the specified rotor
   * @param q The quaternion describing the orientation of the hexarotor w.r.t. the global frame
   * @param rotor_index The index of the regarded rotor
   * @return The rotor velocity influenced by gravitational force
   */
  double getGravitationalVelocityComponent(Eigen::Quaternionf q, uint rotor_index);

protected:
  /**
   * @brief getMappingMatrix Writes the mapping matrix (mapping from rotor velocities to forces/torques) to a given
   * reference \param map_matrix Reference to write resulting mapping matrix to \param q Quaternion indicating the
   * hexacopter's current orientation
   */
  void getMappingMatrix(Eigen::Matrix6f &map_matrix, Eigen::Quaternionf q);

  /**
   * \brief eulerParamsToRotMatrix Calculates a rotation matrix from given euler axis and euler angle
   * \param euler_axis  A 3D unit vector corresponding to the euler axis
   * \param euler_angle The angle, indicating the current rotation around the euler axis
   * \param rotMat  The resulting rotation matrix
   */
  inline void eulerParamsToRotMatrix(Eigen::Vector3f euler_axis, float euler_angle, Eigen::Matrix3f &rotMat);

  /**
   * \brief quatToSteadyStateRotorVelocities
   * \param q Quaternion indicating the orientation to calculate rotor velocities for
   * \return 6D-Vector of rotor velocities per rad²/s²
   */
  Eigen::Vector6f quatToSteadyStateRotorVelocities(Eigen::Quaternionf q);

  /**
   * \brief angularVelocityFromEulerParams Calculates the angular velocity and acceleration w.r.t. the body frame based on current euler parameters
   * \param R_IA The rotation matrix describing the starting orientation w.r.t. {I}
   * \param kA The euler axis defined w.r.t. the starting orientation
   * \param the Current euler angle
   * \param dthe First derivative of current euler angle
   * \param ddthe Second derivative of current euler angle
   * \param omeg Reference to the current angular velocity
   * \param omeg_dot Reference to the current angular acceleration
   */
  inline void angularVelocityFromEulerParams(Eigen::Matrix3f R_IA, Eigen::Vector3f kA, double the, double dthe, double ddthe, Eigen::Vector3f &omeg, Eigen::Vector3f &omeg_dot);

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
};

#endif  // ACUTATORSIMULATION_H

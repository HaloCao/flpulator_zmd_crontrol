/*
 * @author Nils Dunkelberg
 */

#ifndef TRAJECTORYDESIGNER_H
#define TRAJECTORYDESIGNER_H

#include <QWidget>
#include <QTimer>

#include "rviz/view_controller.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "flypulator_traj_generator/polynomial_trajectory.h"

#include "trajectory_ui.h"
#include "actuator_plot.h"
#include "actuator_simulation.h"
#include "feasibility_check.h"

#include <dynamic_reconfigure/server.h>
#include <flypulator_traj_generator/traj_parameterConfig.h>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}  // namespace rviz

/**
 * \typedef polynomial_trajectory Simplify service expression.
 */
typedef flypulator_traj_generator::polynomial_trajectory polynomial_trajectory;

/**
 * \class The TrajectoryDesigner class
 * \brief The Trajectory Designer embeds the main application of the trajectory planning module. It starts a
 * QApplication containing a graphic user interface with a panel to specify the trajectory's parameters, a log area for
 * informative feedback as well as a QCustomPlotting widget, which a priori plots the course of the six rotor velocities
 * over the time of flight. Furthermore it contributes a 3D rviz-Render Scene where the start and target pose and the
 * spacial course of the trajectory are visualized.
 */
class TrajectoryDesigner : public QWidget
{
  Q_OBJECT
public:
  TrajectoryDesigner(QWidget* parent = nullptr);
  virtual ~TrajectoryDesigner();

private Q_SLOTS:

  /**
   * \brief Will be excecuted periodicly for spinning the ros-node.
   * Shuts down the main window if ros stopped running.
   */
  void rosUpdate();

  /**
   * \brief Will be executed whenever the user manipulates the trajectory set-up.
   * \param start_tracking True if the generated trajectory is to be published.
   */
  void updateTrajectoryCallback(bool start_tracking);

  /**
   * \brief configCallback Callback for dynamic reconfigure of trajectory parameters
   */
  void configCallback(flypulator_traj_generator::traj_parameterConfig& config, uint32_t level);

  /**
   * \brief makeFeasibleCallback Will be executed, when user hits button to calculate feasible trajectory. Triggers the
   * calculation and the plotting of the feasible trajectory afterwards.
   */
  void makeFeasibleCallback();

private:
  bool visualize_;      ///< true if the 3d render scene and the actuator_plot shall be shown
  ros::NodeHandle nh_;  ///< Interface to register standard ros components

  ros::ServiceClient polynomial_traj_client_;     ///< Service client which calls ros-service to create polynomial
                                                  ///< trajectories.
  tf::TransformListener poses_changed_listener_;  ///< Listens to new transforms between world and start_pose /
                                                  ///< target_pose frames
  ros::Time start_tf_stamp_;   ///< Stores the time stamp when the last transform between world and start pose fram was
                               ///< published.
  ros::Time target_tf_stamp_;  ///< Stores the time stamp when the last transform between world and target pose fram was
                               ///< published.

  rviz::VisualizationManager* manager_;  ///< Central manager of rviz, holding displays, viewcontrollers etc.
  rviz::RenderPanel* render_panel_;      ///< Widget which shows OGRE-rendered scene in RViz.

  rviz::Display* grid_;          ///< Displays a finite 2D grid in 3D render space.
  rviz::Display* start_frame_;   ///< Coordinate system representing the frame of trajectory's start position.
  rviz::Display* target_frame_;  ///< Coordinate system representing the frame of trajectory's target position.
  rviz::Display* uav_model_;     ///< Model of the hexacopter
  rviz::Display* path_;          ///< Displays the current trajectory as a pose array.

  TrajectoryUI* ui_panel_;       ///< User interface to set the parametrization of a desired trajectory.
  ActuatorPlot* actuator_plot_;  ///< holds a qCustomPlot widget which plots the behaviour of the rotor velocities.
  FeasibilityCheck* feasibility_check_;  ///< Performs feasibility check for a given trajectory and calculates feasible
                                         ///< alternative where required

  dynamic_reconfigure::Server<flypulator_traj_generator::traj_parameterConfig> dr_srv;  ///< server instance for dynamic
                                                                                        ///< reconfigure of trajectory
                                                                                        ///< parameters
  dynamic_reconfigure::Server<flypulator_traj_generator::traj_parameterConfig>::CallbackType cb;  ///< callback for dyn.
                                                                                                  ///< reconfigure
};

#endif  // TRAJECTORYDESIGNER_H

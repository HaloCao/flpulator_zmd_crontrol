/*
 * @todo Description
 * @author Nils Dunkelberg
 */

#ifndef TRAJECTORYDESIGNER_H
#define TRAJECTORYDESIGNER_H

#include <QWidget>
#include <QTimer>

#include "rviz/view_controller.h"
#include <ros/ros.h>

#include "flypulator_traj_generator/polynomial_trajectory.h"

#include "trajectory_ui.h"
#include "actuator_plot.h"
#include "actuator_simulation.h"

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

class TrajectoryDesigner : public QWidget
{
  Q_OBJECT
public:
  TrajectoryDesigner(QWidget* parent = nullptr);
  virtual ~TrajectoryDesigner();

protected:
  /**
   * \brief Callback for a key event.
   * Key Declarations:
   * F ~ Toggle Fullscreen
   * M ~ Toggle Maximize/Minimize
   * Q ~ Quit Application
   */
  bool eventFilter(QObject* object, QEvent* event);

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
  void callTrajectoryGenerator(bool start_tracking);

private:
  ros::NodeHandle nh_;  ///< Interface to register standard ros components

  ros::ServiceClient polynomial_traj_client_;  ///< Service client which calls ros-service to create polynomial
                                               ///< trajectories.

  rviz::VisualizationManager* manager_;  ///< Central manager of rviz, holding displays, viewcontrollers etc.
  rviz::RenderPanel* render_panel_;      ///< Widget which shows OGRE-rendered scene in RViz.

  rviz::Display* grid_;  ///< Displays a finite 2D grid in 3D render space.
  rviz::Display* start_frame_; ///< Coordinate system representing the frame of trajectory's start position.
  rviz::Display* target_frame_; ///< Coordinate system representing the frame of trajectory's target position.
  rviz::Display* uav_model_; ///< Model of the hexacopter


  TrajectoryUI* ui_panel_;       ///< User interface to set the parametrization of a desired trajectory.
  ActuatorPlot* actuator_plot_;  ///< holds a qCustomPlot widget which plots the behaviour of the rotor velocities.
  ActuatorSimulation* actuator_simulation_;  ///< Responsible for simulation of the course of rotor speeds based on a
                                             ///< given trajectory
};

#endif  // TRAJECTORYDESIGNER_H

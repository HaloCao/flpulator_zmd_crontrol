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

#include "trajectory_ui.h"

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}  // namespace rviz

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

private:
  rviz::VisualizationManager* manager_;  ///< Central manager of rviz, holding displays, viewcontrollers etc.
  rviz::RenderPanel* render_panel_;      ///< Widget which shows OGRE-rendered scene in RViz

  rviz::Display* grid_;  ///< Displays a finite 2D grid in 3D render space

  TrajectoryUI* ui_panel_; ///< User interface to set the parametrization of a desired trajectory
};

#endif  // TRAJECTORYDESIGNER_H

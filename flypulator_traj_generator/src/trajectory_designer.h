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

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

class TrajectoryDesigner: public QWidget
{
    Q_OBJECT
public:
    TrajectoryDesigner(QWidget *parent = nullptr);
virtual ~TrajectoryDesigner();

protected:
    /**
     * @brief Callback for a key event.
     * Key Declarations:
     * F ~ Toggle Fullscreen
     * M ~ Toggle Maximize/Minimize
     * Q ~ Quit Application
     *
     */
    bool eventFilter(QObject *object, QEvent *event);

private Q_SLOTS:

    /**
     * @brief Will be excecuted periodicly for spinning the ros-node.
     * Shuts down the main window if ros stopped running.
     */
    void ros_update();

private:
    rviz::VisualizationManager* manager_; ///< Central manager of rviz, holding displays, viewcontrollers etc.
    rviz::RenderPanel* render_panel_; ///< Widget which shows OGRE-rendered scene in RViz

    ros::NodeHandle nh_; ///< interface to register standard ros components

    rviz::Display* grid_; ///< Displays a finite 2D grid in 3D render space
};

#endif // TRAJECTORYDESIGNER_H

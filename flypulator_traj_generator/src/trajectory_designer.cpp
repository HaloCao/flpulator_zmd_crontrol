/*
 * @todo Description
 * @author Nils Dunkelberg
 */

#include <QColor>
#include <QHBoxLayout>
#include <QEvent>
#include <QKeyEvent>
#include <QApplication>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/view_controller.h"
#include "ros/ros.h"

#include "trajectory_designer.h"

void TrajectoryDesigner::ros_update()
{
  if (ros::ok())
  {
    ros::spinOnce();
  }
  else
  {
    QApplication::quit();
  }
}

TrajectoryDesigner::TrajectoryDesigner(QWidget *parent) : QWidget(parent)
{
  // updates the ROS-Spin at 50 Hz (necessary for node communication) and listens to ros-shutdown
  QTimer *t = new QTimer(this);
  QObject::connect(t, SIGNAL(timeout()), this, SLOT(ros_update()));
  t->start(20);

  // Create render panel widget to hold ogre-rendered scene
  render_panel_ = new rviz::RenderPanel();
  QHBoxLayout *main_layout = new QHBoxLayout;

  main_layout->setSpacing(0);
  main_layout->setContentsMargins(0, 0, 0, 0);

  main_layout->addWidget(render_panel_);

  // Set the top-level layout for this TrajectoryDesigner widget.
  setLayout(main_layout);

  // Initialize 3D View (VisualizationManager and RenderPanel)
  manager_ = new rviz::VisualizationManager(render_panel_);
  render_panel_->initialize(manager_->getSceneManager(), manager_);
  manager_->initialize();
  manager_->startUpdate();

  // show grid
  grid_ = manager_->createDisplay("rviz/Grid", "base grid", true);
  ROS_ASSERT(grid_ != NULL);
  grid_->subProp("Color")->setValue(QColor(Qt::black));

  // apply keypresses
  qApp->installEventFilter(this);
}

// Destructor.
TrajectoryDesigner::~TrajectoryDesigner()
{
  delete manager_;
}

bool TrajectoryDesigner::eventFilter(QObject *object, QEvent *event)
{
  static int i = 0;
  if (event->type() == QEvent::KeyPress)
  {
    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
    switch (keyEvent->key())
    {
      case Qt::Key_F:
        if (isFullScreen())
        {
          ROS_INFO("fullscreen disabled by keypress");
          showNormal();
        }
        else
        {
          ROS_INFO("fullscreen enabled by keypress");
          showFullScreen();
        }
        return true;

      case Qt::Key_M:
        if (isMaximized())
        {
          ROS_INFO("maximized disabled by keypress");
          showNormal();
        }
        else
        {
          ROS_INFO("maximized enabled by keypress");
          showMaximized();
        }
        return true;

      case Qt::Key_Q:
        ROS_INFO("shutdown by keypress");
        ros::shutdown();
        qApp->quit();
        return true;
    }
  }
  return false;
}

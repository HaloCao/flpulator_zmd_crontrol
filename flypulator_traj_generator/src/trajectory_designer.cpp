/*
 * @todo Description
 * @author Nils Dunkelberg
 */

#include <QColor>
#include <QGridLayout>
#include <QEvent>
#include <QKeyEvent>
#include <QApplication>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/view_controller.h"
#include "ros/ros.h"

#include "trajectory_designer.h"

TrajectoryDesigner::TrajectoryDesigner(QWidget *parent)
    : QWidget(parent),
      ui_panel_(new TrajectoryUI(this)),
      actuator_plot_(new ActuatorPlot(this))

{
  // updates the ROS-Spin at 50 Hz (necessary for node communication) and listens to ros-shutdown
  QTimer *t = new QTimer(this);
  QObject::connect(t, SIGNAL(timeout()), this, SLOT(rosUpdate()));
  t->start(20);

  // set user interface panel to fixed width and actuator plot widget to fixed height
  ui_panel_->setFixedWidth(500);
  actuator_plot_->setFixedHeight(400);

  // Create render panel widget to hold ogre-rendered scene
  render_panel_ = new rviz::RenderPanel();
  QGridLayout *main_layout = new QGridLayout;

  main_layout->setSpacing(0);
  main_layout->setContentsMargins(0, 0, 0, 0);

  // add user interface panel and the 3D View
  main_layout->addWidget(ui_panel_, 0, 0, 2, 1);
  main_layout->addWidget(render_panel_, 0, 1);
  main_layout->addWidget(actuator_plot_, 1, 1);

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
  grid_->subProp("Color")->setValue(QColor(Qt::white));

  // apply keypresses
  qApp->installEventFilter(this);

  // connect ui_panel poseupdate signal to local slot to get informed about updates in the trajectory set-up
  connect(ui_panel_, SIGNAL(poseUpdate()), this, SLOT(callTrajectoryGenerator()));
}

void TrajectoryDesigner::rosUpdate()
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

void TrajectoryDesigner::callTrajectoryGenerator() {
    // Create references to retrieve the current trajectory setup from user interface
    Eigen::Vector6f start_pose;
    Eigen::Vector6f target_pose;
    double duration;

    // Write references
    ui_panel_->getTrajectorySetup(start_pose, target_pose, duration);

}

// Destructor.
TrajectoryDesigner::~TrajectoryDesigner()
{
  delete manager_;
}

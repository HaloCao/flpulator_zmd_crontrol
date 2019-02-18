/*
 * The main() function initializes ROS, creates a QApplication, creates the top-level widget (of type
 * "TrajectoryDesigner"), shows it, and runs the Qt event loop.
 * @author Nils Dunkelberg
 */

#include <QApplication>
#include <QTimer>
#include <QObject>
#include <ros/ros.h>

#include "trajectory_designer.h"

int main(int argc, char** argv)
{
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "trajectory_designer", ros::init_options::AnonymousName);
  }

  QApplication app(argc, argv);

  TrajectoryDesigner* td = new TrajectoryDesigner();
  td->setWindowTitle("Flypulator - Trajectory Designer");

  app.exec();

  delete td;
}

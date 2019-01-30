/*
 * @author Nils Dunkelberg
 */

#include <QColor>
#include <QGridLayout>
#include <QVector>
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
  : QWidget(parent)
  , ui_panel_(new TrajectoryUI(this))
  , actuator_plot_(new ActuatorPlot(this))
  , actuator_simulation_(new ActuatorSimulation())
  , feasibility_check_(new FeasibilityCheck(actuator_simulation_))

{
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
  // todo: set camera such that whole default pose set is visible
  manager_ = new rviz::VisualizationManager(render_panel_);
  manager_->setFixedFrame("world");
  render_panel_->initialize(manager_->getSceneManager(), manager_);
  manager_->initialize();
  manager_->startUpdate();

  // show grid
  grid_ = manager_->createDisplay("rviz/Grid", "base grid", true);
  ROS_ASSERT(grid_ != NULL);
  grid_->subProp("Color")->setValue(QColor(Qt::white));

  // show start and target pose frames
  start_frame_ = manager_->createDisplay("rviz/Axes", "start_frame", true);
  start_frame_->subProp("Reference Frame")->setValue("start_pose");

  target_frame_ = manager_->createDisplay("rviz/Axes", "target_frame", true);
  target_frame_->subProp("Reference Frame")->setValue("target_pose");

  // show model of the hexacopter
  uav_model_ = manager_->createDisplay("rviz/RobotModel", "uav_hexacopter", true);
  uav_model_->subProp("Robot Description")->setValue("/robot_description");

  // show current trajectory as pose array
  path_ = manager_->createDisplay("rviz/PoseArray", "trajectory_posearray", true);
  path_->subProp("Topic")->setValue("trajectory/visualization");
  path_->subProp("Arrow Length")->setValue(0.03);

  // apply keypresses
  qApp->installEventFilter(this);

  // connect ui_panel poseupdate signal to local slot to get informed about updates in the trajectory set-up
  connect(ui_panel_, SIGNAL(startTracking(bool)), this, SLOT(callTrajectoryGenerator(bool)));

  //connect ui_panel makeFeasible signal to local slot in order to calculate feasible trajectory
  connect(ui_panel_, SIGNAL(makeFeasible()), this, SLOT(makeFeasibleCallback()));

  // register ros service client for polynomial trajectory generation service+
  polynomial_traj_client_ = nh_.serviceClient<polynomial_trajectory>("polynomial_trajectory");

  // setup server for dynamic reconfigure of trajectory generation parameters
  cb = boost::bind(&TrajectoryDesigner::configCallback, this, _1, _2);
  dr_srv.setCallback(cb);

  // updates the ROS-Spin at 50 Hz (necessary for node communication) and listens to ros-shutdown
  QTimer *t = new QTimer(this);
  QObject::connect(t, SIGNAL(timeout()), this, SLOT(rosUpdate()));
  t->start(20);

}

void TrajectoryDesigner::rosUpdate()
{
  if (ros::ok())
  {
    ros::spinOnce();

    // look for new transform between /world and /start_pose, if not available, look for new transform between /world
    // and /target_pose. Less elegant workaround.. Other solution would be to inform the trajectory designer via signal
    // slot concept about new pose configurations, but the callback would run on the gui-thread, thus blocking user
    // interactions and leading to laggy slider movements. The implementation should consider the callback to run in a
    // dedicated thread.
    tf::StampedTransform transform;
    try
    {
      poses_changed_listener_.lookupTransform("/world", "/start_pose", ros::Time(0), transform);
      if (transform.stamp_ != start_tf_stamp_)
      {
        start_tf_stamp_ = transform.stamp_;

        // calculate new trajectory
        callTrajectoryGenerator(false);

        return;
      }

      poses_changed_listener_.lookupTransform("/world", "/target_pose", ros::Time(0), transform);
      if (transform.stamp_ != target_tf_stamp_)
      {
        target_tf_stamp_ = transform.stamp_;

        // calculate new trajectory
        callTrajectoryGenerator(false);
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
  else
  {
    QApplication::quit();
  }
}

void TrajectoryDesigner::configCallback(flypulator_traj_generator::traj_parameterConfig& config, uint32_t level)
{
    // update parameters of actuator simulation through dynamic reconfigure
    actuator_simulation_->updateDroneParameters(config);

    // update upper and lower actuator boundaries for plotting purposes
    actuator_plot_->updateActuatorBoundaries((float) config.rotor_vel_max, (float) config.rotor_vel_min);

    // restart simulation
    callTrajectoryGenerator(false);

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

void TrajectoryDesigner::makeFeasibleCallback()
{
    // Create references to retrieve the current trajectory setup from user interface
    Eigen::Vector6f start_pose;
    Eigen::Vector6f target_pose;
    double duration;

    // Write references (orientation in degrees)
    ui_panel_->getTrajectorySetup(start_pose, target_pose, duration);

    // pass trajectory setup to feasibility check class and retrieve feasible alternative when required
    feasibility_check_->makeFeasible(start_pose, target_pose, duration);

    // set the user interface to the new feasible target pose and duration
    ui_panel_->setDuration(duration);
    ui_panel_->setTargetPose(target_pose);


}

void TrajectoryDesigner::callTrajectoryGenerator(bool start_tracking)
{
  // Create references to retrieve the current trajectory setup from user interface
  Eigen::Vector6f start_pose;
  Eigen::Vector6f target_pose;
  double duration;

  // Write references
  ui_panel_->getTrajectorySetup(start_pose, target_pose, duration);

  // create service
  polynomial_trajectory pt_srv;

  // fill service parameters
  pt_srv.request.p_start.x = start_pose[0];
  pt_srv.request.p_start.y = start_pose[1];
  pt_srv.request.p_start.z = start_pose[2];
  pt_srv.request.rpy_start.x = start_pose[3];
  pt_srv.request.rpy_start.y = start_pose[4];
  pt_srv.request.rpy_start.z = start_pose[5];

  pt_srv.request.p_end.x = target_pose[0];
  pt_srv.request.p_end.y = target_pose[1];
  pt_srv.request.p_end.z = target_pose[2];
  pt_srv.request.rpy_end.x = target_pose[3];
  pt_srv.request.rpy_end.y = target_pose[4];
  pt_srv.request.rpy_end.z = target_pose[5];

  pt_srv.request.duration = duration;

  pt_srv.request.start_tracking = start_tracking;

  if (!polynomial_traj_client_.call(pt_srv))
  {
    ROS_ERROR("[flypulator_traj_generator] Failed to call polynomial trajectory service.");
    return;
  }

  // Create Vector to store rotor velocity evolution
  size_t size = pt_srv.response.p_acc.size();
  trajectory::rotor_velocities_rpm rotor_velocities_rpm(6, QVector<double>(size));

  // convert eulerAxis from geometry message to Vector3f
  Eigen::Vector3f euler_axis(pt_srv.response.euler_axis.x, pt_srv.response.euler_axis.y, pt_srv.response.euler_axis.z);

  // initialize trajectory data struct and fill it
  trajectory::TrajectoryData traj_data(start_pose, target_pose, pt_srv.response.p_acc, pt_srv.response.euler_angle_acc, pt_srv.response.euler_angle, euler_axis, rotor_velocities_rpm);

  // simulate the rotor velocities for the current trajectory
  actuator_simulation_->simulateActuatorVelocities(traj_data);

  // convert timestamps to qvector and draw actuator evolution to custom plot
  QVector<double> time_stamps = QVector<double>::fromStdVector(pt_srv.response.time_stamps);
  actuator_plot_->plotActuatorEvolution(traj_data.rot_vel_rpm_, time_stamps, true);
}

// Destructor.
TrajectoryDesigner::~TrajectoryDesigner()
{
  delete manager_;
}

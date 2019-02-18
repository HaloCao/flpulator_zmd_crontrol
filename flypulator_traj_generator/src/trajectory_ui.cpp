/*
 * @author Nils Dunkelberg
 */

#include <QApplication>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QSpacerItem>

#include "trajectory_ui.h"

TrajectoryUI::TrajectoryUI(QWidget *parent)
  : QWidget(parent)
  , start_slide_mapper_(new QSignalMapper(this))
  , target_slide_mapper_(new QSignalMapper(this))
  , start_spin_mapper_(new QSignalMapper(this))
  , target_spin_mapper_(new QSignalMapper(this))

{
  // main layout of the trajectory user interface
  main_layout_ = new QVBoxLayout();

  // set title of pose panel structure to set user interface title dynamically
  start_pose_panel_.title_ = "Start Pose";
  target_pose_panel_.title_ = "Target Pose";

  // add panels with interface for setting start and target pose of the desired trajectory
  initPosePanel(start_pose_panel_, *start_slide_mapper_, *start_spin_mapper_);
  initPosePanel(target_pose_panel_, *target_slide_mapper_, *target_spin_mapper_);

  // connect the signal mapper to the slider as well as spin box callbacks
  connect(start_slide_mapper_, SIGNAL(mapped(int)), this, SLOT(startposeSliderCallback(int)));
  connect(target_slide_mapper_, SIGNAL(mapped(int)), this, SLOT(targetposeSliderCallback(int)));

  connect(start_spin_mapper_, SIGNAL(mapped(int)), this, SLOT(startposeSpinbCallback(int)));
  connect(target_spin_mapper_, SIGNAL(mapped(int)), this, SLOT(targetposeSpinbCallback(int)));

  // add panel to adjust flight time
  initFlightTimePanel();

  // add reset and start buttons and connect them to their callbacks
  QPushButton *reset_btn = new QPushButton("Reset Poses");
  QPushButton *start_btn = new QPushButton("Start Trajectory Tracking");
  QPushButton *match_start_pose_btn = new QPushButton("Align Start to Drone Pose");
  QPushButton *make_feasible_btn = new QPushButton("Find Feasible Trajectory");

  connect(reset_btn, SIGNAL(clicked()), this, SLOT(resetPoseConfigurations()));
  connect(start_btn, SIGNAL(clicked()), this, SLOT(startTrajectoryTracking()));
  connect(match_start_pose_btn, SIGNAL(clicked()), this, SLOT(alignStartDronePose()));
  connect(make_feasible_btn, SIGNAL(clicked()), this, SLOT(calculateFeasibleTrajectory()));

  // add buttons to horizontal layout and to main layout finally
  QGridLayout *start_reset_layout = new QGridLayout();
  start_reset_layout->addWidget(reset_btn, 0, 0);
  start_reset_layout->addWidget(match_start_pose_btn, 1, 0);
  start_reset_layout->addWidget(make_feasible_btn, 0, 1);
  start_reset_layout->addWidget(start_btn, 1, 1);
  QWidget *container = new QWidget();
  container->setLayout(start_reset_layout);
  main_layout_->addWidget(container);

  // add panel for log messages
  QGroupBox *log_gbox = new QGroupBox("Log");
  QVBoxLayout *log_layout = new QVBoxLayout();
  log_panel_ = new QTextEdit();
  log_panel_->setReadOnly(true);
  log_layout->addWidget(log_panel_);
  log_gbox->setLayout(log_layout);
  main_layout_->addWidget(log_gbox);

  // remove the padding of the buttons within the Spin boxes (no additional buttons used)
  qApp->setStyleSheet("QDoubleSpinBox::up-button {width: 0px;}  QDoubleSpinBox::down-button {width: 0px;}"
                      "QSpinBox::up-arrow {width: 0px;} QSpinBox::up-arrow {width: 0px;}");

  setLayout(main_layout_);

  log("Initialized");

  // set initial pose configuration
  resetPoseConfigurations();

  // broadcast transforms for start- and target_pose frame to initially update the corresponding
  // coord.axes-visualization.
  broadcastStartTransform();
  broadcastTargetTransform();
}

void TrajectoryUI::initPosePanel(PosePanel &pose_panel, QSignalMapper &slide_mapper, QSignalMapper &spin_mapper)
{
  // store start pose panel in a groupbox
  QGroupBox *pose_panel_gbox = new QGroupBox(pose_panel.title_);

  // grid layout of the group box
  QGridLayout *gbox_layout = new QGridLayout();

  // Define the label of the pose components and its units
  QString pose_ids[6] = {"x", "y", "z", QChar(0xC6, 0x03), QChar(0xb8, 0x03), QChar(0xc8, 0x03)};
  QString pose_units[6] = {" m", " m", " m", " °", " °", " °"};

  // Define the maximum absolute values of each pose component
  // (multiplied by 100 to obtain higher precision after double conversion)
  int max_pose[6] = {5000, 5000, 5000, 2000, 2000, 18000};

  // for each pose component apply label, slider and its unit
  for (int i = 0; i < 6; i++)
  {
    // initialize pose component label
    pose_panel.pose_labels_[i] = new QLabel(pose_ids[i]);

    // intialize slider as well as the spinbox containing the current pose component's value and unit
    pose_panel.sliders_[i] = new QSlider(Qt::Horizontal, this);
    pose_panel.pose_values_[i] = new QDoubleSpinBox();

    // specifiy their properties
    initPoseCompSlider(*(pose_panel.sliders_[i]), *(pose_panel.pose_values_[i]), pose_units[i], pose_panel.multiplier,
                       max_pose[i]);

    // dynamically connect each slider's and spin box's valueChanged signal to the signal mapper's mapping-Slot
    // everytime a slider/ spin box is manipulated, the corresponding slider/ spin box callback is triggered and given
    // the index of the active slider/ spin box
    connect(pose_panel.sliders_[i], SIGNAL(valueChanged(int)), &slide_mapper, SLOT(map()));
    connect(pose_panel.pose_values_[i], SIGNAL(valueChanged(double)), &spin_mapper, SLOT(map()));
    slide_mapper.setMapping(pose_panel.sliders_[i], i);
    spin_mapper.setMapping(pose_panel.pose_values_[i], i);

    // for each pose component add all required widgets as a new row to the groupbox (grid) layout
    gbox_layout->addWidget(pose_panel.pose_labels_[i], i, 0);
    gbox_layout->addWidget(pose_panel.sliders_[i], i, 1);
    gbox_layout->addWidget(pose_panel.pose_values_[i], i, 2);
  }

  // apply newly created layout to the pose panel group box and the pose panel to the main layout
  pose_panel_gbox->setLayout(gbox_layout);
  main_layout_->addWidget(pose_panel_gbox);
}

void TrajectoryUI::initPoseCompSlider(QSlider &slider, QDoubleSpinBox &spinbox, QString unit, int multiplier,
                                      double maximum)
{
  // slider properties
  slider.setRange(-maximum, maximum);
  slider.setTickInterval(maximum / 5);
  slider.setTickPosition(QSlider::TicksBelow);

  // spinbox properties
  spinbox.setFixedWidth(70);
  spinbox.setButtonSymbols(QAbstractSpinBox::NoButtons);
  spinbox.setSuffix(unit);
  spinbox.setMaximum(maximum / (double)multiplier);
  spinbox.setMinimum(-maximum / (double)multiplier);
  spinbox.setDecimals(2);
  spinbox.setAlignment(Qt::AlignRight);
  spinbox.setValue(slider.value() / (double)multiplier);
}

void TrajectoryUI::initFlightTimePanel()
{
  // set common properties
  int max_duration = 5000;
  QString unit = " s";
  QString label = "T";
  int multiplier = start_pose_panel_.multiplier;

  // initialize gbox widget and layout
  QGroupBox *dur_gbox = new QGroupBox("Duration");
  QHBoxLayout *dur_gbox_layout = new QHBoxLayout();

  // create spin box and slider
  dur_slider_ = new QSlider(Qt::Horizontal, this);
  dur_value_ = new QDoubleSpinBox();
  initPoseCompSlider(*dur_slider_, *dur_value_, unit, multiplier, max_duration);

  // correct minimums
  dur_slider_->setMinimum(0);
  dur_value_->setMinimum(0);

  // add widgets to group box layout
  dur_gbox_layout->addWidget(new QLabel(label));
  dur_gbox_layout->addWidget(dur_slider_);
  dur_gbox_layout->addWidget(dur_value_);

  // add to group box and main layout
  dur_gbox->setLayout(dur_gbox_layout);

  main_layout_->addWidget(dur_gbox);

  // connect spin box and slider to their callbacks
  connect(dur_slider_, SIGNAL(valueChanged(int)), this, SLOT(durationSliderCallback(int)));
  connect(dur_value_, SIGNAL(valueChanged(double)), this, SLOT(durationSpinbCallback(double)));
}

void TrajectoryUI::startposeSliderCallback(int id)
{
  // since qslider do not support doubles per default, they hold each value multiplied by a constant factor
  // to obtain higher precision of the actual pose component values
  double new_val = start_pose_panel_.sliders_[id]->value() / (double)start_pose_panel_.multiplier;
  start_pose_panel_.pose_values_[id]->setValue(new_val);
  start_pose_[id] = new_val;
  broadcastStartTransform();
}

void TrajectoryUI::targetposeSliderCallback(int id)
{
  // see startpose_slider_callback(int id)
  double new_val = target_pose_panel_.sliders_[id]->value() / (double)target_pose_panel_.multiplier;
  target_pose_panel_.pose_values_[id]->setValue(new_val);
  target_pose_[id] = new_val;
  broadcastTargetTransform();
}

void TrajectoryUI::startposeSpinbCallback(int id)
{
  // retrieve current value from spin box and apply it to corresponding slider
  double new_val = start_pose_panel_.pose_values_[id]->value();
  start_pose_panel_.sliders_[id]->setValue((int)(start_pose_panel_.multiplier * new_val));
  start_pose_[id] = new_val;
  broadcastStartTransform();
}

void TrajectoryUI::targetposeSpinbCallback(int id)
{
  // retrieve current value from spin box and apply it to corresponding slider
  double new_val = target_pose_panel_.pose_values_[id]->value();
  target_pose_panel_.sliders_[id]->setValue((int)(target_pose_panel_.multiplier * new_val));
  target_pose_[id] = new_val;
  broadcastTargetTransform();
}

void TrajectoryUI::durationSliderCallback(int new_val)
{
  // divide slider value by common multiplier to retrieve it as double
  duration_ = (double)new_val / (double)start_pose_panel_.multiplier;
  dur_value_->setValue(duration_);

  // trigger the trajectory designer to calculate a new trajectory
  broadcastTargetTransform();
}

void TrajectoryUI::durationSpinbCallback(double new_val)
{
  // store new duration and set slider accordingly
  duration_ = new_val;
  dur_slider_->setValue(start_pose_panel_.multiplier * duration_);

  // trigger the trajectory designer to calculate a new trajectory
  broadcastTargetTransform();
}

void TrajectoryUI::getTrajectorySetup(Eigen::Vector6f &start_pose, Eigen::Vector6f &target_pose, double &duration)
{
  // Write the references
  start_pose = start_pose_;
  target_pose = target_pose_;
  duration = duration_;
}

void TrajectoryUI::resetPoseConfigurations()
{
  log("Reset Pose Configurations");
  for (int i = 0; i < 6; i++)
  {
    start_pose_panel_.pose_values_[i]->setValue(0);
    target_pose_panel_.pose_values_[i]->setValue(0);
  }

  // set all pose components to zero except target z-Position to 10m
  target_pose_panel_.pose_values_[2]->setValue(10);

  // initial duration is 5 seconds
  dur_value_->setValue(5);
}

void TrajectoryUI::alignStartDronePose()
{
  // listen to transforms between /base_link and /world to retrieve drone's current pose
  tf::StampedTransform baselink_tf;
  try
  {
    baselink_listener_.lookupTransform("/world", "/base_link", ros::Time(0), baselink_tf);

    // apply origin of transform to current start pose's position
    tf::Vector3 orig = baselink_tf.getOrigin();
    start_pose_panel_.pose_values_[0]->setValue(orig.getX());
    start_pose_panel_.pose_values_[1]->setValue(orig.getY());
    start_pose_panel_.pose_values_[2]->setValue(orig.getZ());

    // apply roll pitch yaw angles to current start pose's orientation
    tf::Quaternion q = baselink_tf.getRotation();
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    start_pose_panel_.pose_values_[3]->setValue(roll * 180 / M_PI);
    start_pose_panel_.pose_values_[4]->setValue(pitch * 180 / M_PI);
    start_pose_panel_.pose_values_[5]->setValue(yaw * 180 / M_PI);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    log("Couldn't retrieve current pose of the hexacopter. Ensure the transform between /world and /baselink frames "
        "being broadcasted.");
  }
}

void TrajectoryUI::setTargetPose(Eigen::Vector6f target_pose)
{
  // update locally stored target pose
  target_pose_ = target_pose;

  // update ui elements
  for (size_t i = 0; i < 6; i++)
  {
    target_pose_panel_.pose_values_[i]->setValue(target_pose[i]);
  }

  // broadcast transform to new target position
  broadcastTargetTransform();
}

void TrajectoryUI::setDuration(double duration)
{
  // update ui elements
  dur_slider_->setValue(duration * (double)start_pose_panel_.multiplier);
  dur_value_->setValue(duration);

  // update locally stored duration
  duration_ = duration;
}

void TrajectoryUI::calculateFeasibleTrajectory()
{
  log("Calculating feasible trajectory");
  Q_EMIT makeFeasible();
}

void TrajectoryUI::startTrajectoryTracking()
{
  ROS_INFO("Start Tracking");
  Q_EMIT startTracking(true);
  log("Finished Trajectory Tracking");
}

void TrajectoryUI::broadcastStartTransform()
{
  // convert attitude to rad
  Eigen::Vector3f start_ori_rad = start_pose_.tail(3) * M_PI / 180.0f;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(start_pose_[0], start_pose_[1], start_pose_[2]));
  tf::Quaternion q;
  q.setRPY(start_ori_rad[0], start_ori_rad[1], start_ori_rad[2]);
  transform.setRotation(q);
  start_frame_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "start_pose"));
}

void TrajectoryUI::broadcastTargetTransform()
{
  // convert attitude to rad
  Eigen::Vector3f target_ori_rad = target_pose_.tail(3) * M_PI / 180.0f;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(target_pose_[0], target_pose_[1], target_pose_[2]));
  tf::Quaternion q;
  q.setRPY(target_ori_rad[0], target_ori_rad[1], target_ori_rad[2]);
  transform.setRotation(q);
  target_frame_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "target_pose"));
}

void TrajectoryUI::log(QString message)
{
  // color previous entry gray and append new message
  log_panel_->setTextColor(QColor(150, 150, 150));
  log_panel_->setText(log_panel_->toPlainText());
  log_panel_->setTextColor(Qt::black);
  log_panel_->append(message);

  // scroll to the very bottom
  QTextCursor c = log_panel_->textCursor();
  c.movePosition(QTextCursor::End);
  log_panel_->setTextCursor(c);
}

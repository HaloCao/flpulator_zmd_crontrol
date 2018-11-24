/*
 * @todo Description
 * @author Nils Dunkelberg
 */

#ifndef TRAJECTORYUI_H
#define TRAJECTORYUI_H

#include <QWidget>
#include <QVBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSignalMapper>
#include <QTextEdit>

#include <Eigen/Dense>

#include <ros/ros.h>

/**
 * \struct Structure which holds the graphic components of a pose panel to set start and target pose.
 */
struct PosePanel
{
  QString title_;                   ///< The title of the group box containing the pose panel.
  QLabel *pose_labels_[6];          ///< The labels describing each component of a 6D pose.
  QSlider *sliders_[6];             ///< The sliders to change a pose component.
  QDoubleSpinBox *pose_values_[6];  ///< The labels holding the current value and unit of a pose component.
  int multiplier = 100;  ///< The factor by which the slider value is multiplied w.r.t. the actual pose component value.
};

/**
 * \typedef Vector6f Eigen/Matrix which holds a 6D-Pose (x,y,z,r,p,y).
 */
namespace Eigen
{
typedef Eigen::Matrix<float, 6, 1> Vector6f;
}

class TrajectoryUI : public QWidget
{
  Q_OBJECT

public:
  TrajectoryUI(QWidget *parent = nullptr);

  /**
   * \brief getTrajectorySetup Creates references to the trajectories set-up.
   * Used by parent widget to retrieve the trajectory setup and call the generator service.
   * \param start_pose Reference to the trajectory's start pose
   * \param target_pose Reference to the trajectory's target pose
   * \param duration Reference to the trajectory's duration
   */
  void getTrajectorySetup(Eigen::Vector6f &start_pose, Eigen::Vector6f &target_pose, double &duration);

public:
Q_SIGNALS:

  /**
   * \brief poseUpdate Informs trajectory designer about manipulation of start or target pose.
   */
  void poseUpdate();

protected:
  /**
   * \brief Creates a group box with specific ui tools to set start and target pose of trajectory.
   * \param pose_panel Reference to the corresponding pose panel structure
   * \param slide_mapper Reference to the QSignalMapper which maps each slider signal to one generic slider callback
   * \param spin_mapper Reference to the QSignalMapper which maps each spin box signal to one generic spinbox callback
   */
  void initPosePanel(PosePanel &pose_panel, QSignalMapper &slide_mapper, QSignalMapper &spin_mapper);

  /**
   *\brief Creates a pair of a slider and a spin box next to it, displaying the current values.
   * \param slider Reference to the slider which properties get edited
   * \param spinbox Reference to the spinbox, holding the current value of the slider
   * \param unit Character specifying the unit of the corresponding pose component
   * \param multiplier Factor by which the slider values are multiplied w.r.t. the actual pose component values
   * \param maximum Maximum of the absolute slider value
   */
  void initPoseCompSlider(QSlider &slider, QDoubleSpinBox &spinbox, QString unit, int multiplier, double maximum);

  /**
   *\brief Creates a group box with specific tools to set the desired flight time for a trajectory
   */
  void initFlightTimePanel();

private Q_SLOTS:

  /**
   * \brief Callback for any slider of the start pose panel.
   * Updates the unit label corresponding to active slider and stores current slider value.
   * \param id Index of the active slider within the PosePanel structure of the start pose.
   */
  void startposeSliderCallback(int id);

  /**
   * \brief Callback for any slider of the target pose panel.
   * Updates the unit label corresponding to active slider and stores current slider value.
   * \param id Index of the active slider within the PosePanel structure of the target pose.
   */
  void targetposeSliderCallback(int id);

  /**
   * \brief Callback for the slider of the flight time panel.
   * Updates the unit label corresponding to the slider and stores current slider value.
   * \param new_val Current value of the corresponding slider
   */
  void durationSliderCallback(int new_val);

  /**
   * \brief Callback for any spin box of the start pose panel.
   * Updates the slider value corresponding to active spin box and stores current value.
   * \param id Index of the active spin box within the PosePanel structure of the start pose.
   */
  void startposeSpinbCallback(int id);

  /**
   * \brief Callback for any spin box of the target pose panel.
   * Updates the slider value corresponding to active spin box and stores current value.
   * \param id Index of the active spin box within the PosePanel structure of the target pose.
   */
  void targetposeSpinbCallback(int id);

  /**
   * \brief Callback for the spin box of the flight time panel.
   * Updates the slider value corresponding to the spin box and stores current value.
   * \param new_val Current value of the corresponding spin box.
   */
  void durationSpinbCallback(double new_val);

  /**
   * \brief Callback to reset the start and target pose to initial configuration.
   */
  void resetPoseConfigurations();

  /**
   * \brief Callback to calculate trajectory for given pose-configurations and execute it.
   */
  void startTrajectoryTracking();

private:
  QVBoxLayout *main_layout_;  ///< The main layout of the trajectory-ui-widget

  PosePanel start_pose_panel_;   ///< Structure containing the graphical components of the start pose panel
  PosePanel target_pose_panel_;  ///< Structure containing the graphical components of the target pose panel

  QTextEdit *log_panel_;  ///< Displays log messages delivering information about system status.

  QSignalMapper *start_slide_mapper_;   ///< Dynamically maps every slider signal of a pose panel to one slider callback
  QSignalMapper *target_slide_mapper_;  ///< Dynamically maps every slider signal of a pose panel to one slider callback
  QSignalMapper *start_spin_mapper_;    ///< Dynamically maps every spin box signal of a pose panel to one callback
  QSignalMapper *target_spin_mapper_;   ///< Dynamically maps every spin box signal of a pose panel to one callback

  double duration_;            ///< The desired flight time of trajectory tracking
  QDoubleSpinBox *dur_value_;  ///< Spinbox holding the current value of the desired flight time
  QSlider *dur_slider_;        ///< QSlider for setting the desired flight time

  Eigen::Vector6f start_pose_;   ///< 6D-Vector holding the components of the start pose.
  Eigen::Vector6f target_pose_;  ///< 6D-Vector holding the components of the target pose
};

#endif  // TRAJECTORYUI_H

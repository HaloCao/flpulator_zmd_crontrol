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

#include <ros/ros.h>

/**
 * \struct Structure which holds the graphic components of a pose panel to set start and target pose.
 */
struct PosePanel {
    QString title_;                  ///< The title of the group box containing the pose panel.
    QLabel *pose_labels_[6];         ///< The labels describing each component of a 6D pose.
    QSlider *sliders_[6];            ///< The sliders to change a pose component.
    QDoubleSpinBox *pose_values_[6]; ///< The labels holding the current value and unit of a pose component.
    int multiplier = 100;            ///< The factor by which the slider value is multiplied w.r.t. the actual pose component value

};

class TrajectoryUI : public QWidget
{
  Q_OBJECT
public:
  TrajectoryUI(QWidget* parent = nullptr);

protected:
  /**
   * \brief Creates a group box with specific ui tools to set start and target pose of trajectory.
   * \param pose_panel Reference to the corresponding pose panel structure
   */
  void initPosePanel(PosePanel &pose_panel, QSignalMapper &mapper);

private Q_SLOTS:
  /**
   * \brief Callback for any slider of the start pose panel.
   * Updates the unit label corresponding to active slider and stores current slider value.
   * \param slider_id Index of the active slider within the PosePanel structure of the start pose.
   */
  void startpose_slider_callback(int slider_id);

  /**
   * \brief Callback for any slider of the target pose panel.
   * Updates the unit label corresponding to active slider and stores current slider value.
   * \param id Index of the active slider within the PosePanel structure of the target pose.
   */
  void targetpose_slider_callback(int id);

private:

  ros::NodeHandle nh_;                  ///< Interface to register standard ros components

  QVBoxLayout *main_layout_;            ///< The main layout of the trajectory-ui-widget

  PosePanel start_pose_panel_;          ///< Structure containing the graphical components of the start pose panel
  PosePanel target_pose_panel_;         ///< Structure containing the graphical components of the target pose panel

  QSignalMapper *start_pose_mapper_;    ///< Dynamically maps every slider signal of a pose panel to one slider callback
  QSignalMapper *target_pose_mapper_;   ///< Dynamically maps every slider signal of a pose panel to one slider callback

};

#endif  // TRAJECTORYUI_H

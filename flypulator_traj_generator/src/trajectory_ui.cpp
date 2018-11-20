/*
 * @todo Description
 * @author Nils Dunkelberg
 */

#include <QApplication>
#include <QGridLayout>
#include <QGroupBox>
#include <QSpacerItem>

#include "ros/ros.h"

#include "trajectory_ui.h"

TrajectoryUI::TrajectoryUI(QWidget *parent)
    : QWidget(parent),
      start_pose_mapper_ (new QSignalMapper(this)),
      target_pose_mapper_ (new QSignalMapper(this))

{
    // main layout of the trajectory user interface
    main_layout_ = new QVBoxLayout();

    // set title of pose panel structure to set user interface title dynamically
    start_pose_panel_.title_ = "Start Pose";
    target_pose_panel_.title_ = "Target Pose";

    // add panels with interface for setting start and target pose of the desired trajectory
    initPosePanel(start_pose_panel_, *start_pose_mapper_);
    initPosePanel(target_pose_panel_, *target_pose_mapper_);

    // connect the signal mapper to the slider callbacks
    connect(start_pose_mapper_, SIGNAL(mapped (int)), this, SLOT( startpose_slider_callback(int)));
    connect(target_pose_mapper_, SIGNAL(mapped (int)), this, SLOT( targetpose_slider_callback(int)));

    // apply vertical spacer to push items to the top
    QSpacerItem *v_spacer = new QSpacerItem(10, 100, QSizePolicy::Minimum, QSizePolicy::Expanding);
    main_layout_->addSpacerItem(v_spacer);

    setLayout(main_layout_);
}

void TrajectoryUI::initPosePanel(PosePanel &pose_panel, QSignalMapper &mapper) {
    // store start pose panel in a groupbox
    QGroupBox *pose_panel_gbox = new QGroupBox(pose_panel.title_);

    // grid layout of the group box
    QGridLayout *gbox_layout = new QGridLayout();

    // Define the label of the pose components and its units
    QString pose_ids[6] = {"x", "y", "z", QChar(0xC6, 0x03), QChar(0x98, 0x03), QChar(0xA8, 0x03)};
    QString pose_units[6] = {" m", " m", " m", " °", " °", " °"};

    // Define the maximum absolute values of each pose component
    // (multiplied by 100 to obtain higher precision after double conversion)
    int max_pose[6] = { 5000, 5000, 5000, 1500, 1500, 18000};

    // for each pose component apply label, slider and its unit
    for (int i = 0; i < 6; i++) {
        // intialize slider and its properties
        pose_panel.sliders_[i] = new QSlider(Qt::Horizontal, this);
        pose_panel.sliders_[i]->setRange(-max_pose[i], max_pose[i]);
        pose_panel.sliders_[i]->setTickInterval(max_pose[i] / 5);
        pose_panel.sliders_[i]->setTickPosition(QSlider::TicksBelow);

        // initialize pose component label
        pose_panel.pose_labels_[i] = new QLabel(pose_ids[i]);

        // initialize the spinbox containing the current pose component's value and unit
        pose_panel.pose_values_[i] = new QDoubleSpinBox();
        pose_panel.pose_values_[i]->setFixedWidth(80);
        pose_panel.pose_values_[i]->setButtonSymbols(QAbstractSpinBox::NoButtons);
        pose_panel.pose_values_[i]->setSuffix(pose_units[i]);
        pose_panel.pose_values_[i]->setMaximum(max_pose[i]);
        pose_panel.pose_values_[i]->setMinimum(-max_pose[i]);
        pose_panel.pose_values_[i]->setDecimals(2);
        pose_panel.pose_values_[i]->setAlignment(Qt::AlignRight);
        pose_panel.pose_values_[i]->setValue(pose_panel.sliders_[i]->value() / (double) pose_panel.multiplier);

        // dynamically connect each slider's valueChanged signal to the signal mapper's mapping-Slot
        // everytime a slider is manipulated, the corresponding slider callback is triggered and given the index of the active slider
        connect(pose_panel.sliders_[i], SIGNAL(valueChanged(int)), &mapper, SLOT(map()));
        mapper.setMapping(pose_panel.sliders_[i], i);

        //for each pose component add all required widgets as a new row to the groupbox (grid) layout
        gbox_layout->addWidget(pose_panel.pose_labels_[i], i, 0);
        gbox_layout->addWidget(pose_panel.sliders_[i], i, 1);
        gbox_layout->addWidget(pose_panel.pose_values_[i], i, 2);
    }

    // apply newly created layout to the pose panel group box and the pose panel to the main layout
    pose_panel_gbox->setLayout(gbox_layout);
    main_layout_->addWidget(pose_panel_gbox);

}

void TrajectoryUI::startpose_slider_callback(int id) {
    // since qslider do not support doubles per default, they hold each value multiplied by a constant factor
    // to obtain higher precision of the actual pose component values
    double cur_val = start_pose_panel_.sliders_[id]->value() / (double) start_pose_panel_.multiplier;
    start_pose_panel_.pose_values_[id]->setValue(cur_val);
}

void TrajectoryUI::targetpose_slider_callback(int id) {
    // see startpose_slider_callback(int id)
    double cur_val = target_pose_panel_.sliders_[id]->value() / (double) target_pose_panel_.multiplier;
    target_pose_panel_.pose_values_[id]->setValue(cur_val);
}



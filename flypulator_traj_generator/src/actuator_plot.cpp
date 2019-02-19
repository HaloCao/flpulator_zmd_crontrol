/*
 * @author Nils Dunkelberg
 */

#include <QApplication>
#include <QVBoxLayout>
#include <QVector>

#include "ros/ros.h"

#include "flypulator_traj_generator/actuator_plot.h"

ActuatorPlot::ActuatorPlot(QWidget *parent) : QWidget(parent), actuator_plot_(new QCustomPlot(this))
{
  // set main layout, containing the actuator plotting widget
  QVBoxLayout *main_layout = new QVBoxLayout();

  main_layout->addWidget(actuator_plot_);

  setLayout(main_layout);

  // retrieve actuator velocity boundaries
  ros::param::param<double>("/trajectory/rotor_vel_max", upper_vel_limit_, 5700);
  ros::param::param<double>("/trajectory/rotor_vel_min", lower_vel_limit_, 0);

  // label axes
  actuator_plot_->xAxis->setLabel("Time [s]");
  actuator_plot_->yAxis->setLabel("Rotor velocities [rpm]");
  // set axes range
  actuator_plot_->yAxis->setRange(-2000, 7000);

  // create six empty graphs for the rotor velocities
  // colors of the graphs
  QColor pen_colors[6] = {QColor(0, 114, 189),  QColor(217, 83, 25),  QColor(237, 178, 32),
                          QColor(126, 47, 142), QColor(119, 172, 48), QColor(77, 190, 238)};
  QPen pen;
  pen.setWidth(2);

  for (int i = 0; i < 6; i++)
  {
    // add graph
    actuator_plot_->addGraph();
    actuator_plot_->graph(i)->setName(QString((QChar)0x3C9) + QString::number(i + 1));

    // set style
    pen.setColor(pen_colors[i]);
    actuator_plot_->graph(i)->setPen(pen);
  }

  // show legend (horizontal fill order)
  actuator_plot_->legend->setVisible(true);
  actuator_plot_->legend->setFillOrder(QCPLegend::foColumnsFirst);
  //  actuator_plot_->plotLayout()->setRowStretchFactor(1, 0.001);

  // create constant horizontal lines to visualize the actuator velocity boundaries
  upper_limit_line_ = new QCPItemLine(actuator_plot_);
  lower_limit_line_ = new QCPItemLine(actuator_plot_);
  QPen limit_pen(QColor(255, 0, 0), 1.0, Qt::DashLine);
  upper_limit_line_->setPen(limit_pen);
  lower_limit_line_->setPen(limit_pen);
}

void ActuatorPlot::plotActuatorBoundaries(double t_end)
{
  // update upper limit coordinates
  upper_limit_line_->start->setCoords(0, upper_vel_limit_);
  upper_limit_line_->end->setCoords(t_end, upper_vel_limit_);

  // update lower limit coordinates
  lower_limit_line_->start->setCoords(0, lower_vel_limit_);
  lower_limit_line_->end->setCoords(t_end, lower_vel_limit_);
}

void ActuatorPlot::updateActuatorBoundaries(float upper_limit, float lower_limit)
{
  // update actuator boundaries
  if (upper_limit <= lower_limit)
  {
    return;
  }

  upper_vel_limit_ = upper_limit;
  lower_vel_limit_ = lower_limit;
}

void ActuatorPlot::plotActuatorEvolution(trajectory::rotor_velocities_rpm &rotor_velocities,
                                         QVector<double> &time_stamps, bool feasible)
{
  // update time axis range
  double t_end = time_stamps[time_stamps.size() - 1];
  actuator_plot_->xAxis->setRange(0, t_end);

  // apply data
  for (int i = 0; i < 6; i++)
  {
    actuator_plot_->graph(i)->setData(time_stamps, rotor_velocities[i], true);
  }

  // update the actuator boundary constants
  plotActuatorBoundaries(t_end);

  // brush background light red if actuator boundaries exceeded
  QColor background_col = feasible ? Qt::white : QColor(255, 210, 210);
  actuator_plot_->setBackground(background_col);

  // update custom plot
  actuator_plot_->replot();
}

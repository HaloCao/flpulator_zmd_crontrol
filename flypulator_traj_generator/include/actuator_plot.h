/*
 * @author Nils Dunkelberg
 */

#ifndef ACTUATORPLOT_H
#define ACTUATORPLOT_H

#include <QWidget>
#include <QVector>

#include "qcustomplot.h"

namespace trajectory
{
/**
 *\typedef RotorEvolution Vector holding the rotor velocities of the hexacopter over time
 */
typedef std::vector<QVector<double>> RotorEvolution;
}  // namespace trajectory

/**
 * \class The ActuatorPlot class
 * \brief The ActuatorPlot class is responsible for plotting the retrieved course of the rotor's velocities. Furthermore
 * it plots the upper and lower limit of feasible actuator values and highlights a red background in case of
 * infeasibility. The plotting is based on the open source QCustomPlot widget.
 */
class ActuatorPlot : public QWidget
{
  Q_OBJECT

public:
  ActuatorPlot(QWidget *parent = nullptr);

  /**
   * \brief plotActuatorEvolution Plots the current evolution of the hexacopter propellers to the custom plot widget.
   * \param rotor_velocities Reference to the evolution of retrieved rotor velocities over time
   * \param time_steps Reference to the time stamps of each sampling point
   * \param feasible True if given trajectory doesn't exceed actuator boundaries
   */
  void plotActuatorEvolution(trajectory::RotorEvolution &rotor_velocities, QVector<double> &time_stamps, bool feasible);

protected:
  /**
   * \brief plotActuatorBoundaries draws the upper and lower limit of the permitted rotor velocities as constants.
   * Needs to be updated, if the duration of the trajectory (hence the range of x-Axis) changes.
   * \param t_end The ending time stamp of the trajectory
   */
  void plotActuatorBoundaries(double t_end);

private Q_SLOTS:

private:
  QCustomPlot *actuator_plot_;  ///< Widget for plotting the rotor velocities

  QCPItemLine *upper_limit_line_;  ///< Constant line to visualize the upper limit of actuator velocity space.
  QCPItemLine *lower_limit_line_;  ///< Constant line to visualize the lower limit of actuator velocity space.

  double upper_vel_limit_;  ///< The maximum feasible rotational velocity of a propeller.
  double lower_vel_limit_;  ///< The minimum feasible rotational velocity of a propeller.
};

#endif  // ACTUATORPLOT_H

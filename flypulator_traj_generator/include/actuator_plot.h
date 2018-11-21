/*
 * @todo Description
 * @author Nils Dunkelberg
 */

#ifndef ACTUATORPLOT_H
#define ACTUATORPLOT_H

#include <QWidget>

#include "qcustomplot.h"

class ActuatorPlot : public QWidget
{
  Q_OBJECT

public:
  ActuatorPlot(QWidget *parent = nullptr);


protected:
  /**
   * \brief plotActuatorEvolution Plots the current evolution of the hexacopter propellers to the custom plot widget.
   */
  void plotActuatorEvolution();

private Q_SLOTS:

private:
  QCustomPlot *actuator_plot_;

};

#endif  // ACTUATORPLOT_H

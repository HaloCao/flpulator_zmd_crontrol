/*
 * @todo Description
 * @author Nils Dunkelberg
 */

#include <QApplication>
#include <QVBoxLayout>
#include <QVector>

#include "ros/ros.h"

#include "actuator_plot.h"

ActuatorPlot::ActuatorPlot(QWidget *parent)
  : QWidget(parent),
    actuator_plot_ (new QCustomPlot(this))
{
    QVBoxLayout *main_layout = new QVBoxLayout();

    main_layout->addWidget(actuator_plot_);

    setLayout(main_layout);

    plotActuatorEvolution();
}

void ActuatorPlot::plotActuatorEvolution()
{
    QVector<double> x(101), y(101); // initialize with entries 0..100
    for (int i=0; i<101; ++i)
    {
      x[i] = i/50.0 - 1; // x goes from -1 to 1
      y[i] = x[i]*x[i]; // let's plot a quadratic function
    }
    // create graph and assign data to it:
    actuator_plot_->addGraph();
    actuator_plot_->graph(0)->setData(x, y);
    // give the axes some labels:
    actuator_plot_->xAxis->setLabel("x");
    actuator_plot_->yAxis->setLabel("y");
    // set axes ranges, so we see all data:
    actuator_plot_->xAxis->setRange(-1, 1);
    actuator_plot_->yAxis->setRange(0, 1);
    actuator_plot_->replot();
}

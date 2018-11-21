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
    // set main layout, containing the actuator plotting widget
    QVBoxLayout *main_layout = new QVBoxLayout();

    main_layout->addWidget(actuator_plot_);

    setLayout(main_layout);

    // draw initial data
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
    // create a new graph
    actuator_plot_->addGraph();
    actuator_plot_->graph(0)->setData(x, y);
    // label axes
    actuator_plot_->xAxis->setLabel("Time [s]");
    actuator_plot_->yAxis->setLabel("Rotational Speed [rpm]");
    // set axes range
    actuator_plot_->xAxis->setRange(0, 5);
    actuator_plot_->yAxis->setRange(-5000, 7000);
    actuator_plot_->replot();
}

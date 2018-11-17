#ifndef OFFBOARD_INTERFACE_H
#define OFFBOARD_INTERFACE_H

//include standard dependencies
#include <ros/ros.h>
#include <stdlib.h>
#include <vector>
//include for dynamic reconfigure
#include <flypulator_physical/offb_parameterConfig.h>
#include <dynamic_reconfigure/server.h>
//include message structs
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/State.h>
#include <flypulator_common_msgs/RotorVelStamped.h>


class OffboardInterface 
{
public:
    OffboardInterface();

private:
    //read parameters from server to scale correctly
    void readDroneParametersFromServer();

    //scale controller output to actuator_control msgs
    /* float scaleControlOutputToActuators(float controller_output); */


        

};
#endif //OFFBOARD_INTERFACE_H

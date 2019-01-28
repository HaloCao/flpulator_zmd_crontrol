/*
 * @author Nils Dunkelberg
 */

#include "feasibility_check.h"

FeasibilityCheck::FeasibilityCheck(ActuatorSimulation* actuator_simulation)
    : actuator_simulation_(actuator_simulation)
{
}

bool FeasibilityCheck::makeFeasible(Eigen::Vector6f &start_pose, Eigen::Vector6f &target_pose, double &duration)
{
    // check start pose feasibility. If not feasible, user has to modify it.
    if (!actuator_simulation_->isFeasible(start_pose))
    {
        ROS_INFO("Start pose not feasible. Please adjust!");
        return false;
    }
    // check target pose feasibility. If not feasible, calculate closest feasible alternative.
    if (!actuator_simulation_->isFeasible(target_pose))
    {
        ROS_INFO("Target pose not feasible. Retrieving feasible alternative...");
        retrieveFeasibleEndpose(target_pose);
    }
}

void FeasibilityCheck::retrieveFeasibleEndpose(Eigen::Vector6f &target_pose)
{


}

/*
 * @author Nils Dunkelberg
 */

#include "feasibility_check.h"

FeasibilityCheck::FeasibilityCheck(ActuatorSimulation* actuator_simulation)
    : actuator_simulation_(actuator_simulation)
{
    // load required simulation parameters
    ros::param::param<float>("/trajectory/newton_stepsize", newt_stepsize_, 0.01f);
    ros::param::param<double>("/trajectory/newton_stepsize", newt_epsilon_, 10);
    ros::param::param<double>("/trajectory/boundary_buffer", rotvel_buffer_, 150);

    // actuator velocity boundaries
    ros::param::param<double>("/uav/rotor_vel_max", upper_vel_limit_, 5700);
    ros::param::param<double>("/uav/rotor_vel_min", lower_vel_limit_, 0);

    //store actuator boundaries as rad²/s² as well
    upper_vel_limit_squ_ = pow(upper_vel_limit_ * M_PI / 30, 2);
    lower_vel_limit_squ_ = pow(lower_vel_limit_ * M_PI / 30, 2);

}

bool FeasibilityCheck::makeFeasible(Eigen::Vector6f &start_pose, Eigen::Vector6f &target_pose, double &duration)
{
    // retrieve squared rotor velocities for start and target pose
    Eigen::Vector6f rot_vel_squ_start = actuator_simulation_->getSteadyStateRotorVelocities(start_pose);
    Eigen::Vector6f rot_vel_squ_target = actuator_simulation_->getSteadyStateRotorVelocities(target_pose);

    // check start pose feasibility. If not feasible, user has to modify it.
    if (!isFeasible(rot_vel_squ_start))
    {
        ROS_INFO("Start pose not feasible. Please adjust!");
        return false;
    }

    // check target pose feasibility. If not feasible, calculate closest feasible alternative.
    if (!isFeasible(rot_vel_squ_target))
    {
        ROS_INFO("Target pose not feasible. Retrieving feasible alternative...");
        retrieveFeasibleEndpose(target_pose, rot_vel_squ_target);
    }

    // check if trajectory is feasible
    //callTrajectoryGenerator();

}

bool FeasibilityCheck::isFeasible(Eigen::Vector6f rotor_velocities)
{
    // retrieve maximum and minimum rotor squared velocities
    double rot_vel_min = rotor_velocities.minCoeff();
    double rot_vel_max = rotor_velocities.maxCoeff();

    // feasibility check
    return rot_vel_min >= lower_vel_limit_squ_ && rot_vel_max <= upper_vel_limit_squ_;
}

void FeasibilityCheck::retrieveFeasibleEndpose(Eigen::Vector6f &target_pose, Eigen::Vector6f initial_rotor_velocities)
{
    //######### setup ###################
    // retrieve the squared rotor velocities needed for horizontal hovering (zero orientation)
    Eigen::Vector6f steady_state_rotvels = actuator_simulation_->getSteadyStateRotorVelocities(Eigen::Vector6f::Zero());
    double steady_state_rotvel =steady_state_rotvels[0];

    // set updated actuator limits (decreased/increased by buffer velocity) and converted to rad²/s²
    double ulimit = pow((upper_vel_limit_ - rotvel_buffer_) * M_PI / 30, 2);
    double llimit = pow((lower_vel_limit_ + rotvel_buffer_) * M_PI / 30, 2);

    // since yaw angle has no influence on euler angles, set it to zero
    Eigen::Vector6f target_pose_tmp = target_pose;
    target_pose_tmp[5] = 0;

    // retrieve euler parameters for target pose
    Eigen::Vector3f euler_axis;
    double euler_angle;
    actuator_simulation_->poseToEulerParams(target_pose_tmp, euler_axis, euler_angle);

    // retrieve rotor indices with the maximum and minimum velocities#
    int i_max;
    int i_min;
    initial_rotor_velocities.maxCoeff(&i_max);
    initial_rotor_velocities.minCoeff(&i_min);

    //######## newton approach ############
    double feasible_upper_euler_angle = findFeasbileEulerAngle(steady_state_rotvel, ulimit, euler_axis, i_max);
    double feasible_lower_euler_angle = findFeasbileEulerAngle(steady_state_rotvel, llimit, euler_axis, i_min);

    double feasible_euler_angle = std::min(feasible_upper_euler_angle, feasible_lower_euler_angle);

    //######### result #####################
    // apply new target pose orientation to the original target pose (only roll and pitch angle considered)
    Eigen::Vector3f rpy_angles = actuator_simulation_->eulerParamsToYPR(euler_axis, feasible_euler_angle);
    target_pose.segment(3,2) = rpy_angles.head(2); //segment (starting at coefficient 3, containing 2 successive entrys (roll and pitch))
}

double FeasibilityCheck::findFeasbileEulerAngle(double rotvel_init, double rotvel_limit, Eigen::Vector3f euler_axis, int index)
{
    double rotvel_k = rotvel_init; // holds the resulting rotor velocities after each step of the newton approach
    double theta_k = 0; // holds the corresponding euler angle of each step which will be optimized
    double newt_epsilon = pow(newt_epsilon_ * M_PI / 10, 2); // terminating condition per rad²/s²

    //loop until terminating condition is met
    while (abs(rotvel_limit - rotvel_k) > newt_epsilon) {
        //derivative at current euler angle
        double theta_tmp = theta_k + newt_stepsize_;
        Eigen::Vector6f rotvel_tmp = actuator_simulation_->getSteadyStateRotorVelocities(euler_axis, theta_tmp);
        double d_rotvel = (rotvel_tmp[index] - rotvel_k) / newt_stepsize_;

        // calculate euler angle and rotor velocity at intersection with actuator limit (corresponds to next step k+1)
        theta_k += (rotvel_limit - rotvel_k) / d_rotvel;
        Eigen::Vector6f rotvels_k = actuator_simulation_->getSteadyStateRotorVelocities(euler_axis, theta_k);
        rotvel_k = rotvels_k[index];
    }
    return theta_k;
}

//void FeasibilityCheck::callTrajectoryGenerator(bool start_tracking)
//{
//  // Create references to retrieve the current trajectory setup from user interface
//  Eigen::Vector6f start_pose;
//  Eigen::Vector6f target_pose;
//  double duration;

//  // create service
//  polynomial_trajectory pt_srv;

//  // fill service parameters
//  pt_srv.request.p_start.x = start_pose[0];
//  pt_srv.request.p_start.y = start_pose[1];
//  pt_srv.request.p_start.z = start_pose[2];
//  pt_srv.request.rpy_start.x = start_pose[3];
//  pt_srv.request.rpy_start.y = start_pose[4];
//  pt_srv.request.rpy_start.z = start_pose[5];

//  pt_srv.request.p_end.x = target_pose[0];
//  pt_srv.request.p_end.y = target_pose[1];
//  pt_srv.request.p_end.z = target_pose[2];
//  pt_srv.request.rpy_end.x = target_pose[3];
//  pt_srv.request.rpy_end.y = target_pose[4];
//  pt_srv.request.rpy_end.z = target_pose[5];

//  pt_srv.request.duration = duration;

//  pt_srv.request.start_tracking = start_tracking;

//  if (!polynomial_traj_client_.call(pt_srv))
//  {
//    ROS_ERROR("[flypulator_traj_generator] Failed to call polynomial trajectory service.");
//    return;
//  }

//  // Create Vector to store rotor velocity evolution
//  size_t s = pt_srv.response.p_acc.size();
//  trajectory::rotor_velocities_rpm rotor_velocities(6, QVector<double>(s));

//  // boolean to check feasibility
//  bool feasible = true;

//  // simulate the rotor velocities for the current trajectory
//  actuator_simulation_->simulateActuatorVelocities(start_pose, pt_srv.response.p_acc, pt_srv.response.euler_angle_acc,
//                                                   pt_srv.response.euler_angle, pt_srv.response.euler_axis,
//                                                   rotor_velocities, feasible);

//  // convert timestamps to qvector
//  QVector<double> time_stamps = QVector<double>::fromStdVector(pt_srv.response.time_stamps);
//}



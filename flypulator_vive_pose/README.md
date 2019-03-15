# flupulator_vive_pose
publish pose, velocity and acceleration of the flypulator, w.r.t. the inital coordinate.
## Subscribe topics
**/vive/LHR_08DDEDC9_odom**  
TYPE: nav_msgs::odometry  
NOTE: publish by **vive_tracker** package.  
## Publish topics
**/flypulator_vive_pose/vive_pose**  
TYPE: geometry_msgs::PoseStamped  
NOTE: stamped pose of the tracker w.r.t. the inital coordinate when this node starts.  

**/flypulator_vive_pose/vive_uav_state**  
TYPE: flypulator_common_msgs::UavStateStamped  
NOTE: stamped pose, linear/angular velocity and acceleration of the tracker w.r.t. the inital coordinate when this node starts.  
## Publish tf
* tf: parent: "**flypulator_world**" ---> child: "**flypulator_tracker**"  
* tf: parent: "**vive_world**" ---> child: "**flypulator_world**"


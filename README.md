# Flypulator Project

## Prerequisites

### System requirements
From [gazebosim.org](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1):
 - A dedicated GPU,
   - Nvidia cards tend to work well in Ubuntu
 - A CPU that is at least an Intel I5, or equivalent,
 - At least 500MB of free disk space
 - Ubuntu Trusty or later installed.

### Install ROS and Gazebo
To install ROS, follow the instructions of [Ros Wiki](http://www.ros.org/). It is highly recommended to use Ubuntu based linux OS. Choose to install full version of ROS (for Ubuntu, [Step 1.4: sudo apt-get install ros-[your ros version]]-desktop-full](http://wiki.ros.org/melodic/Installation/Ubuntu#Installation-1)). Gazebo 9 is already included in this package.

### Build
The content of this repository must be cloned into the `/src` folder of a catkin workspace ([how to create an empty workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)). For an existing workspace in `~/catkin_ws` that requires the following steps (in a new terminal):

```
cd ~/your_ws/src/
git clone https://github.com/FLYing-maniPULATOR/flypulator.git
cd ..
catkin_make
```
Dont forget sourcing the setup.bash file:
```
source ~/your_ws/devel/setup.bash
```
To do sourcing permanently, edit the .bashrc file with `gedit ~/.bashrc` and add the source command from above (`source ~/your_ws/devel/setup.bash`). *Note that you have to start a new terminal to apply the changes*. You can check if it has worked by trying to locate a package using `rospack find flypulator_control`.

## Gazebo simulation

### Launch:  

```
roslaunch flypulator_launch gazebo.launch 
```

includes aerodynamics, fake sensor plugin & parameters.


The model parameters of the hexarotor are defined in the file `drone_parameter_igs.yaml` located at `.../flypulator/flypulator_description/param/`. They will be loaded to ROS parameter server using the provided launchfile `gazebo.launch` as well. A manual loading to the server can be performed using the following command: 

```
rosparam load flypulator_ws/src/flypulator/flypulator_description/param/drone_parameter_igs.yaml
```

### Check tf in rviz: 
roslaunch flypulator_description display.launch

## Controller
The controller package (flypulator_control) provides a sliding mode controller for a fully actuated hexarotor. .

### Start

To start the controller node and load the `control_parameter.yaml`, where e.g. the controller type is defined, a launch file is provided in `flypulator_launch/launch` named `controller.launch`, which can be executed using the following command:

```
roslaunch flypulator_launch controller.launch
```

### Parameters

The control parameters including the distinct rotational and translational gains of the ISM controller can be changed via runtime using the [dynamic_reconfigure package](http://wiki.ros.org/dynamic_reconfigure). The default values are defined in the files `ism_parameter.cfg` located at `flypulator_control/cfg/`. To start the dynamic reconfigure GUI, use the following command:

```
rosrun rqt_reconfigure rqt_reconfigure
```

The new controller parameters are passed to the controller, which also can be watched in console output. Note that the parameters are for an ISM controller; to support additional controllers, they need to implement the BaseController interface/abstract class and the file `control_parameter.cfg` needs to be adapted as well as the ControllerInterface class, where the new controller type has to be registered.

The debug level can be changed in a GUI by running

```
rosrun rqt_logger_level rqt_logger_level ` or simply ` rqt_logger_level
```

## Trajectory Generator
The Trajectory Generator is located within the `flypulator_traj_generator`-package. 

### User Interface
The package provides a graphical user interface to set the start and target pose as well as the duration of a desired trajectory. The corresponding node is called `trajectory_uinterface` and can be launched following the command

```
roslaunch flypulator_launch trajectory_generator.launch 
```

The graphical user interface provides a panel with multiple sliders on the upper left, where the 6D-start and -target poses and the desired duration can be defined. 
Beneath these sliders there are four buttons for further interaction: 
 -  _Reset Poses_: Resets the components of the start and target pose and the duration to default values
 -  _Align to Start Pose_: Aligns the pose components of the start pose of the trajectory to the currently estimated hexarotor's pose
 -  _Find Feasible Trajectory_: Performs a feasibility check - an unfeasbible target pose and/or duration will be adjusted towards feasibility such that the actuator boundaries won't get exeeded
 - _Start Trajectory Tracking_: Calls the ROS-service 'polynomial_trajectories' in order to publish the current trajectory to the controller (see sec. Trajectory Generation Service)
 
Beneath those buttons, there is a log panel, where status messages about the execution state and feasibility check pop up. On the right hand, the `trajectory_uinterface` - node provides a 3D-render window, where the hexacopter model and the start and target pose as well as the current trajectory are visualized. On the bottom left, there is a plotting widget, which shows the expected evolution of the rotor velocities over time. The actuator boundaries and the current feasibility state (white or red plotting background) are visualized as well. Both, the plot and the 3D-render scene can be hidden by opening the file `flypulator_launch/launch/trajectory_generator.launch`, changing the argument `_visualize` to false and relaunching the node. 

### Trajectory Generation Service
The `flypulator_traj_generator`-package contains a second node called `trajectory_generator_server`, which holds ROS-Services in order to calculate polynomial or linear trajectories based on the duration, start and target pose and publish them to the controller. This node is also launched via the `trajectory_generator.launch`-file. As described, the 'polynomial_trajectories'-service is called by the `trajectory_uinterface`-node, when hitting the 'Start Trajectory Tracking' button. Both services may be called manually following the terminal command

```
rosservice call /polynomial_trajectory "p_start: {x: 0.0, y: 0.0, z: 0.0}
p_end: {x: 0.0, y: 0.0, z: 10.0}
rpy_start: {x: 0.0, y: 0.0, z: 0.0}
rpy_end: {x: 0.0, y: 0.0, z: 0.0}
duration: 5.0
start_tracking: true" 
```

Replace '/polynomial_trajectory' with '/linear_trajectory' to obtain linear trajectories. Note, that a manual call won't imply the feasibility check, usually performed by the `trajectory_uinterface`-node.

### Parameters
There is a set of parameters belonging to the `trajectory_uinterface`-node, which are defined in `flypulator_traj_generator/cfg/traj_gen_parameter.yaml`. They cover for instance the minimum and maximum allowed rotor velocities, step sizes for simulation etc. and can be reconfigured via the package _dynamic reconfigure_ (see previous section _Controller_->_Parameters_). The default values and ranges for dynamic_reconfigure are stored in `flypulator_traj_generator/cfg/traj_parameter.cfg`.

## Structure

The trajectory generator publishes [MultiDOFJointTrajectoryPoint Messages](http://docs.ros.org/jade/api/trajectory_msgs/html/msg/MultiDOFJointTrajectoryPoint.html) to the topic 
"/trajectory". The controller suscribes to this topic, and to state estimation messages (*UavStateStamped.msg* from `flypulator_common_msgs`). Every time the controller gets a state estimation message, the control output is calculated using the latest available desired pose. The calculated spinning velocities are published on topic `rotor_cmd` using a *RotorVelStamped* - message from package `flypulator_common_msgs`.
For further information, consider the corresponding diploma thesis (Krieglstein), chapter 6.
The motors feedforward control requires the sampling time of the state estimation as parameter, which must be on ros parameter server as `state_estimation/sampling_time`. This is included in launchfile `gazebo.launch`.

## Code

The code style follows the [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide). Hence, class member variables have a underscore suffix (e.g. `variable1_`). Global variables have a leading `g_` prefix (e.g. `g_variable2`). For performance reasons, functions which are called frequently do not return values, but get a reference on the output passed as argument, so the result can be stored in this reference. This is a commonly used principle in C++.

## Known Issues

 - Sometimes gazebo fails to start. Just exit and start again
 - Same may happen to the trajectory generator GUI. Simply exit and relaunch the node. 
 - If `catkin_make` does not succeed because some header files are missing, just run it twice.
 - Sometimes the drone model is not loaded correctly and there are errors in console. Try log out and log in (Linux) or delete `/build` and `/devel` folder in `~/catkin_ws/` and rerun `catkin_make`.
 - When upgrading ROS version, clean your workspace folder (delete `devel` and `build`) and remove related `source ~/your_ws/devel/setup.bash`, since it still points to old version and causes errors.

## Version Numbering Rules
Given a version number MAJOR.MINOR.PATCH, increment the:

 - MAJOR version when you make incompatible API changes,
 - MINOR version when you add functionality in a backwards-compatible manner, and
 - PATCH version when you make backwards-compatible bug fixes.
 
 Refer to [Semantic Versioning](https://semver.org/) for more details.

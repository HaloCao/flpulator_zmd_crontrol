# flypulator_highlevel_command

## Highlevel control command

### Usage: 

#### with roslauch:
```
roslaunch flypulator_launch gazebo_hexatilt_hl_command.launch
```
OR
#### with rosrun:
Run service:
```
rosrun flypulator_highlevel_command hl_command_server
```
#### Send command:
take off:
```
rosservice call /hl_command_service '{command: take off}'
````
or landing:
```
rosservice call /hl_command_service '{command: landing}'
```


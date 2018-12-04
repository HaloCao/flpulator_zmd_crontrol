# Flypulator Physical
This readme file describes how the Flypulator UAV is operated in the current configuration with Pixhawk Flight controller. The setup requres the interaction with two computers, the Intel NUC on the UAV (aka Flypulator) and your computer (aka Ground PC).
## Prerequisites
Make sure the following software in installed on the Ground PC (tested with Ubuntu 18.04):
- ROS melodic
- Flypulator package in your catkin_ws.
- A VNC client (tested with Vinagre)
```
sudo apt install vinagre
```
Add the following to your /etc/hosts:
10.42.0.1 flypulator
### Connect to Flypulator WiFi and start SSH session
Connect to Flypulator WIFI:
- Network name: flypulator
- Password: 0chK4mGu

Start SSH session:
```
ssh flypulator@flypulator -L 5901:127.0.0.1:5901
PW: IFA (to be changed to ifa)
```
start Tmux:
```
tmux
```
- Make vertical split: ctrl+x v
- Make split: ctrl+x s
- Close split: exit
- Make new tab: ctrl+x c
- Move to next/previous tab: ctrl+x n/p
### Start and attach to VNC server
On the flypulator type:
```
vncserver 
```

On the Ground PC:
- Open the remote desktop client
- connect -> select VNC -> type 127.0.0.1:5901 -> press enter
- Password: IFA2018 (to be changed to ifa)
- The desktop of the flypulator should open
### Start SteamVR
open a terminal in the remote desktop. Type:
```
steamVR
```
observe that the tracker and the two lighthouses are marked green.
### ROS setup
On the Flypulator: (open four windows in the ssh session with tmux and launch one node in each window)
```
roscore

roslaunch flypulator_vive_pose vive_pose.launch

roslaunch flypulator_physical Telem2.launch

roslaunch flypulator_launch controller.launch
```

On the Ground PC
```
export ROS_MASTER_URI=http://flypulator:11311

rqt
```






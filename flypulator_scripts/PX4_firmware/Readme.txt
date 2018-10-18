The files in this directory are related to the PX4 firmware that is used on the Pixhawk 2.1 flight
controller.
->Docker
Contains a Dockerfile to launch a docker container with which the PX4 firmware can be compiled
without installing the toolchain on your main computer.
-> etc
This folder contains the configuration scripts that are needed to use the PX4 firmware with the
flypulator. As of now all the neccessary changes to PX4 can be done via the startup scrips,
with exception of the Airframe...

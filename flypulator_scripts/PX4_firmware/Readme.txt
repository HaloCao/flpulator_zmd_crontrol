The files in this directory are related to the PX4 firmware that is used on the Pixhawk 2.1 flight
controller.
->Docker
Contains a Dockerfile to launch a docker container with which the PX4 firmware can be compiled
without installing the toolchain on your main computer.
-> etc
This folder contains the configuration scripts that are needed to use the PX4 firmware with the
flypulator. As of now all the neccessary changes to PX4 can be done via the startup scripts:
-config.txt: this is loaded before the system is booted. We use it to start the mkblctrl driver
which enables the use of I2C ESCs and to turn off/on shell variables.
-extras.txt: This is loaded after boot and is used to turn off/on apps. We use it to control the
mavlink instances and to turn off the px4 controllers.
- mixers/xxx.main.mix: When this mixer file is present and has the same name as the original mixer
  of the chosen airframe, the xxx mixer is loaded instead. We use it to load
  a passthrough mixer that pass the actuator_control messages directly to the output channels of
  the pixhawk. This passthrough mixer is identical with the IO_pass.main.mix in the
  ROMFS/px4fmu_common/mixers directory of the PX4 firmware. 

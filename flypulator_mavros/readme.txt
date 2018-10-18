#This is a working document to document the steps taken to make maros work.

# did the following to install mavros:
sudo apt install mavros mavros-extras

To install GeographicLib which is needed for unit conversions:

sudo apt install geographiclib-tools geographiclib-doc

to install datasets:
sudo geographiclib-get-magnetic emm2015  
sudo geographiclib-get-gravity egm96  
sudo geographiclib-get-geoids egm96-5   

#to test mavros:
rosrun mavros mavros_node _fcu_url:=/dev/ttyUSB0:57600

in another window try a command:
rosrun mavros mavcmd long -b 520 1 0 0 0 0 0 0

To change the baudrate of the telem1 of the pixhawk I must add some line in a
/etc/extras.txt file on the sd card.

Stop the mavlink app:
mavlink stop-all
mavlink start -d /dev/ttyS1 -b 921600 -r 20000

Not tested!!!

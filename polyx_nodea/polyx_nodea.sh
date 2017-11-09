#!/bin/sh

# the two parameters below can be used to configure the serial port
# to connect to the sensor box serial output.  
# The default value to connect with the interface board is as below:

SER_PORT="/dev/ttyS4"
SER_BAUD=230400
SPEED_TOPIC="polyx_WheelSpeed"
OUTPUT_MSG=127

if rosnode list 2>&1 | grep "ERROR: Unable to communicate with master"
then 
   echo ROS has not started, please start ROS first!
else
   if rospack list | grep "polyx_nodea"
   then
     rosparam set polyx_port ${SER_PORT}
     rosparam set polyx_baud ${SER_BAUD}
     rosparam set polyx_speedreport ${SPEED_TOPIC}
     rosparam set polyx_output ${OUTPUT_MSG}
     rosrun polyx_nodea polyx_nodea_talker
   else
    echo Polyx_nodea package is not found, forgot to add package to ROS? 
   fi
fi


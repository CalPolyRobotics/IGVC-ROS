#!/bin/sh
#

# Make sure to put config file at this location
. /etc/default/ntripclient.conf
 
DateStart=`date -u '+%s'`
SLEEPMIN=10     # Wait min sec for next reconnect try
SLEEPMAX=60  # Wait max sec for next reconnect try
NTRIPCLIENT_ROS_NODE=$(catkin_find ntripclient_ros ntripclient_ros_node)
while true; 
do
  $NTRIPCLIENT_ROS_NODE -s $SERVER -r $PORT -m $MOUNTPOINT -u $USER -p $PASSWORD -D $DEVICE -B $BAUDRATE $OTHER
  if test $? -eq 0; 
  then 
    DateStart=`date -u '+%s'`; fi
    DateCurrent=`date -u '+%s'`
    SLEEPTIME=`echo $DateStart $DateCurrent | awk '{printf("%d",($2-$1)*0.02)}'`
  if test $SLEEPTIME -lt $SLEEPMIN; then SLEEPTIME=$SLEEPMIN; fi
  if test $SLEEPTIME -gt $SLEEPMAX; then SLEEPTIME=$SLEEPMAX; fi
  # Sleep 2 percent of outage time before next reconnect try
  echo "Retry after " + $SLEEPTIME
  sleep $SLEEPTIME
done
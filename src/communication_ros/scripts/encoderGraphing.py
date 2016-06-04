#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import time
import rospy
from std_msgs.msg import UInt16MultiArray 
global num
dt = 5 
accel_prescale = 50
global accel

def callback(data):
   global num
   print(data.data)
   num = 3000.0/data.data[0]

def listener():
   global num
   num = 0

   fig = plt.figure()
   ax = fig.add_subplot(121)
   ay = fig.add_subplot(122)

   # some X and Y data
   x = np.arange(100)
   y = [0] * 100
   accel = [0] * 100

   li, = ax.plot(x, y)
   liy, = ay.plot(x, accel)

   # draw and show it
   fig.canvas.draw()
   plt.show(block=False)
   ax.set_ylim([0,1])
   ay.set_ylim([-1,1])

   rospy.init_node('SpeedGun', anonymous=True)

   rospy.Subscriber("Get_Speed", UInt16MultiArray, callback)

   # loop to update the data
   while True:
      try:
         y[:-1] = y[1:]
         y[-1] = num 

         accel_num = accel_prescale * (y[-1] - y[-1 -dt]) / dt 
         print accel_num
         accel[:-1] = accel[1:]
         accel[-1] = accel_num

         # set the new data
         li.set_ydata(y)
         liy.set_ydata(accel)

         fig.canvas.draw()

         time.sleep(.01)

      except KeyboardInterrupt:
         break    
   
   rospy.spin()

if __name__ == '__main__':
    listener() 

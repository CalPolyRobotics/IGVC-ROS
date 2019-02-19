#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import time
import rospy
from std_msgs.msg import UInt16MultiArray, Int16
from sensor_msgs.msg import Imu


global encoder_v_raw
encoder_v_raw = 1
dt = 5 
accel_prescale = 50
global accel


global angular_v_raw

def callbackIMU(data):
    #print data.angular_velocity.x
    global angular_v_raw
    angular_v_raw = data.angular_velocity.z

def callback(data):
   global encoder_v_raw
   encoder_v_raw = data.data

def listener():
   global encoder_v_raw

   global angular_v_raw
   encoder_v_raw = 1
   angular_v_raw = 1


   fig = plt.figure()
   ax = fig.add_subplot(221)
   ay = fig.add_subplot(222)
   az = fig.add_subplot(223)
   a4 = fig.add_subplot(224)


   # some X and Y data
   x = np.arange(100)
   y = [0] * 100
   accel = [0] * 100
   angular = [0] * 100
   radius = [0] * 100

   lix, = ax.plot(x, y)
   liy, = ay.plot(x, accel)

   liz, = az.plot(x, angular)
   li4, = a4.plot(x, radius)
   #liy3, = ay.plot(x, accel)

   # draw and show it
   #ax.set_title(str("hello!"))
   ax.set_title("Velocity")
   ay.set_title("Acceleration")
   az.set_title("Angular Velocity")
   a4.set_title("Radius")

   ax.set_ylim([0,20])
   ay.set_ylim([-5,5])
   az.set_ylim([-2,2])
   a4.set_ylim([-1,20])
   ax.autoscale_view(True)
   
   fig.canvas.draw()
   
   plt.show(block=False)
   

   rospy.init_node('SpeedGun', anonymous=True)

   rospy.Subscriber("Get_Speed", Int16, callback)

   #rospy.init_node('Garbage', anonymous=True)
   rospy.Subscriber('imu/data', Imu, callbackIMU)

   # loop to update the data
   while True:
      try:
         #encoder_v = 27141.84/encoder_v_raw
         encoder_v = 27141
         #encoder_v = 60720.0/encoder_v_raw

         y[:-1] = y[1:]
         y[-1] = encoder_v
         ax.set_title(str(encoder_v))




         accel_num = accel_prescale * (y[-1] - y[-1 -dt]) / dt 
         angular_num = angular_v_raw
         radius_num = encoder_v / angular_num
         a4.set_title(str(3.28*2*radius_num))
         #print accel_num
         accel[:-1] = accel[1:]
         accel[-1] = accel_num
         angular[:-1] = angular[1:]
         angular[-1] = angular_num
         radius[:-1] = radius[1:]
         radius[-1] = radius_num




         # set the new data
         lix.set_ydata(y)
         liy.set_ydata(accel)
         liz.set_ydata(angular)
         li4.set_ydata(radius)





         fig.canvas.draw()

         time.sleep(.01)

      except KeyboardInterrupt:
         #plt.close()
         break

   rospy.spin()

if __name__ == '__main__':
    listener() 

import math

kDEBUG = False
SAFETY_MARGIN = 0.0

# The path of the golf cart while turning is a circle. If the LIDAR is at (0,0), these are
# the polar coordinates of the center of the circular path
lidarAngle = math.radians(90.0-120.0)   # To be determined as a function of steering angle
lidarDistance = 3.23 #3.1877 #2.94  # To be determined as a function of steering angle
circleInnerRadius = 2.15 - SAFETY_MARGIN #2.1717 #1.8542       # To be determined as a function of steering angle
circleOuterRadius = 3.68 #3.7338  + SAFETY_MARGIN #3.455        # To be determined as a function of steering angle

# Convert polar coordinates to cartesian. The center of the circle is located at 
# (circleOriginX,circleOriginY).
circleOriginX = lidarDistance * math.cos(lidarAngle)
circleOriginY = lidarDistance * math.sin(lidarAngle)
if kDEBUG == True: print("Circular path center coordinate: (%f,%f)" % (circleOriginX,circleOriginY))

# The portion of the circle we're using for this part is valid for these indices
kMAX_INDEX = 180 #269
kMIN_INDEX = 90

__calculated_ranges = []

for n in xrange(kMIN_INDEX,kMAX_INDEX + 1):
   # Slope of the ray from the LIDAR. Has equation y = mx
   m = math.tan(math.radians(n / 2.0 - 45))
   if kDEBUG == True: print("Slope: %f" % m)

   # xOuter and yOuter are the coordinates of the ray intersecting the outer circular path
   xOuter = (math.sqrt((pow(m,2) + 1) * pow(circleOuterRadius,2) - pow(m,2) * pow(circleOriginX,2) + 
      2.0 * m * circleOriginX * circleOriginY - pow(circleOriginY,2)) + m * circleOriginY + circleOriginX) / (pow(m,2) + 1.0)
   yOuter = m * xOuter
   if kDEBUG == True: print("Outer intersection coordinate: (%f,%f)" % (xOuter,yOuter))

   # Find the magnitude of the segment from (0,0) to (x,y)
   outerLimit = math.sqrt(pow(xOuter,2) + pow(yOuter,2))

   innerLimit = 0
   # If the inner circle is above the x-axis, then do extra calculations to determine inner limit
   if abs(circleInnerRadius) > abs(circleOriginY):
      # xInner and yInner are the coordinates of the intersection of the ray with the inner circular path
      try:
         xInner = (math.sqrt((pow(m,2) + 1) * pow(circleInnerRadius,2) - pow(m,2) * pow(circleOriginX,2) + 
            2.0 * m * circleOriginX * circleOriginY - pow(circleOriginY,2)) + m * circleOriginY + circleOriginX) / (pow(m,2) + 1.0)
         print xInner
      except Exception as e:
         xInner = 0

      yInner = m * xInner
      if kDEBUG == True: print("Outer intersection coordinate: (%f,%f)" % (xInner,yInner))

      # Find the magnitude of the segment from (0,0) to (x,y)
      innerLimit = math.sqrt(pow(xInner,2) + pow(yInner,2))
   
   __calculated_ranges.append((innerLimit, outerLimit))

def get_minmax_range(steering_angle, index):
   if index < kMIN_INDEX or index > kMAX_INDEX:
      return (0.0, 0.0)
   return __calculated_ranges[index - kMIN_INDEX]


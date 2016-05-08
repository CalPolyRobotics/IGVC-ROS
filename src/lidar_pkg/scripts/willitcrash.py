import math

kDEBUG = False
SAFETY_MARGIN = 0.0

# The portion of the circle we're using for this part is valid for these indices
kMAX_INDEX = 540
kMIN_INDEX = 0

# There are always two possible solutions so supply a 0 or 1 for solution_num to return the appropriate one
def getLimit(m, circleOriginX, circleOriginY, radius, solution_num):
    invert = 1
    if (solution_num == 1): invert = -1
    
    # Default value for no solution
    limit = -1

    # limitX and limitY are the coordinates of the intersection of the ray with the circular path
    try:
        # Solve for the first solution
        limitX = (invert * math.sqrt((m**2 + 1) * radius**2 - m**2 * circleOriginX**2 + 
            2.0 * m * circleOriginX * circleOriginY - circleOriginY**2) + m * circleOriginY + circleOriginX) / (m**2 + 1.0)
        limitY = m * limitX

        if kDEBUG == True: print("Intersection coordinate: (%f,%f)" % (limitX,limitY))
    
        # Find the magnitude of the segment from (0,0) to (x,y)
        limit = math.sqrt(limitX**2 + limitY**2)
  
    except Exception as e:
        pass

    return limit

def getCrashDistancesPolar(setLidarAngle, setLidarDistance, setCircleInnerRadius, setCircleOuterRadius):
    # The path of the golf cart while turning is a circle. If the LIDAR is at (0,0), these are
    # the polar coordinates of the center of the circular path
    lidarAngle = math.radians(90.0-setLidarAngle)   # To be determined as a function of steering angle
    lidarDistance = setLidarDistance #3.23 #3.1877 #2.94  # To be determined as a function of steering angle
    # Convert polar coordinates to cartesian. The center of the circle is located at 
    # (circleOriginX,circleOriginY).
    circleOriginX = lidarDistance * math.cos(lidarAngle)
    circleOriginY = lidarDistance * math.sin(lidarAngle)
    
    return getCrashDistancesCartesian(circleOriginX,circleOriginY,setCircleInnerRadius,setCircleOuterRadius)

def getCrashDistancesCartesian(setCircleOriginX,setCircleOriginY,setCircleInnerRadius,setCircleOuterRadius):
    circleInnerRadius = setCircleInnerRadius - SAFETY_MARGIN #2.15 2.1717 #1.8542       # To be determined as a function of steering angle
    circleOuterRadius = setCircleOuterRadius + SAFETY_MARGIN #3.68 #3.7338   #3.455        # To be determined as a function of steering angle

    #lidarAngle = math.radians(90.0-118.0)   # To be determined as a function of steering angle
    #lidarDistance = 3.72 #3.23 #3.1877 #2.94  # To be determined as a function of steering angle
    #circleInnerRadius = 2.71 - SAFETY_MARGIN #2.15 2.1717 #1.8542       # To be determined as a function of steering angle
    #circleOuterRadius = 4.19 + SAFETY_MARGIN #3.68 #3.7338   #3.455        # To be determined as a function of steering angle

    # Convert polar coordinates to cartesian. The center of the circle is located at (circleOriginX,circleOriginY).
    circleOriginX = setCircleOriginX
    circleOriginY = -1 * setCircleOriginY

    if kDEBUG == True: ("Circular path center coordinate: (%f,%f)" % (circleOriginX,circleOriginY))
    calculatedDistances = []
    
    # Append zeros
    for n in xrange(0,kMIN_INDEX):
        calculatedDistances.append((0,0,0))

    # Calculate for up to index 270 which is straight forward
    for n in xrange(kMIN_INDEX, 271):
        # Slope of the ray from the LIDAR. Has equation y = mx
        m = math.tan(math.radians(n / 2.0 - 45))
        if kDEBUG == True: print("Slope: %f" % m)

        outerLimit = getLimit(m,circleOriginX,circleOriginY,circleOuterRadius,0)
        
        if (circleOriginX > 0):
            innerLimit1 = getLimit(m,circleOriginX,circleOriginY,circleInnerRadius,1)
            innerLimit2 = getLimit(m,circleOriginX,circleOriginY,circleInnerRadius,0)
        else:
            innerLimit1 = -1
            innerLimit2 = -1
        
        calculatedDistances.append((innerLimit1, outerLimit, innerLimit2))
  
    # Calculate for indices greater than 270
    for n in xrange(271, kMAX_INDEX + 1):
        # Slope of the ray from the LIDAR. Has equation y = mx
        m = math.tan(math.radians(n / 2.0 - 45))
        if kDEBUG == True: print("Slope: %f" % m)

        outerLimit = getLimit(m,circleOriginX,circleOriginY,circleOuterRadius,0)
        if (circleOriginX > 0):
            innerLimit1 = -1
            innerLimit2 = -1
        else:
            innerLimit1 = getLimit(m,circleOriginX,circleOriginY,circleInnerRadius,1)
            innerLimit2 = getLimit(m,circleOriginX,circleOriginY,circleInnerRadius,0)

        calculatedDistances.append((innerLimit1, outerLimit, innerLimit2))
 
    # Append zeroes
    for n in xrange(kMAX_INDEX + 1, 540):
       calculatedDistances.append((0,0,0))

    return calculatedDistances

if __name__ == "__main__":
    getCrashDistancesCartesian(40,2,39,40)

from math import sqrt
from controller import Robot, DistanceSensor, Motor, Supervisor, Node, Camera
import numpy as np
import deap
import nnfs

# time in [ms] of a simulation step
TIME_STEP = 32

PrintStats = 0

MAX_SPEED = 6.28

##Bigger the number, the closer it will get to obstacles
DistanceValue = 78
# create the Robot instance.
robot = Supervisor()
camera = Camera("camera")
camera.enable(100)
# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

Colour = 1
ArrivalDeclared = 0
state = 0
boxFound = False

for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

psValuesAverage0 = []
psValuesAverage1 = []
psValuesAverage2 = []
psValuesAverage3 = []
psValuesAverage4 = []
psValuesAverage5 = []
psValuesAverage6 = []
psValuesAverage7 = []
avgSampleSize = 10
turningThreshhold = 15
fb = 0

push = 0
# ----------------------------------------------------------

def avg(lst): 
    if len(lst) != 0:
        return sum(lst) / len(lst) 
    
    
# ----------------------------------------------------------

def Explore():
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 1 * MAX_SPEED
    rightSpeed = 1 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    pass
# ----------------------------------------------------------
def BoxFound():
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    pass
# ----------------------------------------------------------
def IsPixelBox(x):
    pixelIsBox = 1
    image = camera.getImageArray()
    if Colour == 1:
        if image[x][0][0] < 200 or image[x][0][1] < 40 or image[x][0][1] > 60 or image[x][0][2] < 40 or image[x][0][2] > 60:
        #if image[x][0][0] > 60 or image[x][0][0] < 40 or image[x][0][1] < 150 or image[x][0][1] > 190 or image[x][0][2] < 70 or image[x][0][2] > 100:
            pixelIsBox = 0
    if Colour == 0:
        #if image[x][0][0] < 200 or image[x][0][1] < 40 or image[x][0][1] > 60 or image[x][0][2] < 40 or image[x][0][2] > 60:
        if image[x][0][0] > 60 or image[x][0][0] < 40 or image[x][0][1] < 150 or image[x][0][1] > 190 or image[x][0][2] < 70 or image[x][0][2] > 100:
            pixelIsBox = 0
    return pixelIsBox
# ----------------------------------------------------------        
def DirectionToFaceBox():
    direction = 0
    decided = 0
    if IsPixelBox(3) == 0:
        for p in range(len(camera.getImageArray())):
            if IsPixelBox(p):
                #if p<(len(image)-1)/2:
                if p<3:
                   direction = -1
                 #elif p>(len(image)-1)/2:
                elif p>3:
                   direction = 1
    
    for i in range(3):
        if decided == 0:
            if IsPixelBox(i) > IsPixelBox(6-i) and IsPixelBox(i+1) > IsPixelBox(6-(i+1)):
                direction = -1
                decided = 1
            elif IsPixelBox(i) < IsPixelBox(6-i) and IsPixelBox(i+1) < IsPixelBox(6-(i+1)):
                direction = 1
                decided = 1
            
    return direction

# ----------------------------------------------------------
def FaceBox():
    atBox = 0
    centered = 0
    while centered == 0:
    
        for i in range(3):
            if IsPixelBox(i) > IsPixelBox(6-i) and IsPixelBox(i+1) == IsPixelBox(6-(i+1)):
                centered = 1
            elif IsPixelBox(i) < IsPixelBox(6-i) and IsPixelBox(i+1) == IsPixelBox(6-(i+1)):
                centered = 1
            elif IsPixelBox(i) == IsPixelBox(6-i):
                centered = 1
    
        direction = DirectionToFaceBox()
         # initialize motor speeds at 50% of MAX_SPEED.
        leftSpeed  = 0.7 * MAX_SPEED
        rightSpeed = 0.7 * MAX_SPEED
        # modify speeds according to obstacles
        if direction == -1:
            # turn right
            leftSpeed  = 0.5 * MAX_SPEED
            rightSpeed = -0.5 * MAX_SPEED
        elif direction == 1:
            # turn left
            leftSpeed  = -0.5 * MAX_SPEED
            rightSpeed = 0.5 * MAX_SPEED
        # write actuators inputs
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        
    while atBox == 0:
        if (IsPixelBox(0) == 1 and IsPixelBox(3) == 1) or ((IsPixelBox(6) == 1 and IsPixelBox(3) == 1)):
            atBox =1
        else:
            leftMotor.setVelocity(0.1 * MAX_SPEED)
            rightMotor.setVelocity(0.1 * MAX_SPEED)
        
    pass
# ----------------------------------------------------------
def ArrivedAtBox():
    arrived = 1
    
    for i in range(7):
        if IsPixelBox(i) == 0:
            if psValues[0] < DistanceValue+30 or psValues[7] < DistanceValue+30:
                arrived = 0
    return arrived
# ----------------------------------------------------------    
def FaceBox2():

    leftSpeed  = 0.4 * MAX_SPEED
    rightSpeed = 0.4 * MAX_SPEED
    #while DirectionToFaceBox != 0:
    # modify speeds according to obstacles
    if DirectionToFaceBox() == 1:
        # turn right
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
    elif DirectionToFaceBox() == -1:
        # turn left
        leftSpeed  = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED

    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    pass
    
# ----------------------------------------------------------

def FaceBox3(L, R):
    if L > R and L - R > turningThreshhold:
        return 1
    elif L < R and R - L > turningThreshhold:
        return 2
    else:
        return 0

# ----------------------------------------------------------  

def FaceBox3a():
    fb = FaceBox3(psValues[0], psValues[7])
    print("Red: ", fb)
    while fb != 0:
        if fb == 1:
            print("fb = 1")
            leftSpeed  = -0.7 * MAX_SPEED
            rightSpeed = 0.7 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            fb = FaceBox3(psValues[0], psValues[7])
                
        if fb == 2:
            print("fb = 2")
            leftSpeed  = 0.7 * MAX_SPEED
            rightSpeed = -0.7 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            fb = FaceBox3(psValues[0], psValues[7])
            
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

# ----------------------------------------------------------   

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
      
    
    if PrintStats == 1:
        print("Red Camera Array: ", camera.getImageArray())
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    
    if len(psValuesAverage0) < avgSampleSize:
        psValuesAverage0.append(psValues[0])
    else:
        psValuesAverage0 = [avg(psValuesAverage0)]
    
    if len(psValuesAverage1) < avgSampleSize:
        psValuesAverage1.append(psValues[1])
    else:
        psValuesAverage1 = [avg(psValuesAverage1)]
    
    if len(psValuesAverage2) < avgSampleSize:
        psValuesAverage2.append(psValues[2])
    else:
        psValuesAverage2 = [avg(psValuesAverage2)]
    
    if len(psValuesAverage3) < avgSampleSize:
        psValuesAverage3.append(psValues[3])
    else:
        psValuesAverage3 = [avg(psValuesAverage3)]
    
    if len(psValuesAverage4) < avgSampleSize:
        psValuesAverage4.append(psValues[4])
    else:
        psValuesAverage4 = [avg(psValuesAverage4)]
    
    if len(psValuesAverage5) < avgSampleSize:
        psValuesAverage5.append(psValues[5])
    else:
        psValuesAverage5 = [avg(psValuesAverage5)]
    
    if len(psValuesAverage6) < avgSampleSize:
        psValuesAverage6.append(psValues[6])
    else:
        psValuesAverage6 = [avg(psValuesAverage6)]
    
    if len(psValuesAverage7) < avgSampleSize:
        psValuesAverage7.append(psValues[7])
    else:
        psValuesAverage7 = [avg(psValuesAverage7)]

        
        
    # detect obstacles
    right_obstacle = avg(psValuesAverage0) > DistanceValue or avg(psValuesAverage1) > DistanceValue or avg(psValuesAverage2) > DistanceValue
    left_obstacle = avg(psValuesAverage5) > DistanceValue or avg(psValuesAverage6) > DistanceValue or avg(psValuesAverage7) > DistanceValue
    

    
    image = camera.getImageArray()[3][0]
    boxFound = 0
    for p in range(len(camera.getImageArray())):
        if PrintStats == 1:
            print(p, " - ", IsPixelBox(p), " ", camera.getImageArray()[p][0])
        if IsPixelBox(p) == 1:
            boxFound = 1
            state = 0
    if boxFound == 1:
        state = 1
        
    if state == 0 and ArrivalDeclared == 0:
        Explore()
        
    elif(state == 1):
        if PrintStats == 1:
            print("Red Box Found!!! ", image)
        #leftMotor.setVelocity(0)
        #rightMotor.setVelocity(0)
        if boxFound == 0 and ArrivalDeclared == 0:
            state = 0
        #state = 0
        if (left_obstacle or right_obstacle) and ArrivalDeclared != 1 and (state == 0 or state == 1):
            Explore()
        else:
            FaceBox2()
        if ArrivedAtBox() == 1:  
            if ArrivalDeclared == 0:
                ArrivalDeclared = 1
                #print("Green Box located, Awaiting further instructions")
            PrintStats = 0
            ArrivalDeclared = 1
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            
            fb = FaceBox3(avg(psValuesAverage0), avg(psValuesAverage7))
              
            print("Red: ", fb)
            if fb == 0:
                if avg(psValuesAverage2) > 69:
                    push = 1
                else:
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
            
            if fb == 1:
                print("fb = 1")
                leftSpeed  = -0.7 * MAX_SPEED
                rightSpeed = 0.7 * MAX_SPEED
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
                
                        
            if fb == 2:
                print("fb = 2")
                leftSpeed  = 0.7 * MAX_SPEED
                rightSpeed = -0.7 * MAX_SPEED
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
            
            if push ==1:
                leftMotor.setVelocity(MAX_SPEED)
                rightMotor.setVelocity(MAX_SPEED)
              
                    
            
                    

    
    
    if PrintStats == 1:
        print("State: ", state, " | Camera reading: ", image, " | ", leftMotor.getVelocity(), ", ", leftMotor.getVelocity(), " L/R", DirectionToFaceBox())
        print(" ")
    
from math import sqrt
from controller import Robot, DistanceSensor, Motor, Supervisor, Node, Camera
import numpy as np
import deap
import nnfs

# time in [ms] of a simulation step
TIME_STEP = 32

PrintStats = 1

MAX_SPEED = 6.28

##Bigger the number, the closer it will get to obstacles
DistanceValue = 77
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



# ----------------------------------------------------------
def Explore():
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
    for p in range(3):
        if image[x][0][p] > 8:
            pixelIsBox = 0
    return pixelIsBox
# ----------------------------------------------------------        
def DirectionToFaceBox():
    direction = 0
    if IsPixelBox(3) == 0:
        for p in range(len(camera.getImageArray())):
            if IsPixelBox(p):
                #if p<(len(image)-1)/2:
                if p<3:
                    direction = -1
                #elif p>(len(image)-1)/2:
                elif p>3:
                    direction = 1
    return direction

# ----------------------------------------------------------
def FaceBox():
    atBox = 0
    while IsPixelBox(3) != 1:
        direction = DirectionToFaceBox()
         # initialize motor speeds at 50% of MAX_SPEED.
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
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
            leftMotor.setVelocity(0.5 * MAX_SPEED)
            rightMotor.setVelocity(0.5 * MAX_SPEED)
        
    pass
# ----------------------------------------------------------
def ArrivedAtBox():
    arrived = 1
    for i in range(7):
        if IsPixelBox(i) == 0:
            arrived = 0
    return arrived
# ----------------------------------------------------------    
def FaceBox2():

    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
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

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    if PrintStats == 1:
        print("Camera Array: ", camera.getImageArray())
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # detect obstacles
    right_obstacle = psValues[0] > DistanceValue or psValues[1] > DistanceValue or psValues[2] > DistanceValue
    left_obstacle = psValues[5] > DistanceValue or psValues[6] > DistanceValue or psValues[7] > DistanceValue
    
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
        
    if (state == 0):
        Explore()
        
    elif(state == 1):
        if PrintStats == 1:
            print("Box Found!!! ", image)
        #leftMotor.setVelocity(0)
        #rightMotor.setVelocity(0)
        if boxFound == 0:
            state = 0
        #state = 0
        if left_obstacle or right_obstacle:
            Explore()
        else:
            FaceBox2()
            
        print(ArrivedAtBox())
        if ArrivedAtBox() == 1:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
    
    
    
    if PrintStats == 1:
        print("State: ", state, " | Camera reading: ", image, " | ", leftMotor.getVelocity(), ", ", leftMotor.getVelocity(), " L/R", DirectionToFaceBox())
        print(" ")

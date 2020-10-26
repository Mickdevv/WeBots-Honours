"""epuck_avoid_collision controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from math import sqrt
from controller import Robot, DistanceSensor, Motor, Supervisor, Node, Camera
import numpy as np
import deap
import nnfs

#import numpy as np
# create the Robot instance.
#robot = Robot()
robot = Supervisor()
camera = Camera("camera")
camera.enable(100)
# get handle to robot's translation field
robot.getTime()
# get the time step of the current world.
TIME_STEP = 64
# initialize devices
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3',
            'ps4', 'ps5', 'ps6', 'ps7']


def Explore():
# Process sensor data here.
    right_obstacle = psValues[0] > DT or psValues[1] > 83 or psValues[2] > 88
    left_obstacle = psValues[5] > DT or psValues[6] > 83 or psValues[7] > 88

    # Enter here functions to send actuator commands, like:
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    # if left_obstacle & right_obstacle:
    #     leftSpeed = -0.5 * MAX_SPEED
    #     rightSpeed = -0.5 * MAX_SPEED
    if left_obstacle:
        # turn right
        leftSpeed += 0.5 * MAX_SPEED
        rightSpeed -= 0.5 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed -= 0.5 * MAX_SPEED
        rightSpeed += 0.5 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    pass

def MeasureBoxSize():
    leftMotor



for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)

MAX_SPEED = 6.28

leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

#Distance Threshhold: Higher the number, the closer it will get to things
DT = 76
# Main loop:
# - perform simulation steps until Webots is stopping the controller
t = robot.getTime()
while robot.getTime() > -1:
    # Read the sensors:
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    image = camera.getImageArray()
    #gray = Camera.imageGetGray(image, camera.getWidth(), 0, 0)
    #print(gray)
    #red = camera.imageGetRed(image, camera.getWidth(), 0, 0)
    #print(red)
    print(type(image))
    print(image)
    print("--------------------------")
    
    #for x in range(0,camera.getWidth()):
    #     for y in range(0,camera.getHeight()):
    #          red   = image[x]
    #          green = image[x]
    #          blue  = image[x]
    #          gray  = (red + green + blue) / 3
    #          print ('r='+str(red)+' g='+str(green)+' b='+str(blue))
    if robot.step(TIME_STEP) == -1:
        quit()
    
    Explore()
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

# reset robot position and physics
#supervisor.simulationReset()

    

# Enter here exit cleanup code.

"""epuck_Supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from math import sqrt
from controller import DistanceSensor, Motor, Supervisor
import numpy as np
import deap
import nnfs

TIME_STEP = 32

supervisor = Supervisor()

# get handle to robot's translation field
robot_node = supervisor.getFromDef("e-puck")
trans_field = robot_node.getField("translation")

# initialize devices
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3',
            'ps4', 'ps5', 'ps6', 'ps7']

def Explore():
# Process sensor data here.
    right_obstacle = psValues[0] > DT or psValues[1] > DT or psValues[2] > DT
    left_obstacle = psValues[5] > DT or psValues[6] > DT or psValues[7] > DT

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

for i in range(8):
    ps.append(robot_node.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)

MAX_SPEED = 6.28

leftMotor = robot_node.getMotor('left wheel motor')
rightMotor = robot_node.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

#Distance Threshhold: Higher the number, the closer it will get to things
DT = 68

while robot_node.step(timestep) != -1:
    # Read the sensors:
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
        
    Explore()

for a in range(0, 25):
    for b in range(0, 33):
        # evaluate robot during 60 seconds (simulation time)
        t = supervisor.getTime()
        while supervisor.getTime() - t < 1:

            # perform robot control according to a, b
            # (and possibly t) parameters.

            # controller termination
            if supervisor.step(TIME_STEP) == -1:
                quit()

        # compute travelled distance
        values = trans_field.getSFVec3f()
        dist = sqrt(values[0] * values[0] + values[2] * values[2])
        print("a=%d, b=%d -> dist=%g" % (a, b, dist))

        # reset robot position and physics
        INITIAL = [0.2, 0, 0]
        trans_field.setSFVec3f(INITIAL)
        robot_node.resetPhysics()
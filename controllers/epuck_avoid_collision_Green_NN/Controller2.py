from math import sqrt
from controller import Robot, DistanceSensor, Motor, Supervisor, Node, Camera
import numpy as np
import deap
import nnfs

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
            

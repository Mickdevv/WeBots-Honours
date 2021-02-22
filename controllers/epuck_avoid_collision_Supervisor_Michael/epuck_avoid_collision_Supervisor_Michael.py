"""epuck_Supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from math import sqrt
from controller import Robot, DistanceSensor, Motor, Supervisor, Node, Camera, Field, GPS
import numpy as np
import deap
import nnfs
import os
import time
import csv
import sys

TIME_STEP = 32

supervisor = Supervisor()
robot_node = supervisor.getFromDef("Robot")
if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)
trans_field = robot_node.getField("translation")

reset = 0
timeRecorded = 0

Colours = ["Red", "Green", "Blue"]
# ----------------------------------------------------------   
def RobotsArrivedAtBox():
    checkRed = 0
    checkGreen = 0
    checkBlue = 0
    
    file = "..\\epuck_avoid_collision_Red\Red.txt"
    if os.path.exists(file):
        OtherFile = open(file, "r")
        for element in OtherFile.read():
            if element == "1":
                checkRed = 1
                
    file = "..\\epuck_avoid_collision_Blue\Blue.txt"
    if os.path.exists(file):
        OtherFile = open(file, "r")
        for element in OtherFile.read():
            if element == "1":
                checkBlue = 1
            
    file = "..\\epuck_avoid_collision_Green\Green.txt"
    if os.path.exists(file):
        OtherFile = open(file, "r")
        for element in OtherFile.read():
            if element == "1":
                checkGreen = 1
                
    if checkGreen == 1 and checkBlue == 1 and checkRed == 1:
        return 1
    else:
        return 0
         
# ----------------------------------------------------------   
while supervisor.step(TIME_STEP) != -1:
    # if timeRecorded == 0:
        # print(1)
        # print(supervisor.getFromDef("E-puck"))
        # Node.restartController(supervisor.getFromDef("e-puck"))
    # print(2)
    
        #for i in range(len(Colours)):
            #print(1)
            #Node.restartController(supervisor.getFromDef("e-puck_" + Colours[i]))
            #timeRecorded = 1

    if RobotsArrivedAtBox() == 1 and timeRecorded == 0:
        with open('BlueTimes.csv', 'a') as the_file:
            the_file.writelines(str(supervisor.getTime()))
            the_file.writelines('\n')
            print("Time recorded: ", supervisor.getTime())
            timeRecorded = 1
            
        for i in range(len(Colours)):
            f = open(Colours[i] + ".txt", "w")
            f.write("0")
        
        if reset == 1:
            supervisor.worldReload()
        

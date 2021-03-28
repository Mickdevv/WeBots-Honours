"""epuck_Supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot, DistanceSensor, Motor, Supervisor, Node, Camera, Field, GPS
import numpy as np
import deap, nnfs, os, time, csv, sys, random
import pandas as pd
import time

start = time.time()

numberOfRobots = 3

TIME_STEP = 32

SimulationTimeLimit = 600

with open('Times.csv', 'a') as the_file:
    writer = csv.writer(the_file)
    writer.writerow("N")

PositionsSet = 0

Runs = 0
times = []
times2 = []

supervisor = Supervisor()
robot_node = supervisor.getFromDef("Supervisor")
trans_field = robot_node.getField("translation")

#--- Robot Nodes ---
robot_node_Red = supervisor.getFromDef("Red")
trans_field_Red = robot_node_Red.getField("translation")

robot_node_Green = supervisor.getFromDef("Green")
trans_field_Green = robot_node_Green.getField("translation")

robot_node_Blue = supervisor.getFromDef("Blue")
trans_field_Blue = robot_node_Blue.getField("translation")

#--- Wall and box Nodes ---
NorthWallNode = supervisor.getFromDef("NorthWall")
trans_field_North_Wall = NorthWallNode.getField("translation")

SouthWallNode = supervisor.getFromDef("SouthWall")
trans_field_South_Wall = SouthWallNode.getField("translation")

EastWallNode = supervisor.getFromDef("EastWall")
trans_field_East_Wall = EastWallNode.getField("translation")

WestWallNode = supervisor.getFromDef("WestWall")
trans_field_West_Wall = WestWallNode.getField("translation")



startingPositionsPermanent = [
[[1.81, 0.02, -0.14], [-0.48, 0.02, -0.58], [-2.4, 0.02, 0.02]], 
[[-0.92, 0.02, -0.32], [-0.13, 0.02, -1.07], [2.11, 0.02, 0.28]], 
[[-1.13, 0.02, -0.96], [-2.03, 0.02, -0.16], [-1.6, 0.02, 0.13]], 
[[-1.02, 0.02, -0.57], [-1.34, 0.02, -0.06], [-2.34, 0.02, -1.15]],
[[-1.43, 0.02, 0.3], [-1.68, 0.02, -0.89], [-2.62, 0.02, -0.39]],
[[-0.97, 0.02, 0.44], [1.71, 0.02, -1.02], [0.83, 0.02, -0.24]],
[[-0.25, 0.02, 0.5], [-0.67, 0.02, -0.49], [1.3, 0.02, -1.06]],
[[1.41, 0.02, 0.53], [-0.4, 0.02, -0.72], [-0.72, 0.02, -0.72]], 
[[0.74, 0.02, -0.71], [1.89, 0.02, -0.33], [-0.7, 0.02, -0.38]], 
[[-0.83, 0.02, 0.2], [-1.44, 0.02, 0.15], [-0.7, 0.02, 0.18]], 
[[1.67, 0.02, -0.41], [-0.09, 0.02, 0.4], [-1.5, 0.02, -0.98]],
[[-0.51, 0.02, -0.05], [-0.14, 0.02, -0.8], [-1.58, 0.02, -0.96]], 
[[-2, 0.02, -0.26], [-2, 0.02, 0.53], [-2.42, 0.02, 0.49]], 
[[-0.72, 0.02, -0.47], [-0.54, 0.02, -0.43], [-2.29, 0.02, 0.02]], 
[[0.94, 0.02, -0.83], [-2.34, 0.02, -0.67], [2.48, 0.02, -1.1]], 
[[1.34, 0.02, -1.15], [-1.3, 0.02, -0.72], [-2.41, 0.02, -0.78]]]

startingPositionsGenerated = []
#print(trans_field.getSFVec3f())

reset = 0
TimeToRecord = 0
TimeToRecord2 = 0

Colours = ["Red", "Green", "Blue"]
TooCloseDistance = 0.1


#-----------------------------------------------------------

def DistanceBetween(p1, p2):
    Distance = 0.0
    
    for i in range(len(p1)):
        Distance = Distance + pow((p1[i] - p2[i]), 2)
    
    Distance = math.sqrt(Distance)
    return Distance


#-----------------------------------------------------------
def GenerateStartingPositions():
    StartingHeight = 0.02
    StartingPositionsX = []
    StartingPositionsZ = []
    generatedStarting = []
    tempStarting = []
    tooClose = 0
    print(1)
    for k in range(30):
        for j in range(3):
            
            #Generate random position
            X = round(random.uniform(trans_field_South_Wall.getSFVec3f()[0] + TooCloseDistance, trans_field_North_Wall.getSFVec3f()[0] - TooCloseDistance), 2)
            Z = round(random.uniform(trans_field_West_Wall.getSFVec3f()[2] + TooCloseDistance, trans_field_East_Wall.getSFVec3f()[2] - TooCloseDistance), 2)
            # for i in range(len(StartingPositionsZ)):
                # if Z - StartingPositionsZ[i] < TooCloseDistance and X - StartingPositionsZ[i] > -TooCloseDistance:
                    # tooClose = 1
            for i in range(len(StartingPositionsX)):
                    if DistanceBetween([X, Z], [StartingPositionsX[i], StartingPositionsZ[i]]) < TooCloseDistance:
                        tooClose = 1
                        
            #If the position is invalid, generate random positions until it is        
            while DistanceBetween([X, Z], [trans_field.getSFVec3f()[0], trans_field.getSFVec3f()[2]]) < 0.6 or tooClose == 1:
                Z = round(random.uniform(trans_field_West_Wall.getSFVec3f()[2] + TooCloseDistance, trans_field_East_Wall.getSFVec3f()[2] - TooCloseDistance), 2)
                X = round(random.uniform(trans_field_South_Wall.getSFVec3f()[0] + TooCloseDistance, trans_field_North_Wall.getSFVec3f()[0] - TooCloseDistance), 2)
                for i in range(len(StartingPositionsX)):
                    if DistanceBetween([X, Z], [StartingPositionsX[i], StartingPositionsZ[i]]) < TooCloseDistance:
                        tooClose = 1
                  
            StartingPositionsX.append(X)
            StartingPositionsZ.append(Z)
    
        tempStarting.append([StartingPositionsX[0], StartingHeight, StartingPositionsZ[0]])
        tempStarting.append([StartingPositionsX[1], StartingHeight, StartingPositionsZ[1]])
        tempStarting.append([StartingPositionsX[2], StartingHeight, StartingPositionsZ[2]])
        generatedStarting.append(tempStarting)
        StartingPositionsX = []
        StartingPositionsZ = []
        tempStarting = []
    return generatedStarting
    #print(StartingPositionsX)
# ----------------------------------------------------------  
def setRandomPositions():
    StartingHeight = 0.02
    StartingPositionsX = []
    StartingPositionsZ = []
    tooClose = 0
    
    for j in range(3):
        
        #Generate random position
        X = round(random.uniform(trans_field_South_Wall.getSFVec3f()[0] + TooCloseDistance, trans_field_North_Wall.getSFVec3f()[0] - TooCloseDistance), 2)
        Z = round(random.uniform(trans_field_West_Wall.getSFVec3f()[2] + TooCloseDistance, trans_field_East_Wall.getSFVec3f()[2] - TooCloseDistance), 2)
        # for i in range(len(StartingPositionsZ)):
            # if Z - StartingPositionsZ[i] < TooCloseDistance and X - StartingPositionsZ[i] > -TooCloseDistance:
                # tooClose = 1
        for i in range(len(StartingPositionsX)):
                if DistanceBetween([X, Z], [StartingPositionsX[i], StartingPositionsZ[i]]) < TooCloseDistance:
                    tooClose = 1
                    
        #If the position is invalid, generate random positions until it is        
        while DistanceBetween([X, Z], [trans_field.getSFVec3f()[0], trans_field.getSFVec3f()[2]]) < 0.6 or tooClose == 1:
            Z = round(random.uniform(trans_field_West_Wall.getSFVec3f()[2] + TooCloseDistance, trans_field_East_Wall.getSFVec3f()[2] - TooCloseDistance), 2)
            X = round(random.uniform(trans_field_South_Wall.getSFVec3f()[0] + TooCloseDistance, trans_field_North_Wall.getSFVec3f()[0] - TooCloseDistance), 2)
            for i in range(len(StartingPositionsX)):
                if DistanceBetween([X, Z], [StartingPositionsX[i], StartingPositionsZ[i]]) < TooCloseDistance:
                    tooClose = 1
            
            # for i in range(len(StartingPositionsX)):
                # if X - StartingPositionsX[i] < TooCloseDistance and X - StartingPositionsX[i] > -TooCloseDistance:
                    # tooClose = 1
                    
                        
        StartingPositionsX.append(X)
        StartingPositionsZ.append(Z)
    
    trans_field_Red.setSFVec3f([StartingPositionsX[0], StartingHeight, StartingPositionsZ[0]])
    trans_field_Green.setSFVec3f([StartingPositionsX[1], StartingHeight, StartingPositionsZ[1]])
    trans_field_Blue.setSFVec3f([StartingPositionsX[2], StartingHeight, StartingPositionsZ[2]])

    return [[StartingPositionsX[0], StartingHeight, StartingPositionsZ[0]], [StartingPositionsX[1], StartingHeight, StartingPositionsZ[1]], [StartingPositionsX[2], StartingHeight, StartingPositionsZ[2]]]
    #print(StartingPositionsX)
    #print(StartingPositionsZ)
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
def sum(lst):
    sum = 0
    for i in range(len(lst)):
        sum += lst[i]
        
    return sum
        
# ---------------------------------------------------------- 
def avg(lst):
    avg = 0
    avg = sum(lst)/len(lst)
    
    return avg
# ---------------------------------------------------------- 
while supervisor.step(TIME_STEP) != -1 and Runs < len(startingPositionsPermanent):
    
    if len(times) > 0:
        if times[0] == 0:
            times.remove(0)
            runs = 0
    if PositionsSet == 0:
        print("Starting positions:")
        print([startingPositionsPermanent[Runs][0][0], startingPositionsPermanent[Runs][0][1], startingPositionsPermanent[Runs][0][2]])
        print([startingPositionsPermanent[Runs][1][0], startingPositionsPermanent[Runs][1][1], startingPositionsPermanent[Runs][1][2]])
        print([startingPositionsPermanent[Runs][2][0], startingPositionsPermanent[Runs][2][1], startingPositionsPermanent[Runs][2][2]])
        #startingPositionsGenerated = GenerateStartingPositions()
        #print(startingPositionsGenerated)
        #print(GenerateStartingPositions())
        if numberOfRobots == 1:
            trans_field_Green.setSFVec3f([startingPositionsPermanent[Runs][1][0], startingPositionsPermanent[Runs][1][1], startingPositionsPermanent[Runs][1][2]])
            
        if numberOfRobots == 2:
            trans_field_Red.setSFVec3f([startingPositionsPermanent[Runs][0][0], startingPositionsPermanent[Runs][0][1], startingPositionsPermanent[Runs][0][2]])
            trans_field_Blue.setSFVec3f([startingPositionsPermanent[Runs][2][0], startingPositionsPermanent[Runs][2][1], startingPositionsPermanent[Runs][2][2]])
       
        elif numberOfRobots == 3:
            trans_field_Red.setSFVec3f([startingPositionsPermanent[Runs][0][0], startingPositionsPermanent[Runs][0][1], startingPositionsPermanent[Runs][0][2]])
            trans_field_Green.setSFVec3f([startingPositionsPermanent[Runs][1][0], startingPositionsPermanent[Runs][1][1], startingPositionsPermanent[Runs][1][2]])
            trans_field_Blue.setSFVec3f([startingPositionsPermanent[Runs][2][0], startingPositionsPermanent[Runs][2][1], startingPositionsPermanent[Runs][2][2]])
            #startingPositionsPermanent.append(setRandomPositions())
        #print(startingPositionsPermanent)
        #for i in range(len(startingPositionsPermanent)):
            #print(startingPositionsPermanent[i])
            
        PositionsSet = 1
            
    
            #print(trans_field.getSFVec3f())
    if trans_field.getSFVec3f()[0] > 0.31 and TimeToRecord2 == 0:
        TimeToRecord2 = round(supervisor.getTime(), 2)
        
    if trans_field.getSFVec3f()[0] > 2.5:
        TimeToRecord = round(supervisor.getTime(), 2)
        reset = 1
    elif supervisor.getTime() > SimulationTimeLimit:
        TimeToRecord = -1
        reset = 1
                

        
            #if trans_field.getSFVec3f()[0] > 2.5 or supervisor.getTime() > SimulationTimeLimit:
        
    if reset == 1:   
        print("Push time: ", round(TimeToRecord - TimeToRecord2, 2))  
        times.append([TimeToRecord, TimeToRecord2])

                
        with open('Times.csv', 'a') as the_file:
            writer = csv.writer(the_file)
            writer.writerow(times[Runs])
            print("Times recorded: ", TimeToRecord2, ", ", TimeToRecord)
            
        TimeToRecord2 = 0        
        timeRecorded = 1
        supervisor.simulationReset()
        Node.restartController(supervisor.getFromDef("Red"))
        Node.restartController(supervisor.getFromDef("Green"))
        Node.restartController(supervisor.getFromDef("Blue"))
        PositionsSet = 0
        Runs +=1
        if len(times) >= 1:
            print("Runs completed: ", Runs, "/", len(startingPositionsPermanent))
            #print(times)
        reset = 0
        end = time.time()
        print("Time elapsed: ", round(end - start, 2))
    


from math import sqrt
from controller import Robot, DistanceSensor, Motor, Supervisor, Node, Camera, Field, GPS
import numpy as np
import deap
import nnfs
import os
import time
import csv

Colour = 0
PrintStats = 0    
    
# time in [ms] of a simulation step
TIME_STEP = 32

MAX_SPEED = 6.28

##Bigger the number, the closer it will get to obstacles
DistanceValue = 78
# create the Robot instance.
robot = Robot()
camera = Camera("camera")
camera.enable(100)
#gps = GPS("gps")
#gps.enable(100)
# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

state = 0
ArrivalDeclared = 0
reset = 0

if Colour == 0:
    ColourText = "Green"
if Colour == 1:
    ColourText = "Red"
if Colour == 2:
    ColourText = "Blue"

if os.path.exists(ColourText + ".txt"):
    os.remove(ColourText + ".txt")
f = open(ColourText + ".txt", "w")
f.write("0")

with open('Blue.csv', 'w') as csvfile:
    csvwriter = csv.writer(csvfile)
    
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
turningThreshhold = 10
fb = 0

push = 0
# ----------------------------------------------------------

def avg(lst): 
    if len(lst) != 0:
        return sum(lst) / len(lst) 
    
# ----------------------------------------------------------

def sum(lst): 
    sum = 0
    for i in range(len(lst)):
        sum = sum + lst[i]
    
    return sum
    
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
    leftSpeed  = 0.9 * MAX_SPEED
    rightSpeed = 0.9 * MAX_SPEED
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
    
    #Green
    if Colour == 0:
        if image[x][0][0] * 6 > image[x][0][1] + 20 or image[x][0][0] * 6 < image[x][0][1] - 20 or image[x][0][0] * 5 > image[x][0][2]*2 + 40 or image[x][0][0] * 5 < image[x][0][2]*2 - 40:
        #if image[x][0][0] < 200 or image[x][0][1] < 40 or image[x][0][1] > 60 or image[x][0][2] < 40 or image[x][0][2] > 60:
        #if image[x][0][0] > 40 or image[x][0][0] < 20 or image[x][0][1] < 170 or image[x][0][1] > 200 or image[x][0][2] < 70 or image[x][0][2] > 100:
            pixelIsBox = 0
           
    #Red 
    if Colour == 1:
        if image[x][0][0] < image[x][0][1]*5 - 20 or image[x][0][0] > image[x][0][2]*4 + 20 or image[x][0][0] < image[x][0][2]*3 - 20:
        #if image[x][0][0] > 60 or image[x][0][0] < 40 or image[x][0][1] < 150 or image[x][0][1] > 190 or image[x][0][2] < 70 or image[x][0][2] > 100:
            pixelIsBox = 0
    
    #Blue        
    if Colour == 2:
        if image[x][0][0]*3 < image[x][0][2] - 30 or image[x][0][0]*3 > image[x][0][2] + 20 or image[x][0][1]*5 > image[x][0][2]*2 + 30 or image[x][0][1]*5 < image[x][0][2]*2 - 50:
        #if image[x][0][0] > 60 or image[x][0][0] < 40 or image[x][0][1] < 150 or image[x][0][1] > 190 or image[x][0][2] < 70 or image[x][0][2] > 100:
            pixelIsBox = 0        
                    
    if sum(image[x][0]) <= 60:
        return 0
    else:        
        return pixelIsBox
        
# ----------------------------------------------------------        
def DirectionToFaceBox():
    direction = 0
    decided = 0
    st2 = 0
    if IsPixelBox(3) == 0:
        for p in range(len(camera.getImageArray())):
            if IsPixelBox(p):
                #if p<(len(image)-1)/2:
                if p<3:
                   direction = -1
                   st2 = 1
                 #elif p>(len(image)-1)/2:
                elif p>3:
                   direction = 1
                   st2 = 1
    if st2 == 0:
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

    leftSpeed  = 0.9 * MAX_SPEED
    rightSpeed = 0.9 * MAX_SPEED
    #while DirectionToFaceBox != 0:
    # modify speeds according to obstacles
    if DirectionToFaceBox() == 1:
        # turn right
        leftSpeed  = 0.9 * MAX_SPEED
        rightSpeed = -0.9 * MAX_SPEED
    elif DirectionToFaceBox() == -1:
        # turn left
        leftSpeed  = -0.9 * MAX_SPEED
        rightSpeed = 0.9 * MAX_SPEED

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
    #print("Green: ", fb)
    while fb != 0:
        if fb == 1:
            #print("fb = 1")
            leftSpeed  = -0.9 * MAX_SPEED
            rightSpeed = 0.9 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            fb = FaceBox3(psValues[0], psValues[7])
                
        if fb == 2:
            #print("fb = 2")
            leftSpeed  = 0.9 * MAX_SPEED
            rightSpeed = -0.9 * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            fb = FaceBox3(psValues[0], psValues[7])
            
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

# ----------------------------------------------------------   
def OtherRobotArrival():
    checkS1 = 0
    checkS2 = 0
    
    if Colour == 0:
        file = "..\\epuck_avoid_collision_Red\Red.txt"
        if os.path.exists(file):
            OtherFile = open(file, "r")
            for element in OtherFile.read():
                if element == "1":
                    checkS1 = 1
        file = "..\\epuck_avoid_collision_Blue\Blue.txt"
        if os.path.exists(file) and checkS1 == 1:
            OtherFile = open(file, "r")
            for element in OtherFile.read():
                if element == "1":
                    checkS2 = 1
            
    if Colour == 1:
        file = "..\\epuck_avoid_collision_Green\Green.txt"
        if os.path.exists(file):
            OtherFile = open(file, "r")
            for element in OtherFile.read():
                if element == "1":
                    checkS1 = 1
        file = "..\\epuck_avoid_collision_Blue\Blue.txt"
        if os.path.exists(file) and checkS1 == 1:
            OtherFile = open(file, "r")
            for element in OtherFile.read():
                if element == "1":
                    checkS2 = 1
                    
    if Colour == 2:
        file = "..\\epuck_avoid_collision_Green\Green.txt"
        if os.path.exists(file):
            OtherFile = open(file, "r")
            for element in OtherFile.read():
                if element == "1":
                    checkS1 = 1
        file = "..\\epuck_avoid_collision_Red\Red.txt"
        if os.path.exists(file) and checkS1 == 1:
            OtherFile = open(file, "r")
            for element in OtherFile.read():
                if element == "1":
                    checkS2 = 1

    return checkS2
        
# ----------------------------------------------------------   

def BoxInFrame():
    condition = 0
    for i in range(len(image)):
        if IsPixelBox(i) == 1:
            condition = 1
        
    return condition    

# ----------------------------------------------------------   


# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
      
    if reset == 2:
        f = open(ColourText + ".txt", "w")
        f.write("0")
        state = 0
        ArrivalDeclared = 0
        reset = 0
    
    if PrintStats == 1:
        print(ColourText, " Camera Array: ", camera.getImageArray())
        
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
    

    
    image = camera.getImageArray()
    boxFound = 0
    for p in range(len(camera.getImageArray())):
        if PrintStats == 1:
            print(p, " - ", IsPixelBox(p), " ", camera.getImageArray()[p][0])
        if IsPixelBox(p) == 1:
            boxFound = 1
    
    if state == 0 and ArrivalDeclared == 0:
        if boxFound == 1:
            state = 1
        Explore()
        
    elif state == 1:
        if PrintStats == 1:
            print(ColourText, " Box Found!!! ", OtherRobotArrival(), ", ", ArrivedAtBox())
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
            f = open(ColourText + ".txt", "w")
            f.write("1")
            if ArrivalDeclared == 0:
                ArrivalDeclared = 1
                
                #print("Green Box located, Awaiting further instructions")
            #PrintStats = 0
            ArrivalDeclared = 1
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            
            fb = FaceBox3(avg(psValuesAverage0), avg(psValuesAverage7))
              
            #print("Green: ", fb)
            if fb == 0:
                if avg(psValuesAverage2) > 69:
                    push = 1
                else:
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
            
            if fb == 1:
            
                #print("fb = 1")
                leftSpeed  = -0.7 * MAX_SPEED
                rightSpeed = 0.7 * MAX_SPEED
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
                
                        
            if fb == 2:
                #print("fb = 2")
                leftSpeed  = 0.7 * MAX_SPEED
                rightSpeed = -0.7 * MAX_SPEED
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)
                
              
            if OtherRobotArrival() == 1 and ArrivedAtBox() == 1:
                #row = ["Blue: ", str(robot.getTime())]
                #csvwriter.writerow(row)
                if Colour == 2:
                    with open('BlueTimes.csv', 'a') as the_file:
                        the_file.writelines(str(robot.getTime()))
                        the_file.writelines('\n')
                if ArrivedAtBox() == 1:  
                    f = open(ColourText + ".txt", "w")
                    f.write("1")    
                #Field Position_trans_field = robot.getField("translation")
                #print(Position_trans_field)
                
                leftMotor.setVelocity(0.5*MAX_SPEED)
                rightMotor.setVelocity(0.5*MAX_SPEED)
                state = 3
                
                #reset = 1
                #state = 0
                #if Colour == 2:
                    #robot.simulationReset()

    elif state == 3:
        #print("Colour: ", Colour)
        if PrintStats == 1:
            print("State: 3, Colour--: ", ColourText, ", ", ArrivedAtBox())
            for p in range(len(camera.getImageArray())):
                print(p, " - ", IsPixelBox(p), " ", camera.getImageArray()[p][0])
        FaceBox2()
        if BoxInFrame() == 0:
            f = open(ColourText + ".txt", "w")
            f.write("0")
            state = 0
        
                   
    if PrintStats == 1:
        print("State: ", state, " | Camera reading: ", image, " | ", leftMotor.getVelocity(), ", ", leftMotor.getVelocity(), " L/R", DirectionToFaceBox())
        #print(" ", gps.getValues())
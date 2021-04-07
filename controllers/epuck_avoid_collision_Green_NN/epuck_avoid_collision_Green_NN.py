from math import sqrt
from controller import Robot, DistanceSensor, Motor, Supervisor, Node, Camera, Field, GPS
import numpy as np
import deap, nnfs, os, time, csv, sys, random
import pandas as pd
import tensorflow
import keras
from keras.models import Sequential
from keras.layers import Dense
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import OneHotEncoder


Colour = 0
PrintStats = 0
pixelCountSet = 15
shortenCamera = 0
numberOfRobots = 3
# time in [ms] of a simulation step
TIME_STEP = 32

MAX_SPEED = 6.28

TurningSpeed = 0.5
MovingSpeed = 1
leftSpeed = 0
rightSpeed = 0

##Bigger the number, the closer it will get to obstacles
DistanceValue = 78
# create the Robot instance.
robot = Robot()
camera = Camera("camera")
camera.enable(100)


with open('cNN.csv', newline='') as f:
    reader = csv.reader(f)
    data = list(reader)

NNList = []
data = data[0]
for i in range(len(data)):
    NNList.append(float(data[i]))
#print(NNList)

# Neural network
model = Sequential()
model.add(Dense(5, input_dim=22, activation="relu"))
model.add(Dense(5, activation="relu"))
model.add(Dense(2, activation="softmax"))
model.summary()

#print("1, ", model.layers[0].get_weights())

print(2)
myWeightsL1 = NNList[0:110]
myWeightsL2 = NNList[110:135]
myWeightsL3 = NNList[135:145]
print(3)
myBiasesL1 = NNList[145:150]
myBiasesL2 = NNList[150:155]
myBiasesL3 = NNList[155:157]
print(4)
weights_1 = np.reshape(myWeightsL1, (22, 5))
weights_2 = np.reshape(myWeightsL2, (5, 5))
weights_3 = np.reshape(myWeightsL3, (5, 2))
print(5)
newdata_layer1 = np.array([weights_1, myBiasesL1])
newdata_layer2 = np.array([weights_2, myBiasesL2])
newdata_layer3 = np.array([weights_3, myBiasesL3])
print(6)
print(newdata_layer1)
print(model.layers[0].get_weights())
print(7)
model.layers[0].set_weights(newdata_layer1)
model.layers[1].set_weights(newdata_layer2)
model.layers[2].set_weights(newdata_layer3)

print(len(NNList))

weightCount = 0
for layers in model.layers:
    weightCount += len(layers.get_weights())
    #print(len(layers.get_weights()))
print("=======================")
print("2, ", model.layers[0].get_weights()[0])
#print(weightCount)

#print(robot.getDevice(robot.getName()))

#gps = GPS("gps")
#gps.enable(100)
    
# initialize devices
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3',
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
    #print(i)
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
    leftSpeed  = MovingSpeed * MAX_SPEED
    rightSpeed = MovingSpeed * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = TurningSpeed * MAX_SPEED
        rightSpeed = -TurningSpeed * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  = -TurningSpeed * MAX_SPEED
        rightSpeed = TurningSpeed * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    pass
# ----------------------------------------------------------
def BoxFound():
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = MovingSpeed * MAX_SPEED
    rightSpeed = MovingSpeed * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  = TurningSpeed * MAX_SPEED
        rightSpeed = -TurningSpeed * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  = -TurningSpeed * MAX_SPEED
        rightSpeed = TurningSpeed * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    pass
# ----------------------------------------------------------
def IsPixelBox(x):
    pixelIsBox = 1
    image = camera.getImageArray()
    if shortenCamera == 1:
        while len(image) > pixelCountSet:
            image.pop(len(image)-1)
            image.pop(0)
    
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
    image = camera.getImageArray()
    if shortenCamera == 1:
        while len(image) > pixelCountSet:
            image.pop(len(image)-1)
            image.pop(0)
        
    PixelCount = int(len(image))
    middlePixelIndex = int((PixelCount - 1)/2)
    
    if IsPixelBox(middlePixelIndex) == 0:
        for p in range(middlePixelIndex):
            if IsPixelBox(p) > IsPixelBox(PixelCount-1-p):
                direction = -1
            elif IsPixelBox(p) < IsPixelBox(PixelCount-1-p):
                direction = 1
        if PrintStats == 1:
            print("ST1: ", direction)  
                     
    elif IsPixelBox(middlePixelIndex) == 1:
        for p in range(middlePixelIndex-1):
            if IsPixelBox(p) > IsPixelBox(PixelCount-1-p) and IsPixelBox(p+1) > IsPixelBox(PixelCount-1-(p+1)):
                direction = -1
            elif IsPixelBox(p) < IsPixelBox(PixelCount-1-p) and IsPixelBox(p+1) < IsPixelBox(PixelCount-1-(p+1)):
                direction = 1
        if PrintStats == 1:
            print("ST2: ", direction)  
            
    return direction

# ----------------------------------------------------------
def ArrivedAtBox():
    image = camera.getImageArray()
    if shortenCamera == 1:
        while len(image) > pixelCountSet:
            image.pop(len(image)-1)
            image.pop(0)
        
    PixelCount = int(len(image))
    middlePixelIndex = int((PixelCount - 1)/2)
    
    arrived = 1
    
    for i in range(PixelCount):
        if IsPixelBox(i) == 0:
                arrived = 0
                
    return arrived
# ----------------------------------------------------------    
def FaceBox2():

    leftSpeed  = MovingSpeed * MAX_SPEED
    rightSpeed = MovingSpeed * MAX_SPEED
    #while DirectionToFaceBox != 0:
    # modify speeds according to obstacles
    if DirectionToFaceBox() == 1:
        # turn right
        leftSpeed  = TurningSpeed * MAX_SPEED
        rightSpeed = -TurningSpeed * MAX_SPEED
    elif DirectionToFaceBox() == -1:
        # turn left
        leftSpeed  = -TurningSpeed * MAX_SPEED
        rightSpeed = TurningSpeed * MAX_SPEED

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
            leftSpeed  = -TurningSpeed * MAX_SPEED
            rightSpeed = TurningSpeed * MAX_SPEED
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
            fb = FaceBox3(psValues[0], psValues[7])
                
        if fb == 2:
            #print("fb = 2")
            leftSpeed  = TurningSpeed * MAX_SPEED
            rightSpeed = -TurningSpeed * MAX_SPEED
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
        if numberOfRobots == 1:
            checkS2 = 1
                        
        elif numberOfRobots == 3:
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
        if numberOfRobots == 2:
                file = "..\\epuck_avoid_collision_Blue\Blue.txt"
                if os.path.exists(file):
                    OtherFile = open(file, "r")
                    for element in OtherFile.read():
                        if element == "1":
                            checkS2 = 1
        elif numberOfRobots == 3:
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
        if numberOfRobots == 2:
                file = "..\\epuck_avoid_collision_Red\Red.txt"
                if os.path.exists(file):
                    OtherFile = open(file, "r")
                    for element in OtherFile.read():
                        if element == "1":
                            checkS2 = 1
        elif numberOfRobots == 3:
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
      
    image = camera.getImageArray()
    PixelCount = int(len(image))
    middlePixelIndex = int((PixelCount - 1)/2)

    if PrintStats == 1:
        image = camera.getImageArray()
        while len(image) > pixelCountSet:
            image.pop(len(image)-1)
            image.pop(0)
        print(ColourText, " Camera Array: ", image)
        
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
    #right_obstacle = avg(psValuesAverage0) > DistanceValue or avg(psValuesAverage1) > DistanceValue or avg(psValuesAverage2) > DistanceValue
    #left_obstacle = avg(psValuesAverage5) > DistanceValue or avg(psValuesAverage6) > DistanceValue or avg(psValuesAverage7) > DistanceValue

    

    if ArrivedAtBox() == 1:  
        f = open(ColourText + ".txt", "w")
        f.write("1")
        if ArrivalDeclared == 0:
            ArrivalDeclared = 1
                       
    if PrintStats == 1:
        print("State: ", state, " | Camera reading: ", image, " | ", leftMotor.getVelocity(), ", ", leftMotor.getVelocity(), " L/R", DirectionToFaceBox())
        #print(" ", gps.getValues())
        
    inputsNN = []
    inputsNN.append(OtherRobotArrival())    
    for sensorValue in psValues:
        inputsNN.append(sensorValue) 
        
    for i in range(len(image)):
        inputsNN.append(IsPixelBox(i))
        
    #ohe = OneHotEncoder()
    #inputsNN = ohe.fit_transform(inputsNN).toarray()
    inputsNN = np.reshape(inputsNN, (-1, 22))
    
    #print(inputsNN, ", ", len(inputsNN))
    
    speeds = model.predict(inputsNN)
    leftSpeed = speeds[0][0]
    rightSpeed = speeds[0][1]
    #print(speeds[0], " - ", leftSpeed, " - ", rightSpeed)
    
    if(leftSpeed < 0.3):
        leftMotor.setVelocity(MovingSpeed * MAX_SPEED)
    elif(leftSpeed > 0.7):
        leftMotor.setVelocity(-MovingSpeed * MAX_SPEED)
    else:
        leftMotor.setVelocity(0)
        
    if(rightSpeed < 0.3):
        rightMotor.setVelocity(MovingSpeed * MAX_SPEED)
    elif(rightSpeed > 0.7):
        rightMotor.setVelocity(-MovingSpeed * MAX_SPEED)
    else:
        rightMotor.setVelocity(0)
        

        
    
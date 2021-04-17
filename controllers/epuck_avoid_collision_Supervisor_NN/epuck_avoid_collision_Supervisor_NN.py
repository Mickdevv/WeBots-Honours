"""epuck_Supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

import math
from controller import Robot, DistanceSensor, Motor, Supervisor, Node, Camera, Field, GPS
import numpy as np
import deap, nnfs, os, time, csv, sys, matplotlib
import pandas as pd
import time
from random import seed
from random import random
import sklearn
import keras
import tensorflow as tf
#from keras.backend.tensorflow_backend import set_session
from deap import algorithms
from deap import base
from deap import creator
from deap import tools
import matplotlib.pyplot as plt


from numpy import random
from tensorflow import keras
from tensorflow.keras import layers
from keras.models import Sequential
from keras.layers import Dense

import datetime
#gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.2)
#sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options)

os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

print("Started")

evaluationCount = 0.0

robotNames = ["Red", "Green", "Blue"]

numberOfRobots = 3

start = time.time()

numberOfRobots = 3

TIME_STEP = 32

SimulationTimeLimit = 40

done = 0
    
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
[[-0.7,0.02,-0.45], [-0.7,0.02,-0.15], [-0.7,0.02,0.15]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]], 
[[0,0,0], [0,0,0], [0,0,0]]
]


startingPositionsPermanentTesting = [
[[1.8, 0.02, -0.14], [-0.48, 0.02, -0.58], [-2.4, 0.02, 0.02]], 
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

simulationStartTime = supervisor.getTime()

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

            for i in range(len(StartingPositionsX)):
                    if DistanceBetween([X, Z], [StartingPositionsX[i], StartingPositionsZ[i]]) < TooCloseDistance:
                        tooClose = 1
                  
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
        
        X = round(random.uniform(trans_field_South_Wall.getSFVec3f()[0] + TooCloseDistance, trans_field_North_Wall.getSFVec3f()[0] - TooCloseDistance), 2)
        Z = round(random.uniform(trans_field_West_Wall.getSFVec3f()[2] + TooCloseDistance, trans_field_East_Wall.getSFVec3f()[2] - TooCloseDistance), 2)

        for i in range(len(StartingPositionsX)):
                if DistanceBetween([X, Z], [StartingPositionsX[i], StartingPositionsZ[i]]) < TooCloseDistance:
                    tooClose = 1
                        
        while DistanceBetween([X, Z], [trans_field.getSFVec3f()[0], trans_field.getSFVec3f()[2]]) < 0.6 or tooClose == 1:
            Z = round(random.uniform(trans_field_West_Wall.getSFVec3f()[2] + TooCloseDistance, trans_field_East_Wall.getSFVec3f()[2] - TooCloseDistance), 2)
            X = round(random.uniform(trans_field_South_Wall.getSFVec3f()[0] + TooCloseDistance, trans_field_North_Wall.getSFVec3f()[0] - TooCloseDistance), 2)
            for i in range(len(StartingPositionsX)):
                if DistanceBetween([X, Z], [StartingPositionsX[i], StartingPositionsZ[i]]) < TooCloseDistance:
                    tooClose = 1

        StartingPositionsX.append(X)
        StartingPositionsZ.append(Z)
    
    trans_field_Red.setSFVec3f([StartingPositionsX[0], StartingHeight, StartingPositionsZ[0]])
    trans_field_Green.setSFVec3f([StartingPositionsX[1], StartingHeight, StartingPositionsZ[1]])
    trans_field_Blue.setSFVec3f([StartingPositionsX[2], StartingHeight, StartingPositionsZ[2]])

    return [[StartingPositionsX[0], StartingHeight, StartingPositionsZ[0]], [StartingPositionsX[1], StartingHeight, StartingPositionsZ[1]], [StartingPositionsX[2], StartingHeight, StartingPositionsZ[2]]]

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
def distanceBetween(d1, d2):

    x1 = float(d1[0])
    x2 = float(d1[1])
    z1 = float(d2[0])
    z2 = float(d2[1])
    
    distance = (((x1-x2)**2) + ((z1-z2)**2))**0.5
    return distance
    
# ---------------------------------------------------------- 
def evaluate(individual):
    
    #print(len(individual))
    
    fitness = 0
    floatList = []
    for i in range(len(individual)):
        floatList.append(float(individual[i]))
    
    
    fileGreen = "..\\epuck_avoid_collision_Green_NN\cNN.csv"
    if os.path.exists(fileGreen):
        os.remove(fileGreen)
            
    with open("..\\epuck_avoid_collision_Green_NN\cNN.csv", "w") as fileGreen:
        writer = csv.writer(fileGreen)
        writer.writerow(individual)
    
    
    
    fileRed = "..\\epuck_avoid_collision_Red_NN\cNN.csv"
    if os.path.exists(fileRed):
        os.remove(fileRed)
            
    with open("..\\epuck_avoid_collision_Red_NN\cNN.csv", "w") as fileRed:
        writer = csv.writer(fileRed)
        writer.writerow(individual)
        
        
        
    fileBlue = "..\\epuck_avoid_collision_Blue_NN\cNN.csv"
    if os.path.exists(fileBlue):
        os.remove(fileBlue)
            
    with open("..\\epuck_avoid_collision_Blue_NN\cNN.csv", "w") as fileBlue:
        writer = csv.writer(fileBlue)
        writer.writerow(individual)
    
    
    #while not robotsReady(): 
        #supervisor.step(TIME_STEP) 
    boxStartingPosition = trans_field.getSFVec3f()
    greenRobotStartingPosition = trans_field_Green.getSFVec3f()  
    redRobotStartingPosition = trans_field_Red.getSFVec3f()  
    blueRobotStartingPosition = trans_field_Blue.getSFVec3f()  
    #print("=============== ", boxStartingPosition - greenRobotstartingPosition)
    
    startEvaluation = supervisor.getTime()  
    reset = 0
    PositionsSet = 0
    Runs = 0
    times = []
    times2 = []
    numberOfRobots = 3
    supervisor.simulationReset()
    Node.restartController(supervisor.getFromDef("Red"))
    Node.restartController(supervisor.getFromDef("Green"))
    Node.restartController(supervisor.getFromDef("Blue"))
    #print("Starting positions:")
    #print([startingPositionsPermanent[0][0][0], startingPositionsPermanent[0][0][1], startingPositionsPermanent[0][0][2]])
    #print([startingPositionsPermanent[0][1][0], startingPositionsPermanent[0][1][1], startingPositionsPermanent[0][1][2]])
    #print([startingPositionsPermanent[0][2][0], startingPositionsPermanent[0][2][1], startingPositionsPermanent[0][2][2]])

    #trans_field_Red.setSFVec3f([startingPositionsPermanent[0][0][0], startingPositionsPermanent[0][0][1], startingPositionsPermanent[0][0][2]])
    #trans_field_Green.setSFVec3f([startingPositionsPermanent[0][1][0], startingPositionsPermanent[0][1][1], startingPositionsPermanent[0][1][2]])
    #trans_field_Blue.setSFVec3f([startingPositionsPermanent[0][2][0], startingPositionsPermanent[0][2][1], startingPositionsPermanent[0][2][2]])
    inFOVTime = 0    
    
    robotPositions = [[],[],[],[],[],[]]
    failed = 0
    while reset == 0:
        
        if (trans_field_Red.getSFVec3f()[0] < -1.5) and (trans_field_Green.getSFVec3f()[0] < -1.5) and (trans_field_Blue.getSFVec3f()[0] < -1.5):
            failed = 1
        if supervisor.getTime() > 10 and supervisor.getTime() < 12 and distanceBetween(trans_field_Red.getSFVec3f(), redRobotStartingPosition) + distanceBetween(trans_field_Red.getSFVec3f(), redRobotStartingPosition) + distanceBetween(trans_field_Red.getSFVec3f(), redRobotStartingPosition) < 3:
            failed = 1
        
        robotPositions[0].append(trans_field_Red.getSFVec3f()[0])
        robotPositions[1].append(trans_field_Red.getSFVec3f()[2])
        robotPositions[2].append(trans_field_Green.getSFVec3f()[0])
        robotPositions[3].append(trans_field_Green.getSFVec3f()[2])
        robotPositions[4].append(trans_field_Blue.getSFVec3f()[0])
        robotPositions[5].append(trans_field_Blue.getSFVec3f()[2])
        supervisor.step(TIME_STEP)
        
        #print(2)
        #if trans_field.getSFVec3f()[0] > 0.31 and TimeToRecord2 == 0:
            #TimeToRecord2 = round(supervisor.getTime(), 2)
        file = "..\\epuck_avoid_collision_Red_NN\FoundPixelsRed.txt"
        if os.path.exists(file):
            #print(1)
            OtherFile = open(file, "r")
            for element in OtherFile.read():
                #print("= ", element)
                inFOVTime += int(element)       
        
        file = "..\\epuck_avoid_collision_Green_NN\FoundPixelsGreen.txt"
        if os.path.exists(file):
            #print(1)
            OtherFile = open(file, "r")
            for element in OtherFile.read():
                #print("= ", element)
                inFOVTime += int(element)   
        
        file = "..\\epuck_avoid_collision_Blue_NN\FoundPixelsBlue.txt"
        if os.path.exists(file):
            #print(1)
            OtherFile = open(file, "r")
            for element in OtherFile.read():
                #print("= ", element)
                inFOVTime += int(element)   
                
        
        
        if trans_field.getSFVec3f()[0] > 1.5 or supervisor.getTime() > SimulationTimeLimit or failed == 1:
        
            fitness += supervisor.getTime() * 5
        
            distanceToCoverGreen = distanceBetween(trans_field.getSFVec3f(), greenRobotStartingPosition)
            distanceCoveredGreen = distanceBetween(trans_field.getSFVec3f(), trans_field_Green.getSFVec3f()) 
            
            distanceToCoverRed = distanceBetween(trans_field.getSFVec3f(), redRobotStartingPosition)
            distanceCoveredRed = distanceBetween(trans_field.getSFVec3f(), trans_field_Red.getSFVec3f())
            
            distanceToCoverBlue = distanceBetween(trans_field.getSFVec3f(), blueRobotStartingPosition)
            distanceCoveredBlue = distanceBetween(trans_field.getSFVec3f(), trans_field_Blue.getSFVec3f())
            
            avgDistanceToCover = (distanceToCoverRed + distanceToCoverBlue + distanceToCoverGreen) / 3
            avgDistanceCovered = (distanceCoveredRed + distanceCoveredBlue + distanceCoveredGreen) / 3
            #How much closer the robot ended up to the box
            fitness += 10 * avgDistanceCovered / avgDistanceToCover
            #How much closer the box got to its objectuive
            fitness += (1.5 - trans_field.getSFVec3f()[0] + 0.1) * 200 / (1.5 - boxStartingPosition[0]) 
            fitness += inFOVTime/100 #For how long was the box in view
            
            if failed == 1:
                fitness += 200
            
            #print(robotPositions)
            print("Distance fitnesses: ", 10 * avgDistanceCovered / avgDistanceToCover, " | inFOVTime fitness: ", inFOVTime/100)
            print("Box moving fitness: ", (1.5 - trans_field.getSFVec3f()[0] + 0.1) * 200 / (1.5 - boxStartingPosition[0]))
            print("Fitness: ", fitness)
            
            written = 0
            for j in range(generationNum * populationLimit):
                filePos = "..\\epuck_avoid_collision_Supervisor_NN\Positions" + str(j) + ".csv"
                if not os.path.exists(filePos) and written == 0:
                    written = 1
                    with open(filePos, "w") as file:
                        writer = csv.writer(file)
                        for i in range(len(robotPositions)):
                            writer.writerow(robotPositions[i])
                
            return fitness,
            reset = 1

           
# ---------------------------------------------------------- 
def generateIndividual(creator, numGenes):
    individual = creator(np.zeros(numGenes))
    #individual.append(0)
    for i in range(numGenes):
        gene = random.uniform(-geneLimit, geneLimit)
        individual[i] = gene
    return individual
# ---------------------------------------------------------- 
def generatePopulation():
    population = []
    for i in range(populationLimit):
        population.append(generateIndividual())
    return population,
# ---------------------------------------------------------- 
def mutate(individual):
    for i in range(len(individual)):
        randomNumber = random.uniform(0,200)
        if (randomNumber < mutateProbability):
            individual[i] += mutateChange
        elif (randomNumber > mutateProbability and randomNumber < (mutateProbability*2)):
            individual[i] -= mutateChange
    return individual,
# ----------------------------------------------------------   
#Swaps the current gene with another random one
def mutateSwap(individual):	
    for i in range(len(individual)):		
        randomNum = random.randint(0,200)			
        if (randomNum < swapProbability): 				
            r = random.randint(0, len(individual))
            temp = individual.chromosome[i]
            individual.chromosome[i] = individual[r]
            individual[r] = temp	
    return individual,
# ---------------------------------------------------------- 
def OnePCX(parent1, parent2):
    randomNum = random.randint(0,len(parent1))
    child = []
    child.append(parent1[0:randomNum])
    child.append(parent2[randomNum:len(parent1)])
    return child,
    
# ---------------------------------------------------------- 	
def selection(pop, returnIndividualsCount):
    parents = []
    for i in range(len(pop)):
        parents.append(pop[random.randint(0,len(pop))])
    return parents,
# ---------------------------------------------------------- 
def main():
    # choose a population size: e.g. 200

    population = populationLimit
    pop = toolbox.population(n=population)

    # keep track of the single best solution found
    hof = tools.HallOfFame(1)

    #create a statistics object: we can log what ever statistics 
    #we want using this. We use the numpy Python library
    #to calculate the stats and label them with convenient labels
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean)
    stats.register("std", np.std)
    stats.register("min", np.min)
    stats.register("max", np.max)

    # run the algorithm: we need to tell it what parameters to use
    # cxpb = crossover probability; mutpb = mutation probability; ngen = number of iterations
    pop, log = algorithms.eaSimple(pop, toolbox, cxpb=0.6, mutpb=mutateProbability, ngen=generationNum,
                                   stats=stats, halloffame=hof, verbose=True)
    print(population)
    return pop, log, hof
# ---------------------------------------------------------- 	
#EA stuff
generationNum = 100
numGenes = 127
geneLimit = 4.0
populationLimit = 20
pop = []
totalEvaluations = generationNum * populationLimit

mutateProbability = 0.3
mutateChange = 0.2
mutateSwapProbability = 20

# define the fitness class and create an individual class
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)
# create a toolbox
toolbox = base.Toolbox()
# USE THIS LINE IF YOU WANT TO USE THE CUSTOM INIT FUNCTION
toolbox.register("individual", generateIndividual, creator.Individual, numGenes)
#  a population consist of a list of individuals
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# register all operators we need with the toolbox
toolbox.register("evaluate", evaluate)
toolbox.register("mate", tools.cxTwoPoint)
#toolbox.register("mate", OnePCX)
toolbox.register("mutate", mutate)
toolbox.register("select", tools.selTournament, tournsize=4)

for j in range(generationNum):
    filePos = "..\\epuck_avoid_collision_Supervisor_NN\Positions" + str(j) + ".csv"
    if os.path.exists(filePos):
        os.remove(filePos)

#-----------------------------------------------------------
while supervisor.step(TIME_STEP) != -1 and done == 0:
    
    x=0
    #Run the EA and save the best NN
    if(x == 0):
        start = time.time()
        pop, log, hof = main()
        # extract the best fitness
        best = hof[0].fitness.values[0]
        print(best)
        print(log)
        file = "..\\NNBest.csv"
        if os.path.exists(file):
            os.remove(file)
        with open("..\\NNBest.csv", "w") as file:
            writer = csv.writer(file)
            writer.writerow(hof[0])
        timeTaken = (time.time() - start)
        print("Time taken: ", str(datetime.timedelta(seconds=timeTaken)))
        
        # code for plotting
        
        gen = log.select("gen")
        fit_max = log.select("max")
        fit_min = log.select("min")
        fit_avg = log.select("avg")
        
        fig, ax1 = plt.subplots()
        
        line1 = ax1.plot(gen, fit_max, "b-", label="max Fitness", color="r")
        line2 = ax1.plot(gen, fit_min, "b-", label="min Fitness", color="b")
        line3 = ax1.plot(gen, fit_avg, "b-", label="avg Fitness", color="g")
        ax1.set_xlabel("Generations")
        ax1.set_ylabel("Fitness", color="b")
        for tl in ax1.get_yticklabels():
            tl.set_color("b")
        ax1.set_ylim(0, 200)
        
        lns = line1 + line2 + line3
        labs = [l.get_label() for l in lns]
        ax1.legend(lns, labs, loc="center right")
        
        plt.show()
        
    #Test the best EA from the last run
    elif(x == 1):
        with open("..\\NNBest.csv", newline='') as f:
            reader = csv.reader(f)
            data = list(reader)
        NNList = []
        data = data[0]
        for i in range(len(data)):
            NNList.append(float(data[i]))
        evaluate(NNList)
            
    done = 1
    
    print("========================= Done =========================")
    
    #supervisor.simulationSetMode(0)
            #print(pop)
    
    
    


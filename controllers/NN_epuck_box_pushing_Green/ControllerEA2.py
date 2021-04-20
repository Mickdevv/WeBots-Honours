# KNAPSACK CODE


#import some standard python packages that will be useful
import csv
import array
import random
import numpy
import matplotlib
import matplotlib.pyplot as plt


# import deap packages required
from deap import algorithms
from deap import base
from deap import creator
from deap import tools

# DEFINE A RANDOM INSTANCE OF A KNAPSACK PROBLEM
# you need to choose:
#   number of items
#   max weight for knapsack
#   a range to choose item weights from
#   a range to choose item values from 

# Create a set of items: each item is a (weight, value) 2-uple.
items = {}

NBR_ITEMS = 100
MAX_WEIGHT =300

# Create random items and store them in the items' dictionary.
for i in range(NBR_ITEMS):
    items[i] = (random.randint(1, 10), random.uniform(0, 100))
    
## SET UP THE EA

    
# define the fitness class and creare an individual class
creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", list, fitness=creator.FitnessMax)


# create a toolbox
toolbox = base.Toolbox()

# Attribute generator
toolbox.register("attr_bool", random.randint, 0, 1)
toolbox.register("attr_int", random.randint, 0, 100)

#an individual consists of repeated genes of type "attr_bool"  - we specify 100 genes
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_int, 100)

#  a population consist of a list of individuals
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

with open('output.csv','wb') as result_file:
    wr = csv.writer(result_file, dialect='excel')
# DEFINE FITNESS FOR KNAPSACK
# fitness function definition

def evalKnapsack(individual):
    weight = 0.0
    value = 0.0
    for item in range(NBR_ITEMS):
        if (individual[item]==1):
            weight += items[item][0]
            value += items[item][1]
    if  weight > MAX_WEIGHT:
        return 0,          # Bags that are overweight get a fitness of 0
    return  value,

# register all operators we need with the toolbox
toolbox.register("evaluate", evalKnapsack)
toolbox.register("mate", tools.cxOnePoint)
toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)
toolbox.register("select", tools.selTournament, tournsize=3)

# main function

def main():
    
    # choose a population size: e.g. 200
    pop = toolbox.population(n=200)
    
    # keep track of the single best solution found
    hof = tools.HallOfFame(1)
 
    # create a statistics object: we can log what ever statistics we want using this. We use the numpy Python library
    # to calculate the stats and label them with convenient labels
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", numpy.mean)
    stats.register("std", numpy.std)
    stats.register("min", numpy.min)
    stats.register("max", numpy.max)
    
    # run the algorithm: we need to tell it what parameters to use
    # cxpb = crossover probability; mutpb = mutation probability; ngen = number of iterations
    pop, log = algorithms.eaSimple(pop, toolbox, cxpb=1.0, mutpb=0.05, ngen=200, 
                                   stats=stats, halloffame=hof, verbose=False)
    
    return pop, log, hof


##############################
# run the main function 
pop, log, hof = main()

##############################


best = hof[0].fitness.values[0]   # best fitness found is stored at index 0 in the hof list


# look in the logbook to see what generation this was found at

max = log.select("max")  # max fitness per generation stored in log


    
for i in range(200):  # set to ngen
    
    fit = max[i]
    if fit == best:
        break       
        
print("max fitness found is %s at generation %s" % (best, i))
 

# code for plotting

gen = log.select("gen")
fit_max = log.select("max")
fit_min = log.select("min")
fit_avg = log.select("avg")

fig, ax1 = plt.subplots()
line1 = ax1.plot(gen, fit_max, "b-", label="max Fitness", color="r")
line2 = ax1.plot(gen, fit_min, "b-", label="min Fitness", color="b")
line3 = ax1.plot(gen , fit_avg, "b-", label="avg Fitness", color="g")
ax1.set_xlabel("Generations")
ax1.set_ylabel("Fitness", color="b")
for tl in ax1.get_yticklabels():
    tl.set_color("b")

    
lns = line1+line2+line3
labs = [l.get_label() for l in lns]
ax1.legend(lns, labs, loc="center right")

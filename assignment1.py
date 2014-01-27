#Brendan McOmber
#CS206 Bongard
#Assignment 1 Final 1/27/2014

import numpy
import matplotlib.pyplot as plt
from scipy import*

def MatrixCreate(r,c): #create a matrix with r rows and c columns
    "Set every entry to 0"
    mat = zeros((r, c))
    return mat

import random
def MatrixRandomize(mat): #create a matrix with random entries from 0 to 1
    "randomize every number in the matrix"
    for i in range(0, size(mat)):
        random.seed()
        newRand = random.random()
        mat[0, i] = newRand
    return mat

def Fitness(mat): #returns mean value of all elements in matrix
    "Sum values and return the result"
    total = 0
    for i in range(0, size(mat)):
        current = mat[0, i]
        total = total + current
    average = divide(total, size(mat))
    return average
    

def MatrixPerturb(p, n): #makes a copy of the parent vector p then replaces elements based on probability
    "With probability n the element's value is replaced with a new random()"
    child = copy(p)
    for k in range(0, size(child)):
        if( random.random()<n):
            random.seed()
            x = random.random()
            child[0, k] = x
    return child

def PlotVectorAsLine(vec): #plots the given vector as a line
    "Plot the fitness of the algorithm over time"
    plt.plot(vec[0, :])
    plt.xlabel("Generation")
    plt.ylabel("Fitness") 


#Create and store the Matrices for assignment figures A-C

#fits is for plotting one run of hill climber for fig A
fits = MatrixCreate(1, 5000)

#Genes is for the pixelated graph of 5000 runs of hill climber for fig C 
Genes = MatrixCreate(50, 5000)

#Start the hill climber algorithm
parent = MatrixCreate(1,50)
parent = MatrixRandomize(parent)
parentFitness = Fitness(parent)

for currentGeneration in range(0,5000):
    child = MatrixPerturb(parent,0.05)
    childFitness = Fitness(child)
    if ( childFitness > parentFitness ):
        parent = child
        parentFitness = childFitness

    #Store parentFitness after each run in fits
    fits[0, currentGeneration] = parentFitness

    #store all 50 random values of the parent in one column for all 5000 parents
    for k in range(0, 50):
        Genes[k, currentGeneration] = parent[0, k]

#Show vector fits over time for fig A
PlotVectorAsLine(fits)
plt.show()

#Run the hill climber algorithm 5 times, each time storing the vectors as fits2 and plotting
for i in range(0,5):
    parent2 = MatrixCreate(1,50)
    parent2 = MatrixRandomize(parent2)
    parentFitness2 = Fitness(parent2)
    fits2 = MatrixCreate(1, 5000)
    for currentGeneration2 in range(0,5000):
        child2 = MatrixPerturb(parent2, 0.05)
        childFitness2 = Fitness(child2)
        if ( childFitness2 > parentFitness2 ):
            parent2 = child2
            parentFitness2 = childFitness2
        fits2[0, currentGeneration2] = parentFitness2
    #Plot vector from one run of hill climber
    PlotVectorAsLine(fits2)
plt.show()

#Plot Genes
plt.imshow(Genes, cmap = plt.cm.gray, aspect='auto',interpolation='nearest')
plt.xlabel('Generation')
plt.ylabel('Genes')
plt.show()



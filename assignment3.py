#Brendan McOmber
#CS206 Bongard
#Assignment 3

import numpy
import matplotlib.pyplot as plt
from scipy import*
import random

def MatrixCreate(row,col): #create a matrix with r rows and c columns
    "Set every entry to 0"
    mat = zeros((row, col))
    return mat

def VectorCreate(col): #create a matrix with r rows and c columns
    "Set every entry to 0"
    vec = zeros((col), dtype='f')
    return vec


def MatrixRandomize(mat): #create a matrix with random entries from 0 to 1
    "randomize every number in the matrix"
    for i in range(0, mat.shape[0]):
        for j in range(0, mat.shape[1]):
            random.seed()
            mat[i,j] = random.uniform(-1,1)
    return mat

def MatrixPerturb(mat, n): #makes a copy of the parent vector p then replaces elements based on probability
    "With probability n the element's value is replaced with a new random()"
    child = copy(mat)
    for i in range(0, mat.shape[0]):
        for j in range(0, mat.shape[1]):
            if( random.random()<n):
                random.seed()
                child[i, j]=random.random()
    return child

def plotNetwork(neuronValues):
    "Show changing neurons over time (print neuronValues)"
    plt.imshow(neuronValues, cmap = plt.cm.gray, aspect='auto',interpolation='nearest')
    plt.xlabel('Neurons')
    plt.ylabel('Time Stamp')
    plt.show()


def Update(neurons, synapses, row):
    for j in range(0, 10):
        temp = 0
        for k in range(0, 10):
            temp = temp + synapses[j,k]*neurons[row-1, k]
        if(temp<0):
            temp = 0
        elif(temp > 1):
            temp = 1
        neurons[row, j] = temp       
    return neurons

def MeanDistance(v1, v2):
    total = 0
    for i in range(0, size(v1)):
        total = total+ abs(v1[i]-v2[i])
    total = total/size(v1)
    return total

def Fitness(parent): #return a single value that indicated the fitness of a neural networ
    "Based on the synaptic weights"
    neuronValues = MatrixCreate(10, 10)
    for i in range(0, neuronValues.shape[1]):
        neuronValues[0, i] = .5
    for k in range(1, neuronValues.shape[0]):
        neuronValues = Update(neuronValues, parent, k)
    actualNeuronValues = neuronValues[9, :]
    desiredNeuronValues = VectorCreate(10)
    for j in range(0, 10, 2):
        desiredNeuronValues[j]=1
    distance = MeanDistance(actualNeuronValues, desiredNeuronValues)
    return ((1- distance), neuronValues)

def Fitness2(parent): #return a single value that indicated the fitness of a neural networ
    "Based on the synaptic weights"
    neuronValues = MatrixCreate(10, 10)
    for i in range(0, neuronValues.shape[1]):
        neuronValues[0, i] = .5
    for k in range(1, neuronValues.shape[0]):
        neuronValues = Update(neuronValues, parent, k)
    actualNeuronValues = neuronValues[9, :]
    desiredNeuronValues = VectorCreate(10)
    for j in range(0, 10, 2):
        desiredNeuronValues[j]=1
    diff=0.0
    for i in range(0,9):
        for j in range(0,9):
            diff=diff + abs(neuronValues[i,j]-neuronValues[i,j+1])
            diff=diff + abs(neuronValues[i+1,j]-neuronValues[i,j])
    diff=diff/(2*9*9)
    return (diff, neuronValues)

def PlotVectorAsLine(vec): #plots the given vector as a line
    "Plot the fitness of the algorithm over time"
    plt.plot(vec[0, :])
    plt.xlabel("Generation")
    plt.ylabel("Fitness") 


#Start the hill climber algorithm
parent = MatrixCreate(10, 10)
parent = MatrixRandomize(parent)
parentFitness, parentValues = Fitness2(parent)
pFitness = VectorCreate(5000)

plotNetwork(parentValues)


for currentGeneration in range(0,5000):
    child = MatrixPerturb(parent,0.05)
    childFitness, childValues = Fitness2(child)
    if ( childFitness > parentFitness ):
        parent = child
        parentFitness = childFitness
        parentValues = childValues
    pFitness[currentGeneration]=parentFitness
    #print parentFitness, childFitness

plotNetwork(parentValues)

#plt.plot(pFitness)

plt.show()




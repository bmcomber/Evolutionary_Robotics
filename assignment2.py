#Brendan McOmber
#CS206 Bongard
#Assignment 2 1/28/2013

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
    for i in range(0, mat.shape[1]):
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


#Start Assignment 2

neuronValues = MatrixCreate(50, 10)

#Represents initial values of neurons
neuronValues = MatrixRandomize(neuronValues)

#Stores Positions of the neurons
neuronPositions = MatrixCreate(2, 10)

angle = 0.0
angleUpdate = 2 * pi /10
for i in range(0,10):
    x = sin(angle)
    y = cos(angle)
    angle = angle + angleUpdate
    neuronPositions[0, i] = x
    neuronPositions[1, i] = y

#print neuronPositions
plt.plot(neuronPositions[0, :], neuronPositions[1,:], 'ko', markerfacecolor = [1, 1, 1], markersize = 18)
plt.show()

#Create and fill synapses with random values between -1 and 1
synapses = MatrixCreate(10, 10)
for i in range(0, 10):
    for j in range(0, 10):
        synapses[i, j] = random.uniform(-1,1)

for i in range(0, 10):
    for j in range(0, 10):
        if(synapses[i, j] < 0):
            color = '.8'
        else:
            color = 'k'
        w = int(10*abs(synapses[i,j]))+1
        plt.plot([neuronPositions[0, i], neuronPositions[0, j]], [neuronPositions[1, i], neuronPositions[1, j]], color, linewidth = w)
plt.show()

#print synapses

def Update(neurons, synaps, row):
    for j in range(0, 10):
        temp = 0
        for k in range(0, 10):
            temp = temp + synaps[j,k]*neurons[row-1, k]
        if(temp<0):
            temp = 0
        elif(temp > 1):
            temp = 1
        neurons[row, j] = temp       
    return neurons

neuronValues = MatrixRandomize(neuronValues)
for i in range(1, 50):
    neuronValues = Update(neuronValues, synapses, i)

#Show changing neurons over time (print neuronValues)
plt.imshow(neuronValues, cmap = plt.cm.gray, aspect='auto',interpolation='nearest')
plt.xlabel('Neurons')
plt.ylabel('Time Stamp')
plt.show()
                





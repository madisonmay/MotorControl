import matplotlib.pyplot as plt
from matplotlib.axes import Axes
import mpl_toolkits.mplot3d.axes3d as p3
from math import radians, cos, sin, tan

#2D data points
real = []

#Division by axis for 2D plot
x, y = [], []


#load in csv
def load(filename):
    f = open(filename, 'r')
    for line in f:
        time, pos  = line.strip().split(',')
        real.append((time, pos))

if __name__ == '__main__':
    #data file
    load("step.txt")  

    #2d Scatter Plot
    for xy in real:
        x.append(xy[0])
        y.append(xy[1])

    fig2 = plt.figure()
    plt.xlabel('Time (ms)')
    plt.ylabel('Position (degrees)')
    plt.plot(x, y, 'r')
    fig2.show()

    #wait for user input
    input()
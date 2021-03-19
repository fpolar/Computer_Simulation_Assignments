import numpy as np 
from matplotlib import pyplot as plt 

def readRotationValues():

	file1 = open('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\131_04-dance.amc', 'r')
	Lines = file1.readlines()
	 
	root_count = 0
	lfemur_count = 0
	count = 0

	for line in Lines:
	    count += 1
	    if 'lfemur' in line:
	    	lfemur_count += 1
	    	x_rot = line.split(" ")[1]
	    	print("LFemur {}: {}".format(lfemur_count, x_rot))
	    # print("Line{}: {}".format(count, line))
	    # print("Line{}: {}".format(count, line.strip()))

def G1():
	x = np.arange(600, 800) 
	yI = g1_in_XRots
	yLE = g1_LE_XRots
	yBE = g1_BE_XRots
	plt.title("Left Femur X-Axis Rotation for linear Euler vs Bezier Euler interpolation") 
	plt.xlabel("frame") 
	plt.ylabel("Left Femur X-Axis Rotation in degrees") 
	plt.plot(x,y) 
	plt.show()

readRotationValues()
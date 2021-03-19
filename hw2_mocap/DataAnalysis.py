import numpy as np 
from matplotlib import pyplot as plt 

def readRotationValues(path, bone, dim):

	# file1 = open('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\131_04-dance.amc', 'r')
	file1 = open(path, 'r')
	Lines = file1.readlines()
	 
	bone_count = 0
	count = 0
	out = []

	for line in Lines:
	    if bone in line:
	    	# bone_count += 1
	    	val = line.split(" ")[dim]
	    	out.append(val)
	    	# print("{} {}: {}".format(bone, bone_count, val))
	    # print("Line{}: {}".format(count, line))
	    # print("Line{}: {}".format(count, line.strip()))

	return out

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

lfemurIN = readRotationValues('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\131_04-dance.amc', 'lfemur', 1)
print(lfemurIN)
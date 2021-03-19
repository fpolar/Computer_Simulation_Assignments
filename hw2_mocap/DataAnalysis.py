import numpy as np 
from matplotlib import pyplot as plt 

# path is the path to the amc file
# bone is the name of the bone you are trying to read
# dim is the index of the dimension you are searching for, 1-based, 
# x is 1, y is 2, z is 3, etc
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
	    	out.append(float(val))
	    	# print("{} {}: {}".format(bone, bone_count, val))
	    # print("Line{}: {}".format(count, line))
	    # print("Line{}: {}".format(count, line.strip()))

	return out

lfemurIN = readRotationValues('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\131_04-dance.amc', 'lfemur', 1)
lfemurLE = readRotationValues('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\D_LE_20.amc', 'lfemur', 1)
lfemurBE = readRotationValues('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\D_BE_20.amc', 'lfemur', 1)
lfemurLQ = readRotationValues('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\D_LQ_20.amc', 'lfemur', 1)
lfemurBQ = readRotationValues('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\D_BQ_20.amc', 'lfemur', 1)

G1 = True
G2 = True
G3 = True
G4 = True

if(G1):
	x = np.arange(600, 800) 
	yI = lfemurIN[600:800]
	yLE = lfemurLE[600:800]
	yBE = lfemurBE[600:800]
	plt.title("Left Femur X-Axis Rotation for linear Euler vs Bezier Euler interpolation") 
	plt.xlabel("frame") 
	plt.ylabel("Left Femur X-Axis Rotation in degrees") 
	plt.plot(x,yI) 
	plt.plot(x,yLE) 
	plt.plot(x,yBE) 
	plt.legend(["Input", "Linear Euler", "Bezier Euler"])
	plt.show()

if(G2):
	x = np.arange(600, 800) 
	yI = lfemurIN[600:800]
	yLQ = lfemurLQ[600:800]
	yBQ = lfemurBQ[600:800]
	plt.title("Left Femur X-Axis Rotation for linear vs Bezier Quaternion interpolation") 
	plt.xlabel("frame") 
	plt.ylabel("Left Femur X-Axis Rotation in degrees") 
	plt.plot(x,yI) 
	plt.plot(x,yLQ) 
	plt.plot(x,yBQ) 
	plt.legend(["Input", "Linear Quaternion", "Bezier Quaternion"])
	plt.show()

rootIN = readRotationValues('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\131_04-dance.amc', 'root', 3)
rootLE = readRotationValues('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\D_LE_20.amc', 'root', 3)
rootBE = readRotationValues('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\D_BE_20.amc', 'root', 3)
rootLQ = readRotationValues('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\D_LQ_20.amc', 'root', 3)
rootBQ = readRotationValues('csci520-assignment2-startercode\\IDE-starter\\VS2017\\Debug\\D_BQ_20.amc', 'root', 3)

if(G3):
	x = np.arange(200, 500) 
	yI = lfemurIN[200:500]
	yLE = lfemurLE[200:500]
	yBE = lfemurBE[200:500]
	plt.title("Root Z-Axis Rotation for linear vs Bezier Euler interpolation") 
	plt.xlabel("frame") 
	plt.ylabel("Root Z-Axis Rotation in degrees") 
	plt.plot(x,yI) 
	plt.plot(x,yLE) 
	plt.plot(x,yBE) 
	plt.legend(["Input", "Linear Euler", "Bezier Euler"])
	plt.show()

if(G4):
	x = np.arange(200, 500) 
	yI = lfemurIN[200:500]
	yLQ = lfemurLQ[200:500]
	yBQ = lfemurBQ[200:500]
	plt.title("Root Z-Axis Rotation for linear vs Bezier Quaternion interpolation") 
	plt.xlabel("frame") 
	plt.ylabel("Root Z-Axis Rotation in degrees") 
	plt.plot(x,yI) 
	plt.plot(x,yLQ) 
	plt.plot(x,yBQ) 
	plt.legend(["Input", "Linear Quaternion", "Bezier Quaternion"])
	plt.show()
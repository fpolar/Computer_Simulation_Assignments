<Please submit this file with your solution.>

CSCI 520, Assignment 1
Frank Pol

================

The executable is fpolar_hw1_jello/starter-csci520-assign1Bin/Debug/jello.exe
to run it I cd into fpolar_hw1_jello/starter-csci520-assign1Bin/Debug and run "jello.exe ../../world/gravity.w"

<Description of what you have accomplished>

Most of my work was in the computeAcceleration function, where a for loop runs through every vertice in the jello world and 
calculates structural force, shear force, bend force, collision force, and the force coming from the force field points being read in.
It does this for structural, shear, and bend springs by going to every neighbor for every vertex and calculating the spring forces depending on which neighbor is where.
The collision force is calculated through the penalty method, setting the force equal to a spring using the colliding particles position and adding/subtracting 2 to bring it back within the cube.
For the force field, at each vertice, 8 values are read from it so that they can be all interpolated to the actual position of the current vertex.

Outside of the compute acceleration function, I also defined a few functions in jello.h to help with point calculations, following the pattern already set by the starter code.
Also, since spring forces with hook and damping need to be calculated so often, I created a function called computeSpringForce that does all that if possible, given a springs data.

I also want to clarify that I did create many of my own worlds to test with, but
'make' on createWorld wasn't working on my windows machine, where I did most of the work for this and from where I submitted, 
so I ended up creating and testing my other worlds on another machine

<Also, explain any extra credit that you have implemented.>

This is an assignment I would normally love to sink my teeth into and explore, but I did not experiment with much extra credit on this assignment. 
I went from working 2 jobs to 3 this semester and it's been a bit of a challenge time management wise these first few weeks.
But I think I am getting the hang of it, so I'm excited to go above and beyond on the next assignment :)

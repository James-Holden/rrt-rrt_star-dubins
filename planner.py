import sys
import numpy as np
import math
import random
from pyflann import *
import scipy.spatial 
import time

#Node representation 
class Node(object): 
	def __init__(self, parent, location):
		self.parent = parent
		self.location = location
		self.x = location[0]
		self.y = location[1]
		
#sets depth and returns it 
def evalDepth(node):
	depth = 0
	cur = node
	while cur.parent != None:
		depth += scipy.spatial.distance.cdist([cur.parent.location], [cur.location])[0][0]
		cur = cur.parent
	return depth

#tests the depth of node given a parent (both nodes)
def testDepth(node, givenParent):
	depth = scipy.spatial.distance.cdist([node.location], [givenParent.location])[0][0]
	depth += evalDepth(givenParent)
	return depth
 




#returns valid non-goal sample
def validSample():
	samplex = random.uniform(0, numColumns)
	sampley = random.uniform(0, numRows)
	_sample = np.array((samplex, sampley))
	worldItem = entries[int(sampley)][int(samplex)]
	if(worldItem == "#"):
		return validSample()
	else:
		return _sample

#given node, return all NODES within a given radius 
def near(flann, node):
	flann.build_index(locations)
	indices = flann.nn_radius(node.location, radius)[0]
	ret = list()
	for index in indices:
		ret.append(nodes[index])
	return ret 


#given node, return nearest if it lies within radius,
#node if near returns empty  
def nearestInRadius(flann, node):
	try:
		near(flann, node)[0]
	except IndexError:
		return None 


#returns a sample
#.05 goal bias
def sample():
	rand = random.randint(1,1000)
	if(rand == 20):
		return goal
	else:
		return validSample()


		
#DDA algorithm used to see what spaces of the map the line 
#will go though to later check for collision w/ # 
#adapted from template @ https://www.geeksforgeeks.org/dda-line-generation
def trajectory(given, sample):
	dx = sample[0] - given[0]
	dy = sample[1] - given[1]

	if(dx > dy):
		k = abs(dx)
	else:
		k = abs(dy)

	if k == 0: k = 1
	k = k*500
	xinc = dx/k
	yinc = dy/k

	ret = []
	x = given[0]
	y = given[1]
	ret.append((x,y))
	for i in range(1,int(k+1)):
		x = x + xinc
		y = y + yinc
		ret.append((x,y))
	return ret

#checks points along trajectory for collions
#given list of points as floats  
def validPath(trajectory):
	first = trajectory[0]
	last = trajectory[-1]
	# print first, last
	if(first == last):
		return False
	if(abs((last[1]-first[1])/(last[0]-first[0]) > 9)):
		return False
	for point in trajectory:
		discrX = int(point[0])
		discrY = int(point[1])
		worldItem = entries[discrY][discrX]
		if(worldItem == "#"):
			return False

	return True


#take target x, y, return closest node using flann  
def nearest(x, y):
	flann.build_index(locations)
	dataset = locations
	testset = np.array((x,y))
	result,dists = flann.nn(locations,testset,1,algorithm="kmeans",branching=32, iterations=7, checks=16)
	return nodes[result[0]]

#Rapidly Exploring Random Tree
#returns solution node 
def rrt():
	global locations
	while True:
		_sample = sample()
		_closestNode = nearest(_sample[0], _sample[1])
		if validPath(trajectory((_closestNode.x,_closestNode.y), _sample)):
			new = Node(_closestNode, _sample)
			nodes.append(new)
			locations = np.concatenate((locations, [new.location]), axis=0)
			if new.x == goal[0] and new.y == goal[1]:
				return new


#actively rewire a neighborhood (nodes) considering center node 
def rewire(neighborhood, center):
	for neighbor in neighborhood:
		if testDepth(neighbor, center) < evalDepth(neighbor):
			if validPath(trajectory(neighbor.location, center.location)):
				neighbor.parent = center


#Rapidly Exploring Random Tree w/ rewiring 
def rrt_star():
	global locations
	while True:
		_sample = sample()
		_closestNode = nearest(_sample[0], _sample[1])
		new = Node(_closestNode, _sample)
		neighborhood = near(flann, new)#
		neighborhood.sort(key=lambda x: evalDepth(x), reverse=False)
		if len(neighborhood) == 0: continue 
		parent = neighborhood[0]
		if validPath(trajectory((parent.x,parent.y), new.location)):
			new.parent = parent
			nodes.append(new)
			locations = np.concatenate((locations, [new.location]), axis=0)
			rewire(neighborhood, new)
			if new.x == goal[0] and new.y == goal[1]:
				return new	



if __name__ == "__main__":
	#parse args 
	try:
		algorithm = eval(sys.argv[1])
		radius = float(sys.argv[2])
	except IndexError:
		#defaults 
		algorithm = rrt
		radius = 5
	
	#libs
	flann = FLANN()
	np.set_printoptions(threshold=np.nan)

	#stdin processing 
	lines = sys.stdin.read().splitlines()
	entries = []
	numColumns = int(lines[0])
	numRows = int(lines[1])
	for line in lines[2:numRows+2]:
		entries.insert(0, line)

	#setup
	start = float(lines[numRows+2]), float(lines[numRows+3])
	root = Node(None, np.array((start[0], start[1])))
	goal = np.array(( float(lines[numRows+4]) , float(lines[numRows+5]) ))
	nodes = list()
	nodes.append(root)
	locations = np.array([root.location])

	#Path planning 
	startTime = time.time()
	solution = algorithm()
	duration = time.time() - startTime

	# #reporting for analysis 
	# print evalDepth(solution) , ',' , duration


	# solution reporting for visualization
	solutionTrajectory = list()
	cur = solution
	while cur != None:
		solutionTrajectory.append((cur.x, cur.y))
		cur = cur.parent
	print len(solutionTrajectory)
	for point in reversed(solutionTrajectory):
		print point[0], point[1] 

	#other explored nodes 
	print len(nodes) - 1
	for node in nodes:
		if(node.parent != None):
			print node.parent.x, node.parent.y, node.x, node.y 


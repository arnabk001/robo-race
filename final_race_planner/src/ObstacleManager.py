import cv2
import math
import numpy
import Utils

class ObstacleManager(object):

	def __init__(self, mapMsg, car_width, car_length, collision_delta):
		self.map_info = mapMsg.info
		self.mapImageGS = numpy.array(mapMsg.data, dtype=numpy.uint8).reshape(
			(mapMsg.info.height, mapMsg.info.width, 1))
		print("mapheight = ", mapMsg.info.height, "mapwidth = ", mapMsg.info.width)
		# Retrieve the map dimensions
		height, width, channels = self.mapImageGS.shape
		self.mapHeight = height
		self.mapWidth = width
		self.mapChannels = channels

		# Binarize the Image
		self.mapImageBW = 255 * numpy.ones_like(self.mapImageGS, dtype=numpy.uint8)
		self.mapImageBW[self.mapImageGS == 0] = 0

		# Obtain the car length and width in pixels
		self.robotWidth = int(car_width / self.map_info.resolution + 0.5)
		self.robotLength = int(car_length / self.map_info.resolution + 0.5)
		self.collision_delta = collision_delta

	# Check if the passed config is in collision
	# config: The configuration to check (in meters and radians)
	# Returns False if in collision, True if not in collision
	def get_state_validity(self, config):

		# Convert the configuration to map-coordinates -> mapConfig is in pixel-space
		mapConfig = Utils.world_to_map(config, self.map_info)

		# ---------------------------------------------------------
		# Return true or false based on whether the robot's configuration is in collision
		# Use a square to represent the robot, return true only when all points within
		# the square are collision free
		#
		# Also return false if the robot is out of bounds of the map
		#
		# Although our configuration includes rotation, assume that the
		# square representing the robot is always aligned with the coordinate axes of the
		# map for simplicity
		# ----------------------------------------------------------
		if mapConfig[1] >= self.mapHeight or mapConfig[0] >= self.mapWidth :
			print ("Robot Out of bounds")
			return False


		# Check collision for each corner of the robot square
		cornerdist = int (math.sqrt((self.robotWidth**2)+(self.robotLength**2))/2.0)
		
		cornerMin = [mapConfig[0]-cornerdist,mapConfig[1]-cornerdist]
		cornerMax = [mapConfig[0]+cornerdist,mapConfig[1]+cornerdist]

		if cornerMin[1] >= self.mapHeight or cornerMin[0] >= self.mapWidth :
			print ("Corner Out of bounds")
			return False

		if cornerMax[1] >= self.mapHeight or cornerMax[0] >= self.mapWidth :
			print ("Corner Out of bounds")
			return False

		# print(cornerMin[1], cornerMax[1], cornerMin[0], cornerMax[0])

		BW = self.mapImageBW[cornerMin[1]:cornerMax[1],cornerMin[0]:cornerMax[0]]
            			
		if BW.sum ():
			print ("path in collision")
			return False
		else:
			return True

	# Discretize the path into N configurations, where N = path_length / self.collision_delta
	#
	# input: an edge represented by the start and end configurations
	#
	# return three variables:
	# list_x - a list of x values of all intermediate points in the path
	# list_y - a list of y values of all intermediate points in the path
	# edgeLength - The euclidean distance between config1 and config2
	def discretize_edge(self, config1, config2):
		list_x, list_y = [], []
		edgeLength = 0
		# -----------------------------------------------------------
		# YOUR CODE HERE
		# -----------------------------------------------------------
		x1, y1 = numpy.array(config1, dtype=numpy.float32)
    	x2, y2 = numpy.array(config2, dtype=numpy.float32)
	
		edgeLength = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

		# Discretize the edge
		num_points = int(edgeLength / self.collision_delta)
		list_x = numpy.linspace(x1, x2, num_points)
		list_y = numpy.linspace(y1, y2, num_points)


		return list_x, list_y, edgeLength


	# Check if there is an unobstructed edge between the passed configs
	# config1, config2: The configurations to check (in meters and radians)
	# Returns false if obstructed edge, True otherwise
	def get_edge_validity(self, config1, config2):
		# -----------------------------------------------------------
		# YOUR CODE HERE
		#
		# Check if endpoints are obstructed, if either is, return false
		# Find path between two configs by connecting them with a straight line
		# Discretize the path with the discretized_edge function above
		# Check if all configurations along path are obstructed
		# -----------------------------------------------------------
 
		if not self.get_state_validity(config2):
			return False

		list_x, list_y, _ = self.discretize_edge(config1, config2)

    		# Check collision for each configuration along the edge
    		for x, y in zip(list_x, list_y):
        		if not self.get_state_validity([x, y]):
            			return False

		return True


# Write Your Test Code For Debugging
#if __name__ == '__main__':
#	return
	# Write test code here!

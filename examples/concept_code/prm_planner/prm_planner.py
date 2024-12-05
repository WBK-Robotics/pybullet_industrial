import numpy as np
import pybullet as p
from robot_base import RobotBase
import ompl


class PRMPlanner:

	def __init__(self, robot: RobotBase, num_samples, num_neighbors):
		self.robot = robot
		self.num_samples = num_samples
		self.num_neighbors = num_neighbors
		self.ignored_collisions = {}
		self.prm = ompl.PRM()	
	
	def detect_collision(self):
		p.peformCollisionDetetection()
		contact_points = p.getContactPoints()
		collisions = [] #choosing suitable data structure
		if contact_points: 
			for contact_point in contact_points:
				if not contact_point in self.ignored_collisions:  
					collisions.append(contact_point)
		return collisions

	def is_collision_free(self):
		collisions = self.detect_collision()
		if collisions == []:
			return True
		else:
			return False


	def create_sample_nodes(self):
		# creates n-dimensional sample node within joint_limit
		size = self.robot.get_nuber_of_moveable_joints # needs to be implemented
		lower_limit, upper_limit = self.robot.get_joint_limits() # needs to be implemented
		samples = [] #choosing suitable data structure
		for _ in range(self.num_samples):
			sample = []
			for joint in self.robot.joints:
				val = ompl.sampling(lower_limit[joint], upper_limit[joint])
				sample[joint] = val
			samples.append(sample)
		return samples
					
	def validate_sample_nodes(self,nodes):
		valid_nodes = []
		for node in nodes:
			self.robot.set_robot(node)
			if self.is_collision_free():
				valid_nodes.append(node)
		return valid_nodes

	def collision_free_edge(self,node1 , node2):
		self.robot.set_robot(node1)
		self.robot.set_joint_position(node2)
		for _ in range(30):
			p.StepSimulation()
			if not self.is_collision_free():
				return False
		return True
		
				
	def create_prm(self):
		#1st phase: creating valid samples
		random_nodes = self.create_samples() # creates Samples in Configuration Space
		valid_nodes = self.validate_samples(random_nodes) # build free-c-space samples
		self.prm.add_nodes(valid_nodes)
		#2nd phase: connecting samples
		for node in self.prm.nodes:
			neighbors = ompl.find_nearest_neigbors(node, self.num_neighbors)
			for neighbour in neighbors:
				if self.is_collision_free_edge(node,neighbour):
					self.prm.add_edge(node, neighbour)
		return self.prm.get_graph()

	def create_collision_free_path(self, start_config, end_config):
		#Check if start & end configuration are valid
		self.robot.set_robot(start_config)
		start_config_valid = self.is_collision_free()
		if not start_config_valid:
			raise ValueError("start config is not valid")
		self.robot.set_robot(end_config)
		end_config_valid = self.is_collision_free()
		if not end_config_valid:
			raise ValueError("end config is not valid")
		
		#Create collision-free path
		self.prm.add_to_roadmap(start_config)
		self.prm.add_to_roadmap(end_config)
		collision_free_path = self.prm.build_path(start_config, end_config)	
		return collision_free_path
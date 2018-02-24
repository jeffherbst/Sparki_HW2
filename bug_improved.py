import pygame
import sys
import time
import math
import argparse
import numpy as np
from RobotLib.FrontEnd import *
from RobotLib.IO import *
from Robot import Robot
from OccupancyMap import OccupancyMap
from RobotLib.Math import *

class MyFrontEnd(FrontEnd):
	def __init__(self,sparki,omap_path):
		self.omap = OccupancyMap(omap_path)
		FrontEnd.__init__(self,self.omap.width,self.omap.height)
		self.sparki = sparki
		self.robot = Robot()

		# center robot
		self.robot.x = self.omap.width*0.5
		self.robot.y = self.omap.height*0.5
		
		# zero out robot velocities
		self.robot.lin_vel = 0
		self.robot.ang_vel = 0
		
		#is robot at goal
		self.goalReached = True

		#PID control constants
		self.K_linear = .5
		self.K_angular = .5
		self.distanceThreshold = 1
		self.switchToBugDist = 30

	def mouseup(self,x,y,button):
		self.robot.set_map_goal(x,y)
		self.goalReached = False

	def draw(self,surface):
		# draw occupancy map
		self.omap.draw(surface)

		# draw robot
		self.robot.draw(surface)
	
	def get_sonar_distance(self):
		# get sonar distance
		if self.sparki.port == '':
			# simulate rangefinder
			T_sonar_map = self.robot.get_robot_map_transform() * self.robot.get_sonar_robot_transform()
			self.robot.sonar_distance = self.omap.get_first_hit(T_sonar_map)
		else:
			# read rangefinder
			self.robot.sonar_distance = self.sparki.dist
		
		print('Sonar Distance (%d)'%(self.robot.sonar_distance))
	
	def update(self,time_delta):
		self.get_sonar_distance()
		
		"""
		PID navigation
		Calculate angle to goal set linear and angular vel
		Switch to bug if sonar reading is less than tolarance
			Bug is defined as turning setting angular velocity to left 
		"""

		if not self.goalReached:
			if self.switchToBugDist > self.robot.sonar_distance and not self.robot.sonar_distance == 0: #bug if too close
				self.robot.ang_vel = .3
				self.robot.lin_vel = 0
			elif self.robot.goalXr > self.distanceThreshold or self.robot.goalXr < -self.distanceThreshold or self.robot.goalYr > self.distanceThreshold or self.robot.goalYr < -self.distanceThreshold:
				theta = math.atan2(self.robot.goalYr,self.robot.goalXr)
				self.robot.lin_vel = self.K_linear * self.robot.goalXr
				self.robot.ang_vel = self.K_angular * theta * self.sweep(time_delta)
			else:
				self.robot.ang_vel = 0
				self.robot.lin_vel = 0
				self.goalReached = True


			
		# calculate motor settings
		left_speed, left_dir, right_speed, right_dir = self.robot.compute_motors()
		
		# send command
		#self.sparki.send_command(left_speed,left_dir,right_speed,right_dir)
		
		# update robot position
		self.robot.update(time_delta)

		# update robot goal
		self.robot.set_robot_goal()

	def sweep(self, time_delta):
		"""
			Sweeps to the left and the right to see
			where the farthest sonar distance lies
			Returns -1 for left and 1 for right
		"""
		#Temporarily store linear and angular velocity
		tempLin = self.robot.lin_vel
		tempAng = self.robot.ang_vel
		
		#Set linear velocity to 0 temporarily
		self.robot.lin_vel = 0
		
		#Set angular velocity to negative values 
		self.robot.ang_vel = -self.K_angular
		
		#Update the robot's angle
		self.robot.update(time_delta)
		
		#Save the sonar distance
		self.get_sonar_distance()
		left_distance = self.robot.sonar_distance
		
		#Change the angular velocity to twice the positive value
		#to get to the other "side" of the angle
		self.robot.ang_vel = 2 * self.K_angular
		
		#Update the robot's angle
		self.robot.update(time_delta)
		
		#Save the sonar distance
		self.get_sonar_distance()
		right_distance = self.robot.sonar_distance
		
		#Set the angular velocity to 1 * negative angular velocity
		#to bring the robot back to its original position
		self.robot.ang_vel = -self.K_angular
		
		#Update the robot's angle
		self.robot.update(time_delta)
		
		#Reset the sonar distance
		self.get_sonar_distance()
		
		#Reload the initial velocity values
		self.robot.lin_vel = tempLin
		self.robot.ang_vel = tempAng
		
		#Check which distance is smaller
		if left_distance == 0 or right_distance == 0:
			if left_distance <= right_distance:
				return 1
			else:
				return -1
		else:
			if left_distance < right_distance:
				return 1 * (left_distance / right_distance)
			elif left_distance > right_distance:
				return -1 * (right_distance / left_distance)
			else:
				return 1

def main():
	# parse arguments
	parser = argparse.ArgumentParser(description='template')
	parser.add_argument('--omap', type=str, default='map.png', help='path to occupancy map image file')
	parser.add_argument('--port', type=str, default='', help='port for serial communication')
	args = parser.parse_args()

	with SparkiSerial(port=args.port) as sparki:
		# make frontend
		frontend = MyFrontEnd(sparki,args.omap)
	
		# run frontend
		frontend.run()

if __name__ == '__main__':
	main()

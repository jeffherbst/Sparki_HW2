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
        self.K_linear = 1
        self.K_angular = 1
        self.distanceThreshold = 1

    def mouseup(self,x,y,button):
        self.robot.set_map_goal(x,y)
        self.goalReached = False

    def draw(self,surface):
        # draw occupancy map
        self.omap.draw(surface)

        # draw robot
        self.robot.draw(surface)
    
    def update(self,time_delta):
        """
        # get sonar distance
        if self.sparki.port == '':
            # simulate rangefinder
            T_sonar_map = self.robot.get_robot_map_transform() * self.robot.get_sonar_robot_transform()
            self.robot.sonar_distance = self.omap.get_first_hit(T_sonar_map)
        else:
            # read rangefinder
            self.robot.sonar_distance = self.sparki.dist
        """
        """
        PID navigation
        Calculate angle to goal set linear and angular vel
        """

        if not self.goalReached:
            if self.robot.goalXr > self.distanceThreshold or self.robot.goalXr < -self.distanceThreshold or self.robot.goalYr > self.distanceThreshold or self.robot.goalYr < -self.distanceThreshold:
                theta = math.atan2(self.robot.goalYr,self.robot.goalXr)
                self.robot.lin_vel = self.K_linear * self.robot.goalXr
                self.robot.ang_vel = self.K_angular * theta
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

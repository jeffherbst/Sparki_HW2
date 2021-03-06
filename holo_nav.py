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
        
        # bool check if robot is at goal
        self.goalReached = True

    def mouseup(self,x,y,button):
        """ Gets mouse location on click prints goal passes to robot class """
        
        print('mouse clicked at %d, %d'%(x,y)) 
        self.robot.set_map_goal(x,y)
        self.goalReached = False
        
    
    def draw(self,surface):
        # draw occupancy map
        self.omap.draw(surface)

        # draw robot
        self.robot.draw(surface)
    
    def update(self,time_delta):
        # get sonar distance
        if self.sparki.port == '':
            # simulate rangefinder
            T_sonar_map = self.robot.get_robot_map_transform() * self.robot.get_sonar_robot_transform()
            self.robot.sonar_distance = self.omap.get_first_hit(T_sonar_map)
        else:
            # read rangefinder
            self.robot.sonar_distance = self.sparki.dist
        
        """ Holonomic Navigation
            If 1 < Y or Y < -1 then turn
            elseif x > 0 then move forward
            else goal reached stop motors
        """
        if not self.goalReached:
            if self.robot.goalYr > 1.:
                self.robot.ang_vel = .3
                self.robot.lin_vel = 0
            elif self.robot.goalYr < -1.:
                self.robot.ang_vel = -.3
                self.robot.lin_vel = 0
            elif self.robot.goalXr > 0.:
                self.robot.ang_vel = 0
                self.robot.lin_vel = 10
            else:
                self.goalReached = True
                self.robot.ang_vel = 0
                self.robot.lin_vel = 0

        # calculate motor settings
        left_speed, left_dir, right_speed, right_dir = self.robot.compute_motors()
        
        # send command
        self.sparki.send_command(left_speed,left_dir,right_speed,right_dir)
        
        # update robot position
        self.robot.update(time_delta)

        #update goal position
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

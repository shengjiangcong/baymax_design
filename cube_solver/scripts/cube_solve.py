#!/usr/bin/env python
# -*- coding: utf-8 -*-

import kociemba
import rospy, sys
import thread, copy
import string
from collections import deque
from cube_solver.srv import *

class switch(object):
    def __init__(self, value):
    	self.value = value
    	self.fall = False
      
    def __iter__(self):
    	"""Return the match method once, then stop"""
    	yield self.match
    	raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

def callback(req):
    cube_string = rospy.get_param("/cube_string")
    solve_string = kociemba.solve(cube_string)
    excute_queue = deque()
    for i in solve_string.split():
              #print(i)  
          excute_queue.append(i)
        
    while (len(excute_queue) != 0):
          pop_string = excute_queue.popleft()
          for case in switch(pop_string):
              if case('U'):
                 print "U面顺时针旋转90"
              elif case('D'):
                 print "D面顺时针旋转90"
              elif case('L'):
                 print "L面顺时针旋转90"
              elif case('R'):
                 print "R面顺时针旋转90"
              elif case('F'):
                 print "F面顺时针旋转90"
              elif case('B'):
                 print "B面顺时针旋转90"
              elif case("U'"):
                 print "U面逆时针旋转90"
              elif case("D'"):
                 print "D面逆时针旋转90"
              elif case("L'"):
                 print "L面逆时针旋转90"
              elif case("R'"):
                 print "R面逆时针旋转90"
              elif case("F'"):
                 print "F面逆时针旋转90"
              elif case("B'"):
                 print "B面逆时针旋转90"
              elif case("U2"):
                 print "U面旋转180"
              elif case("D2"):
                 print "D面旋转180"
              elif case("L2"):
                 print "L面旋转180"
              elif case("R2"):
                 print "R面旋转180"
              elif case("F2"):
                 print "F面旋转180"
              elif case("B2"):
                 print "B面旋转180"
              elif case():
                 print "参数错误"
    return solve_string


def Cube_solver():
    rospy.init_node('cube_solver')     
    s = rospy.Service('kociemba', solve_list, callback)
    rospy.spin()

if __name__ == "__main__":
    Cube_solver()

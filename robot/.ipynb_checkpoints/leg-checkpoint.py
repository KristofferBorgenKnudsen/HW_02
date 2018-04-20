
import odrive.core
import time
import math

import numpy as np
import matplotlib.pyplot as plt



class leg:
    """
    This is our first class in class :)
    
    We will define a leg class to interface with the leg and standardize 
    the kind of operations we want to perform
    
    """
    
    # Classes start with a constructor that can take in initial parameters. At
    # a minimum it takes in a copy of itself (python... weird). The constructor
    # is one place we can define and initialize class variables
    
    def __init__(self):
        
        drv = connect_to_controller()
        
        # motor controller parameters
        encoder2angle = 2048*4
        
        # home angles 
        home_L = 0
        home_R = 0  
        
        # leg geometry
        l1 = 1
        l2 = 1
        l_base = 1
        
        # positioning gains
        
    def connect_to_controller(self):
        return odrive.core.find_any(consider_usb=True, consider_serial=False, printer=print)
    
    def set_home(self):
        
        
    def get_joints(self):
        
        
    def get_xy(self):
        
    
    def move_home(self):
        
    
    def set_foot_pos(self, x, y):
        
    
    def set_joint_angs(self, theta1, theta2):
        
    
    def record_trajectory(self):
        
        
    def move_trajectory(self, xx, yy):
        
    
    def visualize(self):
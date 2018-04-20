import odrive.core
import time
import math

import numpy as np
import matplotlib.pyplot as plt

# For symbolic processing
import sympy
from sympy import symbols
from sympy import sin, cos, asin, acos, pi
from sympy.utilities.lambdify import lambdify
from sympy import Matrix


class Leg:
    """
    This is our first class in class :)

    We will define a leg class to interface with the leg and standardize 
    the kind of operations we want to perform

    """

    #### Variables outside the init function are constants of the class
    # leg geometry
    l1 = 1  # NEED TO UPDATE units of cm
    l2 = 1  # NEED TO UPDATE units of cm
    l_base = 1  # NEED TO UPDATE units of cm

    # motor controller parameters
    encoder2angle = 2048 * 4

    ### Methods
    # Classes are initiated with a constructor that can take in initial parameters. At
    # a minimum it takes in a copy of itself (python... weird). The constructor
    # is one place we can define and initialize class variables

    def __init__(self, simulate = False):
        """
        This is the constructor for the leg class. Whenever you make a new leg
        this code will be called. We can optionally make a leg that will simulate
        the computations without needing to be connected to the ODrive
        """

        self.simulate = simulate

        # make the option to code without having the odrive connected
        if self.simulate == False:
            self.drv = self.connect_to_controller()
            self.m0 = self.drv.motor0  # easier handles to the motor commands
            self.m1 = self.drv.motor1

        else:
            self.drv = None

        # home angles
        self.joint_0_home = 0
        self.joint_1_home = 0

        # current positions
        m0_pos, m1_pos = self.get_joint_pos()
        self.joint_0_pos = m0_pos
        self.joint_1_pos = m1_pos

        # We will compute the jacobian and inverse just once in the class initialization.
        # This will be done symbolically so that we can use the inverse without having
        # to recompute it every time
        print('here2')
        self.J = self.compute_jacobian()
        # self.Jacobian_inv = self.Jacobian.inv

    def connect_to_controller(self):
        """
        Connects to the motor controller
        """
        drv = odrive.core.find_any(consider_usb=True, consider_serial=False)

        if drv is None:
            print('No controller found')
        else:
            print('Connected!')
        return drv

    ###
    ### Motion functions
    ###
    def get_joint_pos(self):
        """
        Get the current joint positions and store them in self.joint_0_pos and self.joint_1_pos in degrees.
        Also, return these positions using the return statement to terminate the function
        """
        # if simulating exit function
        if self.simulate == True:
            return (-1, -1)


        #
        # Your code here
        #

        return (self.joint_0_pos, self.joint_1_pos)

    def set_home(self):
        """
        This function updates the home locations of the motors so that 
        all move commands we execute are relative to this location. 
        """
        # if simulating exit function
        if self.simulate == True:
            return

        #
        # Your code here
        #


    def set_joint_pos(self, theta0, theta1, vel0=0, vel1=0, curr0=0, curr1=0):
        """
        Set the joint positions in units of deg, and with respect to the joint homes.
        We have the option of passing the velocity and current feedforward terms.
        """
        # if simulating exit function
        if self.simulate == True:
            return

        #
        # Your code here
        #


    def move_home(self):
        """
        Move the motors to the home position
        """
        # if simulating exit function
        if self.simulate == True:
            return


        #
        # Your code here
        #

    def set_foot_pos(self, x, y):
        """
        Move the foot to position x, y. This function will call the inverse kinematics 
        solver and then call set_joint_pos with the appropriate angles
        """
        # if simulating exit function
        if self.simulate == True:
            return

        #
        # Your code here
        #

    def move_trajectory(self, tt, xx, yy):
        """
        Move the foot over a cyclic trajectory to positions xx, yy in time tt. 
        This will repeatedly call the set_foot_pos function to the new foot 
        location specified by the trajectory.
        """
        # if simulating exit function
        if self.simulate == True:
            return

        #
        # Your code here
        #

    ###
    ### Leg geometry functions
    ###
    def compute_internal_angles(self, theta_0, theta_1):
        """
        Return the internal angles of the robot leg 
        from the current motor angles
        """

        #
        # Your code here
        #

        return (alpha_0, alpha_1)

    def compute_jacobian(self):
        """
        This function implements the symbolic solution to the Jacobian.
        """

        # initiate the symbolic variables
        theta0_sym, theta1_sym, alpha0_sym, alpha1_sym = symbols(
            'theta1_sym theta2_sym alpha1_sym alpha2_sym', real=True)

        #
        # Your code here that solves J as a matrix
        #

        return J

    def inverse_kinematics(self, x, y):
        """
        This function will compute the required theta_0 and theta_1 angles to position the 
        foot to the point x, y. We will use an iterative solver to determine the angles.
        """

        #
        # Your code here that solves J as a matrix
        #

        return (theta_0, theta_1)

    ###
    ### Visualization functions
    ###
    def draw_leg(ax=False):
        """
        This function takes in the four angles of the leg and draws
        the configuration
        """

        theta1, theta2 = self.joint_0_pos, self.joint_1_pos
        link1, link2, width = self.l1, self.l2, self.l_base

        alpha1, alpha2 = self.compute_internal_angles()

        def pol2cart(rho, phi):
            x = rho * np.cos(phi)
            y = rho * np.sin(phi)
            return (x, y)

        if ax == False:
            ax = plt.gca()
            ax.cla()

        ax.plot(-width / 2, 0, 'ok')
        ax.plot(width / 2, 0, 'ok')

        ax.plot([-width / 2, 0], [0, 0], 'k')
        ax.plot([width / 2, 0], [0, 0], 'k')

        ax.plot(-width / 2 + np.array([0, link1 * cos(theta1)]), [0, link1 * sin(theta1)], 'k')
        ax.plot(width / 2 + np.array([0, link1 * cos(theta2)]), [0, link1 * sin(theta2)], 'k')

        ax.plot(-width / 2 + link1 * cos(theta1) + np.array([0, link2 * cos(alpha1)]), \
                link1 * sin(theta1) + np.array([0, link2 * sin(alpha1)]), 'k');
        ax.plot(width / 2 + link1 * cos(theta2) + np.array([0, link2 * cos(alpha2)]), \
                np.array(link1 * sin(theta2) + np.array([0, link2 * sin(alpha2)])), 'k');

        ax.plot(width / 2 + link1 * cos(theta2) + link2 * cos(alpha2), \
                np.array(link1 * sin(theta2) + link2 * sin(alpha2)), 'ro');

        ax.axis([-2, 2, -2, 2])
        ax.invert_yaxis()
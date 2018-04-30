# Import the robot class we are developing
import odrive.core
import time
import math

import numpy as np
import matplotlib.pyplot as plt

# For symbolic processing
import sympy
from sympy import symbols, diff
from sympy import sin, cos, asin, acos, atan, pi
from sympy.utilities.lambdify import lambdify
from sympy import Matrix

# initiate the symbolic variables
theta_0_sym, theta_1_sym = symbols("""
theta_0_sym, theta_1_sym """, real=True)

# Make a new leg variable which is a robot.Leg class

class Leg:
    """
    This is our first class in class :)

    We will define a leg class to interface with the leg and standardize 
    the kind of operations we want to perform

    """

    #### Variables outside the init function are constants of the class
    # leg geometry
    l1 = 7  # NEED TO UPDATE units of cm, 7
    l2 = 14  # NEED TO UPDATE units of cm, 14
    l_base = 7.5  # NEED TO UPDATE units of cm, 7.5

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

        self.simulate = simulate #simulate, True

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
        self.Jacobian, self.FK = self.compute_jacobian()
        print(self.Jacobian.shape)
        #self.J_inv = self.Jacobian.pinv()
        
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

        # Your code here
        m0 = self.m0.encoder.pll_pos
        m1 = self.m1.encoder.pll_pos
    
        self.joint_0_pos = m0*2*np.pi/self.encoder2angle
        self.joint_1_pos = m1*2*np.pi/self.encoder2angle

        return (self.joint_0_pos, self.joint_1_pos)

    def set_home(self):
        """
        This function updates the home locations of the motors so that 
        all move commands we execute are relative to this location. 
        """
        # if simulating exit function
        if self.simulate == True:
            return

        # Your code here
        self.joint_0_home, self.joint_1_home = self.get_joint_pos()
        
    def set_joint_pos(self, theta0, theta1, vel0=0, vel1=0, curr0=0, curr1=0):
        """
        Set the joint positions in units of deg, and with respect to the joint homes.
        We have the option of passing the velocity and current feedforward terms.
        """
        # if simulating exit function
        if self.simulate == True:
            return

        # Your code here
        self.joint_0_pos = theta0-self.joint_0_home 
        self.joint_1_pos = theta1-self.joint_1_home 
        setpoint0 = self.joint_0_pos*self.encoder2angle/(2*np.pi)
        setpoint1 = self.joint_1_pos*self.encoder2angle/(2*np.pi)
        
        self.m0.set_pos_setpoint(self.joint_0_pos, vel0, curr0)
        self.m1.set_pos_setpoint(self.joint_1_pos, vel1, curr1)
        
       
    def move_home(self):
        """
        Move the motors to the home position
        """
        # if simulating exit function
        if self.simulate == True:
            return

        # Your code here
        self.joint_0_pos = self.joint_0_home
        self.joint_1_pos = self.joint_1_home 
        
        self.m0.set_pos_setpoint(self.joint_0_pos, 0.0, 0.0)
        self.m1.set_pos_setpoint(self.joint_1_pos, 0.0, 0.0)
        

    def set_foot_pos(self, x, y):
        """
        Move the foot to position x, y. This function will call the inverse kinematics 
        solver and then call set_joint_pos with the appropriate angles
        """
        # if simulating exit function
        if self.simulate == True:
            return

        # Your code here
        theta_0, theta_1 = self.inverse_kinematics(x, y)
        self.set_joint_pos(theta_0, theta_1, vel0=0, vel1=0, curr0=0, curr1=0)
        

    def move_trajectory(self, tt, xx, yy):
        """
        Move the foot over a cyclic trajectory to positions xx, yy in time tt. 
        This will repeatedly call the set_foot_pos function to the new foot 
        location specified by the trajectory.
        """
        # if simulating exit function
        if self.simulate == True:
            return

        # Your code here
        
        length = len(xx)
        
        for i in range(0,length):
            time.sleep(tt/length)
            a=xx[i] 
            b=yy[i]
          
            self.set_foot_pos(a, b)  
            

    ###
    ### Leg geometry functions
    ###
    def compute_internal_angles(self, theta_0_sym, theta_1_sym):
        """
        Return the internal angles of the robot leg 
        from the current motor angles
        """
        
        #theta_0_sym = theta_0_sym*np.pi/180
        #theta_1_sym = theta_1_sym*np.pi/180
        
        # Your code here
        d_2 = pow(self.l_base,2) + pow(self.l1,2) - 2*self.l_base*self.l1*cos(pi-theta_0_sym)
        d = sympy.sqrt(d_2)
        beta = acos((pow(self.l_base,2) + d_2 - pow(self.l1,2))/(2*self.l_base*d))
      
        a = (self.l1*cos(theta_1_sym-beta)-d)/self.l2
        b = (self.l1*sin(theta_1_sym-beta))/self.l2
        q4 = atan(-a/b)+0.5*acos((2-pow(a,2)-pow(b,2))/2)
        q2 = atan(-a/b)-0.5*acos((2-pow(a,2)-pow(b,2))/2)
        
        alpha_1 = q2 + beta
        alpha_0 = q4 + beta
        
        return (alpha_0, alpha_1)

    def compute_jacobian(self):
        """
        This function implements the symbolic solution to the Jacobian.
        """
        alpha_0, alpha_1 = self.compute_internal_angles(theta_0_sym, theta_1_sym)

        # Your code here that solves J as a matrix
        x = self.l_base/2 + self.l1*cos(theta_0_sym) + self.l2*cos(alpha_0)
        y = self.l1*sin(theta_0_sym) + self.l2*sin(alpha_0)
        
        FK = Matrix([[x], [y]])
        
        J = FK.jacobian([theta_0_sym, theta_1_sym])
        
        print(J)
        
        return (J, FK)
    
    def inverse_kinematics(self, x, y):
        """
        This function will compute the required theta_0 and theta_1 angles to position the 
        foot to the point x, y. We will use an iterative solver to determine the angles.
        """
        # Your code here that solves J as a matrix
        J = self.Jacobian
        FK = self.FK
        
        J_current_inv = lambdify((theta_0_sym, theta_1_sym), J)
                
        
        theta_current = self.get_joint_pos()
        theta_current = np.array(theta_current).astype(np.float64)
        
        x_current = FK.subs({theta_0_sym: theta_current[0], theta_1_sym: theta_current[1]})
        
        x_target = np.array([x,y])
        x_current = np.array([x_current[0],x_current[1]])
        
        beta = 0.1
        epsilon = 0.5
                
        for kk in range(1000):
            
            x_current = FK.subs({theta_0_sym: theta_current[0], theta_1_sym: theta_current[1]})
            
            
            x_error = sympy.N(Matrix([x-x_current[0],y-x_current[1]]))
            
            if x_error.norm() < epsilon:
                break
            print(x_error.norm())
            
            J_current_inv = J_current_inv(theta_current[0], theta_current[1])
            
            J_current_inv = np.linalg.pinv(J_current_inv)

            theta_current = beta*J_current_inv*x_error + theta_current
            
        theta_0 = theta_current[0]
        theta_1 = theta_current[1]
        
        return (theta_0, theta_1)

    ###
    ### Visualization functions
    ###
    def draw_leg(self, ax=False):
        """
        This function takes in the four angles of the leg and draws
        the configuration
        """

        theta1, theta2 = self.joint_0_pos, self.joint_1_pos
        link1, link2, width = self.l1, self.l2, self.l_base

        alpha1, alpha2 = self.compute_internal_angles(theta1, theta2)

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

        ax.axis([-30, 30, -30, 30])
        ax.invert_yaxis()
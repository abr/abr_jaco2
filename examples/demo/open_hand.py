"""
Demo script, non-compliant hold position.
"""
import numpy as np
import signal
import sys
import time
import abr_control
import abr_jaco2
import threading
import os
import termios #Imports for keyboard handling
import atexit
from select import select


class KBHit: 
    '''Class for dealing with keyboard inputs''' 
    def __init__(self):
        '''Creates a KBHit object that you can call to do various keyboard things.
        '''
        # Save the terminal settings
        self.fd = sys.stdin.fileno()
        self.new_term = termios.tcgetattr(self.fd)
        self.old_term = termios.tcgetattr(self.fd)

        # New terminal setting unbuffered
        self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

        # Support normal-terminal reset at exit
        atexit.register(self.set_normal_term)

    def set_normal_term(self):
        ''' Resets to normal terminal. 
        '''
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)

    def getch(self):
        ''' Returns a keyboard character after kbhit() has been called.
        '''
        return sys.stdin.read(1)

    def kbhit(self):
        ''' Returns True if keyboard character was hit, False otherwise.
        '''
        dr,dw,de = select([sys.stdin], [], [], 0)
        return dr != []

kp = 4.0
kv = 2.0
loop_limit = 15000

# initialize our robot config for neural controllers
robot_config = abr_jaco2.robot_config(
    use_cython=True, hand_attached=False)

# NOTE: right now, in the osc when vmax = None, velocity is compensated
# for in joint space, with vmax set it's in task space

# instantiate the REACH controller for the jaco2 robot
ctrlr = abr_control.controllers.osc(
    robot_config, kp=kp, kv=kv, vmax=1.0, null_control=False)
# create signal to compensate for friction
friction = abr_jaco2.signals.friction(robot_config)

# run controller once to generate functions / take care of overhead
# outside of the main loop, because force mode auto-exits after 200ms
ctrlr.control(np.zeros(6), np.zeros(6), target_pos=np.zeros(3))
friction.generate(dq=np.zeros(6))

# create our interface for the jaco2
interface = abr_jaco2.interface(robot_config)
# connect to the jaco
interface.connect()

try:
    # move to the home position
    interface.apply_q(robot_config.home_position_start)

    kb = KBHit()

    while True:
        start = time.time()
        if kb.kbhit():
            c = kb.getch()
            if ord(c) == 99: # c
                interface.open_hand(False)
            if ord(c) == 111: # o
                interface.open_hand(True)

    kb.set_normal_term()

except Exception as e:
     print(e)

finally:
    # return back to home position
    interface.init_position_mode()
    interface.apply_q(robot_config.home_position_start)
    # close the connection to the arm
    interface.disconnect()

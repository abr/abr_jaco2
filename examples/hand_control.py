""" press 'o' to open hand and 'p' to close it

Keyboard interface for opening and closing the jaco2 hand
"""
import abr_jaco2
from abr_analyze.utils import KBHit

# initialize our robot config
robot_config = abr_jaco2.Config(
    use_cython=True, hand_attached=True)
interface = abr_jaco2.Interface(robot_config)

kb = KBHit.KBHit()

# connect to and initialize the arm
interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.START_ANGLES)

print('\'o\' : open hand')
print('\'p\' : close hand')
print('\'q\' : quit')

while 1:
    #  if keyboard hit followed by enter key detected...
    if kb.kbhit():
        # saves the value of the key pressed
        c = kb.getch()
        # if key pressed matches desired key, perform task
        # can be used to start / stop scripts, run functions,
        # provide feedback etc.
        if ord(c) == ord('q'):
            print('Goodbye')
            break
        if ord(c) == ord('o'):
            interface.open_hand(True)
        if ord(c) == ord('p'):
            interface.open_hand(False)
        else:
            print('Not a valid key')

# set the terminal back to its initial state
kb.set_normal_term()

# close the connection to the arm
interface.send_target_angles(robot_config.START_ANGLES)
interface.disconnect()

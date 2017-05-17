""" Tests the basic connection and disconnect from the Jaco2 """

import abr_jaco2

# initialize our robot config for neural controllers
robot_config = abr_jaco2.Config()
# create our interface for the jaco2
interface = abr_jaco2.Interface(robot_config)

try:
    interface.connect()

except Exception as e:
    print(e)

finally:
    interface.disconnect()

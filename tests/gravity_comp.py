"""moves jaco to various joint positions and switches to torque
mode and float controller, records joint positions. Test to 
quantify gravity compensation improvements"""
import numpy as np
import os
import time
import abr_control
import abr_jaco2

# ---------- INITIALIZATION ----------
# initialize our robot config for the ur5
robot_config = abr_jaco2.robot_config(
    regenerate_functions=False, use_cython=True,
    use_simplify=False, hand_attached=False)

# instantiate the REACH controller
ctrlr = abr_control.controllers.floating(robot_config)

# run controller once to generate functions / take care of overhead
# outside of the main loop (so the torque mode isn't exited)
ctrlr.control(np.zeros(6), np.zeros(6))

# create our VREP interface
interface = abr_jaco2.interface(robot_config)

interface.connect()
interface.init_position_mode()

# Values must be in range of -360 to 360 degrees
read_positions = np.array([
                    [250.0, 140.0, 100.0, 230.0, 40.0, 0.0],
                    [250.0, 120.0, 100.0, 230.0, 40.0, 0.0],
                    [305.0, 105.0, 105.0, 230.0, 25.0, -0.0]], dtype="float32")
calc_euclid_error = False
# ---------- MAIN BODY ----------
# Move to home position
interface.apply_q(robot_config.home_position)
#interface.disconnect()

try:
    for ii in range(0, len(read_positions)):
        # move to read position ii
        #interface.connect()
        print('Moving to read position ', ii)

        if calc_euclid_error is True:
            torque_load = np.array(np.zeros(6), dtype="float32")
            old_torque_load = np.array(np.zeros(6), dtype="float32")
            for jj in range (0, 1000):
                old_torque_load = torque_load
                interface.apply_q(read_positions[ii])    
                t_feedback = interface.get_torque_load()
                torque_load = np.array(t_feedback['torque_load'], dtype="float32")
                print("Euclid Dist: ", np.linalg.norm(torque_load - old_torque_load))

        else:
            interface.apply_q(read_positions[ii])
            t_feedback = interface.get_torque_load()
            torque_load = np.array(t_feedback['torque_load'], dtype="float32")    
            t_feedback = interface.get_torque_load()
            

        interface.init_force_mode(expected_torque = torque_load)
        loop_count = 0

        while loop_count < 2000:
            loop_count += 1

            # get arm feedback
            feedback = interface.get_feedback()
            q = (np.array(feedback['q']) % 360) * np.pi / 180.0
            dq = np.array(feedback['dq']) * np.pi / 180.0

            # generate a control signal
            u = ctrlr.control(q=q, dq=dq)

            interface.apply_u(np.array(u, dtype='float32'))      

        interface.init_position_mode()
        #interface.apply_q(robot_config.home_position)
        #interface.disconnect()

finally:
    interface.disconnect()
    print('DONE!')
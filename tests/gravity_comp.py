"""moves jaco to various joint positions and switches to torque
mode and float controller, records joint positions. Test to 
quantify gravity compensation improvements"""
import numpy as np
import os
import time
import abr_control
import abr_jaco2
# --- NAME TEST FOR SAVING ---
test_name = "baseline"


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

loop_limit = 10

# Values must be in range of -360 to 360 degrees
read_positions = np.array([
                    [250.0, 140.0, 100.0, 230.0, 40.0, 0.0],
                    [250.0, 120.0, 100.0, 230.0, 40.0, 0.0],
                    [305.0, 105.0, 105.0, 230.0, 25.0, -0.0]], dtype="float32")

joint_angles = np.zeros((6, loop_limit, len(read_positions)))

# ---------- MAIN BODY ----------
# Move to home position
interface.apply_q(robot_config.home_position)

try:
    for ii in range(0, len(read_positions)):
        # move to read position ii
        print('Moving to read position ', ii)
        interface.apply_q(read_positions[ii])
        t_feedback = interface.get_torque_load()
        torque_load = np.array(t_feedback['torque_load'], dtype="float32")    
        t_feedback = interface.get_torque_load()
            
        interface.init_force_mode()
        loop_count = 0

        while loop_count < loop_limit - 1:
            loop_count += 1

            # get arm feedback
            feedback = interface.get_feedback()
            joint_angles[:, loop_count, ii] = (np.array(feedback['q']) % 360) * np.pi / 180.0
            dq = np.array(feedback['dq']) * np.pi / 180.0

            # generate a control signal
            u = ctrlr.control(q=joint_angles[:, loop_count, ii], dq=dq)

            interface.apply_u(np.array(u, dtype='float32'))      

        interface.init_position_mode()
        #interface.apply_q(robot_config.home_position)
        #interface.disconnect()

except Exception as e:
    print(e)

finally:
    interface.apply_q(robot_config.home_position)
    interface.disconnect()
    print('Disconnected')

    # check if the directory for saving files exists
    if not os.path.exists('data/'):
        os.makedirs('data/gravity_comp/')

    elif not os.path.exists('data/gravity_comp/'):
        os.makedirs('data/gravity_comp/')

    if not os.path.exists('data/gravity_comp/%s' % test_name):
        os.makedirs('data/gravity_comp/%s' % test_name)

    print('Saving files...')
    np.savez_compressed('data/gravity_comp/%s/q_actual' % (test_name),
                            q_actual=joint_angles)
    np.savez_compressed('data/gravity_comp/%s/q_desired' % (test_name),
                            q_desired= (read_positions % 360) * np.pi / 180.0)
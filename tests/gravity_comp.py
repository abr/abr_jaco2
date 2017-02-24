"""moves jaco to various joint positions and switches to torque
mode and float controller, records joint positions. Test to 
quantify gravity compensation improvements"""
import numpy as np
import os
import time
import abr_control
import abr_jaco2
# --- NAME TEST FOR SAVING ---
test_name = "retest"

# ---------- INITIALIZATION ----------
# initialize our robot config for the ur5
robot_config = abr_jaco2.robot_config(
    use_cython=True, hand_attached=True)

# instantiate the controller

#if using osc controller
#ctrlr = abr_control.controllers.osc(robot_config, kp=2, kv=0.5, vmax=0.5)
#ctrlr.control(np.zeros(6), np.zeros(6), target_x=np.zeros(3))

#if using floating controller
ctrlr = abr_control.controllers.floating(robot_config)
ctrlr.control(np.zeros(6), np.zeros(6))

#if using dynamic_floating controller
#ctrlr = abr_control.controllers.dynamic_floating(robot_config)
#ctrlr.control(np.zeros(6), np.zeros(6))

# create our VREP interface
interface = abr_jaco2.interface(robot_config)

interface.connect()
interface.init_position_mode()

loop_limit = 2000

# Values must be in range of -360 to 360 degrees
read_positions = np.array([
                    [250.0, 140.0, 100.0, 230.0, 40.0, 0.0],
                    [250.0, 200.0, 200.0, 150.0, 40.0, 0.0],
                    [305.0, 210.0, 250.0, 100.0, 25.0, -0.0]], dtype="float32") *np.pi/180.0

joint_angles = np.zeros((6, loop_limit, len(read_positions)))
times = np.zeros((loop_limit, len(read_positions)))

# ---------- MAIN BODY ----------
# Move to home position
interface.apply_q(robot_config.home_position_start)

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
        start = time.time()
        
        while loop_count < loop_limit - 1:
            loop_count += 1

            # get arm feedback
            feedback = interface.get_feedback()
            joint_angles[:, loop_count, ii] = (np.array(feedback['q']) % 360) * np.pi / 180.0
            dq = np.array(feedback['dq']) * np.pi / 180.0

            # generate a control signal
            # osc
            #u = ctrlr.control(q=joint_angles[:, loop_count, ii], dq=dq, 
            #    target_x=[-.467, .22, .78])
            # floating and dynamic_floating
            u = ctrlr.control(q=joint_angles[:, loop_count, ii], dq=dq)

            interface.send_forces(np.array(u, dtype='float32'))      
            times[loop_count, ii] = time.time() - start

        interface.init_position_mode()
        #interface.apply_q(robot_config.home_position)
        #interface.disconnect()

except Exception as e:
    print(e)

finally:
    interface.apply_q(robot_config.home_position_end)
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
    np.savez_compressed('data/gravity_comp/%s/time' % (test_name),
                            time=times)
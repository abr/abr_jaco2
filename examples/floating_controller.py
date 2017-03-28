"""moves jaco to various joint positions and switches to torque
mode and float controller, records joint positions. Test to
quantify gravity compensation improvements"""
import numpy as np
import traceback

import abr_control
import abr_jaco2
# --- NAME TEST FOR SAVING ---

# ---------- INITIALIZATION ----------
# initialize our robot config for the ur5
robot_config = abr_jaco2.config(
    use_cython=True, hand_attached=True)

record_q = True
q_list = []
ctrlr = abr_control.controllers.floating(robot_config)
ctrlr.control(np.zeros(6), np.zeros(6))

interface = abr_jaco2.interface(robot_config)

interface.connect()
interface.init_position_mode()

init_torque_position = np.array(
    [0.0, 2.79, 2.72, 4.71, 0.0, 3.04], dtype="float32")
# ---------- MAIN BODY ----------
# Move to home position
interface.send_target_angles(init_torque_position)

try:
    interface.init_force_mode()

    while 1:
        # get arm feedback
        feedback = interface.get_feedback()
        q = np.array(feedback['q'])
        dq = np.array(feedback['dq'])

        u = ctrlr.control(q=q, dq=dq)

        interface.send_forces(np.array(u, dtype='float32'))

        if record_q is True:
            q_list.append(np.copy(q))

except Exception as e:
    print(traceback.format_exc())

finally:
    interface.init_position_mode()
    interface.send_target_angles(init_torque_position)
    interface.disconnect()
    if record_q is True:
        q_list = np.array(q_list)
        np.savez_compressed('q',q=q_list)
        # parameter_file = open('float_recorded_q', 'w')
        # parameter_file.seek(0)
        # parameter_file.truncate(0)
        # for ii in range(0,len(q_list)):
        #     parameter_file.write('%.3f %.3f %.3f %.3f %.3f %.3f\n'%
        #         (q_list[ii,0],q_list[ii,1],q_list[ii,2],
        #          q_list[ii,3],q_list[ii,4],q_list[ii,5]))
    print('Disconnected')

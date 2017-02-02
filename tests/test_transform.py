import abr_control
import abr_jaco2

# robot_config = abr_control.arms.onelink.config(
robot_config = abr_jaco2.robot_config(
    regenerate_functions=True, use_cython=False,
    use_simplify=False, hand_attached=False)

interface = abr_control.interfaces.vrep(
    robot_config=robot_config, dt=.001)
interface.connect()

name = 'EE'
try:
    # test out our orientation calculations
    while 1:
        feedback = interface.get_feedback()
        print('q: ', feedback['q'])
        xyz = robot_config.Tx(name, q=feedback['q'])

        quaternion = robot_config.orientation(name, q=feedback['q'])
        # the r means it's a relative frame
        angles = abr_control.utils.transformations.euler_from_quaternion(
            quaternion, axes='rxyz')
        # print('angles: ', np.array(angles) * 180.0 / np.pi)

        interface.set_xyz('hand', xyz)
        interface.set_orientation('hand', angles)

finally:
    # stop and reset the VREP simulation
    interface.disconnect()

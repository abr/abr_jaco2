*********
ABR Jaco2
*********

ABR_Jaco2: Force control interface for the Kinova Jaco2 with torque sensors

Installation
============

ABR_Jaco2 depends on the ABR_Control repo. The repo can be found
at https://github.com/abr/abr_control with install instructions in the README.

Additionally, ABR_Jaco2 library depends on Numpy, Sympy, CloudPickle,
Matplotlib, and Scipy, and we recommend that you install these libraries
before ABR_Jaco2. If you're not sure how to do this, we recommend using
`Anaconda <https://store.continuum.io/cshop/anaconda/>`_.
Note that installing in a clean environment will require compiling of the
dependent libraries, and will take a few minutes.

To install ABR_Jaco2, clone this repository and run::

    python setup.py install
    python setup.py develop

ABR_Jaco2 is tested to work on Python 3.4+.

Usage
=====

The ABR_Jaco2 repo is comprised of an interface to communicate with
the Kinova Jaco2, and a configuration file for use with the ABR_Control
library. The uses Kinova's low level API to allow for direct force control
of the torques being applied by each motor, as well as position control.

1) The interface class provided here communicates with the arm through the low
level RS485 API provided by Kinova. With this interface, a control mode can be
selected (either position or force), and then target angles or joint torques
can be sent to the arm.

NOTE: When using force control mode the control signal must be sent at a
frequency faster that 200ms, otherwise the arm will revert to position mode.

2) All of the required information about an arm model is kept in that arm's
config file. More information on the formatting of the config can be found in
the ABR_Control repo's README. Here we provide a configuration file for the
physical Kinova Jaco2 (as opposed to the VREP model config file provided in
the ABR_Control repo), based off of SolidWorks CAD files and empirical tuning.

3) The ABR_Control library provides implementations of controllers and path
planners that can be used on the Jaco2, with examples provided in this repo.

A script for basic control of the Jaco2 looks like::

    import abr_jaco2
    from abr_control.controllers import OSC
    
    robot_config = abr_jaco2.Config()
    interface = abr_jaco2.Interface(robot_config)
    ctrlr = OSC(robot_config)
    # instantiate things to avoid creating 200ms delay in main loop
    zeros = np.zeros(robot_config.N_LINKS)
    ctrlr.generate(q=zeros, dq=zeros, target=zeros(3))
    # run once outside main loop as well, returns the cartesian
    # coordinates of the end effector
    robot_config.Tx('EE', q=zeros)
    
    interface.connect()
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    
    target_xyz = [.57, .03 .87]  # (x, y, z) target (metres)
    interface.init_force_mode()
    
    while 1:
        # returns a dictionary with q, dq
        feedback = interface.get_feedback() 
        # ee position
        xyz = robot_config.Tx('EE', q=q, target_pos = target_xyz)
        u = ctrlr.generate(feedback['q'], feedback['dq'], target_xyz)
        interface.send_forces(u, dtype='float32')
    
        error = np.sqrt(np.sum((xyz - TARGET_XYZ[ii])**2))
    
        if error < 0.02:
            break
    
    # switch back to position mode to move home and disconnect
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
    interface.disconnect()

Examples
========

The ABR_Jaco2 repo comes with several examples that demonstrate the use of
the different controllers. To run simulations of the Jaco2 in VREP, please
see the ABR_Control repo.

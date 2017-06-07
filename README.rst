*********
ABR Jaco2
*********

ABR_Jaco2: Config and interface for Kinova Jaco2 with force sensors

Installation
============

ABR_Jaco2 depends on the ABR_Control repo. The repo can be found
at https://github.com/abr/abr_control with install instructions in the README

Additionally, ABR_Jaco2 library depends on cloudpickle, sympy, nengo, numpy,
matplotlib, and scipy, and we recommend that you install these libraries before
ABR_Jaco2.

If you're not sure how to do this, we recommend using
`Anaconda <https://store.continuum.io/cshop/anaconda/>`_.
Note that installing in a clean environment will require compiling of the
dependent libraries, and will take a few minutes.

To install ABR_Jaco2, clone this repository and run::

    python setupy.py install
    python setup.py develop

ABR_Jaco2 is tested to work on Python 3.4+.

Usage
=====

The ABR_Jaco2 repo is comprised of the config and interface to communicate with
the Kinova Jaco2 with force sensors, using Kinova's low level API. The
controllers are located in the ABR_Control repo.

1) All of the required information about an arm model is kept in that arm's
config file. More information on the formatting of the config can be found in
the ABR_Control repo's README.

2) The controllers make use of the robot configuration files to generate
control signals that drive the robot to a target. The ABR_Control library
provides implementations of operational space control, joint space control,
and a floating controller, all of which can be used on the Jaco2

Additionally, there are signals and path planners that can be used in
conjunction with the controllers. See the `obstacle_avoidance` or
`linear_path_planning` files in ABR_Control for examples on how to use these.

3) For communications to and from the system under control, an interface class
is used. This communication is done through the low level RS485 API provided by
Kinova. When using the force control option of the arm, it is important to send
a control signal at a frequency faster that 200ms, otherwise the arm will revert
to position mode. 

A control loop using these three files looks like::

    import abr_jaco2
    from abr_control.controllers import OSC
      
    robot_config = abr_jaco2.Config()
    interface = abr_jaco2.Interface(robot_config)
    ctrlr = OSC(robot_config)
    # run controller once out of main loop to avoid auto exit after 200ms delay
    zeros = np.zeros(robot_config.N_LINKS)
    ctrlr.generate(q=zeros, dq=zeros, target=zeros(3))
    # run once outside main loop as well, this will return the cartesian
    # coordinates of the end effector
    robot_config.Tx('EE', q=zeros

    interface.connect()
    # need to switch to a position where the torque on the joints is known to
    # successfully switch to force mode. Use the default in the Config to do so
    interface.init_position_mode()
    interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

    target_xyz = [.57, .03 .87]  # in metres
    interface.init_force_mode()
    
    while 1:
        feedback = interface.get_feedback()  # returns a dictionary with q, dq
        xyz = robot_config.Tx('EE', q=q, target_pos = target_xyz) # ee position
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
the different controllers. If you would like to run simulations of the Jaco2,
please see the ABR_Control repo.

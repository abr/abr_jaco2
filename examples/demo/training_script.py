# -*- coding: utf-8 -*-
"""

This is a temporary script file.
"""
#import send_camera_targets
import deliverable_5_auto
num_trials = 2
for ii in range(0,num_trials):
    print("Running Trial ", ii)
    deliverable_5_auto.main(trial=ii, auto_start=True,
        data_folder='data/deliverable5/wrench/auto')

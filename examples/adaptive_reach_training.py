"""
"""
import time
import traceback
import numpy as np
from abr_control.utils import Target
from abr_control.utils.training import Training
import abr_jaco2

for session in range(0,1):
    for run in range(0,3):
        training = Training(n_neurons=1000, n_ensembles=1, test_group='testing',
                     test_name="training_metod0", session=session, run=None,
                     weights=None ,pes_learning_rate=5e-6, backend='nengo',
                     offset=None, kp=20, kv=6, ki=0, seed=1, SCALES=None,
                     MEANS=None, probe_weights=True, avoid_limits=True,
                     adapt_input=[False, True, True, False, False, False],
                     adapt_output=[False, True, True, False, False, False],
                     neuron_type='lif')

        try:
            for targets in range(0, 1):
                # target_xyz = Target.random_target(r=[0.6, 0.8], theta=[0.57, 6.2],
                #         phi=[2.1, 4.21])
                target_xyz = [.57, .03, .87]
                reaching_time  = 4
                print("TARGET: ", target_xyz)
                if targets == 0:
                    training.connect_to_arm()
                training.reach_to_target(target_xyz=np.array(target_xyz), reaching_time=reaching_time)
        except:
            print(traceback.format_exc())

        finally:
            training.stop()
            training.save_data()
            training.save_parameters()

            # give user time to pause before the next run starts, only
            # works if looping through tests using a bash script
            print('2 Seconds to pause: Hit ctrl z to pause, fg to resume')
            time.sleep(1)
            print('1 Second to pause')
            time.sleep(1)
            print('Starting next test...')

# TO DO: MOVE TO CONTROL/UTILS

# Generates zeros weight file for first run of learning

import numpy as np

def __init__(filename='default_name', dim_in=6, n_neurons=1000):
    weights=np.zeros((dim_in,n_neurons))
    np.savez_compressed('data/learning_osc/%s/%i_neurons/zeros' %(filename,n_neurons), weights=[weights])

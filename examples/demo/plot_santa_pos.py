import numpy as np
import random
#import matplotlib.pylot as plt
camera_xyz_santa = np.load('data/camera_xyz_santa.npz')['camera_xyz_santa']
print(camera_xyz_santa)
print(len(camera_xyz_santa))
for ii in range(0,10):
  print(camera_xyz_santa[random.randrange(0,len(camera_xyz_santa))])

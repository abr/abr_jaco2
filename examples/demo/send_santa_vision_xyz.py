import time
import numpy as np
import os

save_to = ('data/target_position.txt')
camera_xyz_santa = np.load('data/camera_xyz_santa.npz')['camera_xyz_santa']
for ii in range(0,len(camera_xyz_santa)):
    time.sleep(2)
    f = open(save_to, 'w')
    f.write('%f,%f,%f' % (camera_xyz_santa[ii,0], 
                          camera_xyz_santa[ii,1], 
                          camera_xyz_santa[ii,2]))
    f.close()

    if ii>0 and camera_xyz_santa[ii,0] != camera_xyz_santa[ii-1,0] :
        print(camera_xyz_santa[ii])
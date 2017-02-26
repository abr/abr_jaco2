import numpy as np
import os

# ---------- Function to receive cartesian coordinate from camera ----------
def get_target_position(filename):

    # save run info for plotting
    if os.path.exists(filename):
        append_write = 'a'  # append if file exists
    else:
        append_write = 'w'  # make a new file if it does not exist

    print('Checking for coordinate information')
    while 1:
        if os.path.getsize(filename) <= 0:
            print('Waiting to obtain tooltip coordinates...')
            time.sleep(.5)

        elif os.path.getsize(filename) > 0:
            with open(filename, 'r') as myfile:
                xyz_vision = (myfile.readline()).split(',')
            print('xyz_vision read from text file: ', xyz_vision)

            # erase contents of text files for next position
            with open(filename, 'w') as myfile:
                myfile.seek(0)
                myfile.truncate()
            return xyz_vision
            break

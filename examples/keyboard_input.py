""" Waits for a key to be pressed (followed by enter key) and
saves the pressed key. You can then use this input to trigger
function calls, stop a script, etc.
"""
from abr_control.utils import KBHit

kb = KBHit.KBHit()

# set the terminal back to its initial state
kb.set_normal_term()

exit_script = False

print('\'s\' : start script')
print('\'q\' : exit script')
print('Hit Enter after selection')

while not exit_script:
    #  if keyboard hit followed by enter key detected...
    if kb.kbhit():
        # saves the value of the key pressed
        c = kb.getch()
        # if key pressed matches desired key, perform task
        # can be used to start / stop scripts, run functions,
        # provide feedback etc.
        if ord(c) == ord('q'):
            print('Goodbye')
            break
        if ord(c) == ord('s'):
            print('ABR Rocks!')
        else:
            print('Not a valid key')

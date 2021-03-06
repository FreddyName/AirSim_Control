"""
A simple example of hooking the keyboard on Linux using pyxhook
Any key pressed prints out the keys values, program terminates when spacebar
is pressed.
"""
from __future__ import print_function

# Libraries we need
import pyxhook
import time


# This function is called every time a key is presssed
def kbevent(event):
    global running
    if(event.ScanCode == 24):
        print("press q.")
    if(event.ScanCode == 88):
        print("press 2.")
    if(event.ScanCode == 83):
        print("press 4.")

    # If the ascii value matches spacebar, terminate the while loop
    if event.Ascii == 32:
        running = False
    print(event)

# Create hookmanager
hookman = pyxhook.HookManager()
# Define our callback to fire when a key is pressed down
hookman.KeyDown = kbevent
# Hook the keyboard
hookman.HookKeyboard()
# Start our listener
hookman.start()

# Create a loop to keep the application running
running = True
while running:
    time.sleep(0.1)

# Close the listener when we are done
hookman.cancel()

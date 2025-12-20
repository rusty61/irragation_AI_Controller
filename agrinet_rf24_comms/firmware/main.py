import agri_brain
from arduino.app_utils import App
import time

# Initialize the brain (this registers Bridge callbacks internally)
print(">>> [SYSTEM] Initializing AgriNet AI Engine...", flush=True)
brain = agri_brain.AgriBrain()
brain.start() # This registers Bridge.provide callbacks

# The App.run() will keep the process alive and handle callbacks
print(">>> [SYSTEM] Application Framework Ready.", flush=True)
App.run()

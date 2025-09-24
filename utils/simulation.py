import time
import pybullet as p
from configs.config import config




def wait(steps):
    """Wait for a specified number of simulation steps."""
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(config.simulation.step_delay)

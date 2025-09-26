import time
import pybullet as p
from configs.config import config




def wait(steps,simulation_steps=None):
    """Wait for a specified number of simulation steps."""
    sim_steps = simulation_steps if simulation_steps else config.simulation.step_delay
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(sim_steps)

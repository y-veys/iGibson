from gibson2.envs.locomotor_env import NavigateEnv, NavigateRandomEnv, NavigateRandomEnvSim2Real
from time import time
import numpy as np
from time import time
import gibson2
import os
from gibson2.core.render.profiler import Profiler
import logging

#logging.getLogger().setLevel(logging.DEBUG) #To increase the level of logging

def main():
    config_filename = os.path.join(os.path.dirname(gibson2.__file__),
                                   '../examples/configs/jr2_reaching.yaml')
    nav_env = NavigateEnv(config_file=config_filename, mode='pbgui')

    for j in range(10):
        nav_env.reset()
        for i in range(100):
            with Profiler('Environment action step'):
                action = nav_env.action_space.sample()
                print(action)
                state, reward, done, info = nav_env.step(action)
                if done:
                    logging.info("Episode finished after {} timesteps".format(i + 1))
                    break

if __name__ == "__main__":
    main()

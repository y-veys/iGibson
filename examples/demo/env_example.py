from gibson2.envs.locomotor_env import NavigateEnv, NavigateRandomEnv
from time import time
import numpy as np
from time import time
import gibson2
import os
from gibson2.core.render.profiler import Profiler

def main():
    config_filename = os.path.join(os.path.dirname(gibson2.__file__),
                                   '../examples/configs/jr2_reaching.yaml')
    nav_env = NavigateEnv(config_file=config_filename, mode='gui')
    for j in range(1000):
        nav_env.reset()
        for i in range(1000):
            with Profiler('Env action step'):
                action = nav_env.action_space.sample()
                action = [0,0,0,0,0.2,0,0]
                state, reward, done, info = nav_env.step(action)
                if done:
                    print("Episode finished after {} timesteps".format(i + 1))
                    break

if __name__ == "__main__":
    main()
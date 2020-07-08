from gibson2.core.physics.robot_locomotors import JR2_Kinova
from gibson2.core.physics.interactive_objects import VisualMarker
from gibson2.core.physics.scene import BuildingScene
from gibson2.utils.utils import parse_config
import os
import time
import numpy as np
import pybullet as p
import pybullet_data

def main():
    p.connect(p.GUI)
    p.setGravity(0,0,-9.8)
    p.setTimeStep(1./240.)

    floor = os.path.join(pybullet_data.getDataPath(), "mjcf/ground_plane.xml")
    p.loadMJCF(floor)

    #scene = BuildingScene('Placida',
    #                      is_interactive=True,
    #                      build_graph=True,
    #                      pybullet_load_texture=True)
    #scene.load()

    robots = []
    config = parse_config('../configs/jr2_reaching.yaml')
    jr = JR2_Kinova(config)
    robots.append(jr)

    positions = [[0, 0, 0]]

    for robot, position in zip(robots, positions):
        robot.load()
        robot.set_position(position)
        robot.robot_specific_reset()
        robot.keep_still()

    marker_position = [0,4,1]
    marker = VisualMarker(visual_shape=p.GEOM_SPHERE, radius=0.1)
    marker.load()
    marker.set_position(marker_position)

    print("Wait")

    for _ in range(120):  # keep still for 10 seconds
        p.stepSimulation()
        time.sleep(1./240.)

    print("Move")

    action_base = np.random.uniform(0, 1, robot.wheel_dim)

    for _ in range(2400):  # move with small random actions for 10 seconds
        for robot, position in zip(robots, positions):
            action_arm = np.random.uniform(-1, 1, robot.arm_dim)
            action = np.concatenate([action_base, action_arm])
            robot.apply_action([1,1,1,1,1,1,1])
        p.stepSimulation()
        time.sleep(1./240.0)

    p.disconnect()


if __name__ == '__main__':
    main()


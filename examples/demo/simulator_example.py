from gibson2.core.physics.robot_locomotors import JR2_Kinova_Head
from gibson2.core.simulator import Simulator
from gibson2.core.physics.scene import BuildingScene
from gibson2.core.physics.scene import StadiumScene
from gibson2.core.physics.interactive_objects import YCBObject
from gibson2.utils.utils import parse_config
import pybullet as p
import numpy as np
from gibson2.core.render.profiler import Profiler


def main():
    config = parse_config('../configs/jr2_reaching.yaml')
    s = Simulator(mode='gui', image_width=512, image_height=512)
    #scene = BuildingScene('Rs',
    #                      build_graph=True,
    #                      pybullet_load_texture=True)
    scene = StadiumScene()
    s.import_scene(scene)
    jr = JR2_Kinova_Head(config)
    #turtlebot = Turtlebot(config)
    s.import_robot(jr)

    for _ in range(10):
        obj = YCBObject('003_cracker_box')
        s.import_object(obj)
        obj.set_position_orientation(np.random.uniform(low=0, high=2, size=3), [0,0,0,1])

    for i in range(10000):
        with Profiler('Simulator step'):
            jr.apply_action([0,0,0.1,0.1,0,0,0,0,0])
            s.step()
            rgb = s.renderer.render_robot_cameras(modes=('rgb'))

    s.disconnect()


if __name__ == '__main__':
    main()

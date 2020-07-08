from gibson2.core.simulator import Simulator
from gibson2.core.physics.scene import StadiumScene
from gibson2.core.physics.robot_locomotors import JR2_Kinova
from gibson2.core.physics.interactive_objects import VisualMarker
from gibson2.utils.utils import parse_config
import pybullet as p
import numpy as np
from gibson2.core.render.profiler import Profiler


def main():  

    config = parse_config('../configs/jr2_reaching.yaml')   
    s = Simulator(mode='gui', image_width=512, image_height=512)
    
    scene = StadiumScene()
    s.import_scene(scene)

    jr = JR2_Kinova(config)
    s.import_robot(jr)

    marker = VisualMarker(visual_shape=p.GEOM_SPHERE, rgba_color=[0, 0, 1, 0.3], radius=0.5)
    s.import_object(marker)
    marker.set_position([0,4,1])

    for i in range(10000):
        with Profiler('Simulator step'):

            jr.apply_action([0,0,0,0,0,0,0])
            s.step()
            rgb = s.renderer.render_robot_cameras(modes=('rgb'))

    s.disconnect()


if __name__ == '__main__':
    main()

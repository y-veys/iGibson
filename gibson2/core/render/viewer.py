import cv2
import numpy as np
from gibson2.core.render.mesh_renderer.mesh_renderer_cpu import MeshRenderer


class Viewer:
    def __init__(self,
                 initial_pos = [0,0,1.2], 
                 initial_view_direction = [1,0,0], 
                 initial_up = [0,0,1],
                 ):
        self.px = initial_pos[0]
        self.py = initial_pos[1]
        self.pz = initial_pos[2]
        self.theta = np.arctan2(initial_view_direction[1], initial_view_direction[0])
        self.phi = np.arctan2(initial_view_direction[2], np.sqrt(initial_view_direction[0] ** 2 +
                                                                initial_view_direction[1] ** 2))

        self._mouse_ix, self._mouse_iy = -1, -1
        self.left_down = False
        self.middle_down = False
        self.right_down = False
        self.view_direction = np.array(initial_view_direction)
        self.up = initial_up
        self.renderer = None

        cv2.namedWindow('ExternalView')
        cv2.moveWindow("ExternalView", 0,0)
        #cv2.namedWindow('RGB HEAD')
        #cv2.namedWindow('RGB WRIST')
        #cv2.namedWindow('SEG')
        #cv2.namedWindow('DEPTH')
        cv2.setMouseCallback('ExternalView', self.change_dir)

    def change_dir(self, event, x, y, flags, param):
        if flags == cv2.EVENT_FLAG_LBUTTON + cv2.EVENT_FLAG_CTRLKEY and not self.right_down: 
            # Only once, when pressing left mouse while cntrl key is pressed
            self._mouse_ix, self._mouse_iy = x, y
            self.right_down = True
        elif (event == cv2.EVENT_MBUTTONDOWN) or (flags == cv2.EVENT_FLAG_LBUTTON + cv2.EVENT_FLAG_SHIFTKEY and not self.middle_down): 
            #middle mouse button press or only once, when pressing left mouse while shift key is pressed (Mac compatibility)
            self._mouse_ix, self._mouse_iy = x, y
            self.middle_down = True
        elif event == cv2.EVENT_LBUTTONDOWN: #left mouse button press
            self._mouse_ix, self._mouse_iy = x, y
            self.left_down = True
        elif event == cv2.EVENT_LBUTTONUP: #left mouse button released
            self.left_down = False
            self.right_down = False
            self.middle_down = False
        elif event == cv2.EVENT_MBUTTONUP: #middle mouse button released
            self.middle_down = False

        if event == cv2.EVENT_MOUSEMOVE: #moving mouse location on the window
            if self.left_down: #if left button was pressed we change orientation of camera
                dx = (x - self._mouse_ix) / 100.0
                dy = (y - self._mouse_iy) / 100.0
                self._mouse_ix = x
                self._mouse_iy = y

                self.phi += dy
                self.phi = np.clip(self.phi, -np.pi/2 + 1e-5, np.pi/2 - 1e-5)
                self.theta += dx

                self.view_direction = np.array([np.cos(self.theta)* np.cos(self.phi), np.sin(self.theta) * np.cos(
                    self.phi), np.sin(self.phi)])
            elif self.middle_down: #if middle button was pressed we get closer/further away in the viewing direction
                d_vd = (y - self._mouse_iy) / 100.0
                self._mouse_iy = y

                motion_along_vd = d_vd*self.view_direction
                self.px += motion_along_vd[0]
                self.py += motion_along_vd[1]
                self.pz += motion_along_vd[2]
            elif self.right_down: #if right button was pressed we change translation of camera

                zz = self.view_direction/np.linalg.norm(self.view_direction)
                xx = np.cross(zz, np.array([0,0,1]))
                xx = xx/np.linalg.norm(xx)
                yy = np.cross(xx, zz)
                motion_along_vx = -((x - self._mouse_ix) / 100.0)*xx
                motion_along_vy = ((y - self._mouse_iy) / 100.0)*yy
                self._mouse_ix = x
                self._mouse_iy = y

                self.px += (motion_along_vx[0] + motion_along_vy[0])
                self.py += (motion_along_vx[1] + motion_along_vy[1])
                self.pz += (motion_along_vx[2] + motion_along_vy[2])

    def update(self):
        camera_pose = np.array([self.px, self.py, self.pz])
        if not self.renderer is None:
            self.renderer.set_camera(camera_pose, camera_pose + self.view_direction, self.up)

        if not self.renderer is None:
            frame = cv2.cvtColor(np.concatenate(self.renderer.render(modes=('rgb')), axis=1),
                                 cv2.COLOR_RGB2BGR)
        else:
            frame = np.zeros((300, 300, 3)).astype(np.uint8)

        # Text with the position and viewing direction of the camera of the external viewer
        cv2.putText(frame, "px {:1.1f} py {:1.1f} pz {:1.1f}".format(self.px, self.py, self.pz), (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, "[{:1.1f} {:1.1f} {:1.1f}]".format(*self.view_direction), (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow('ExternalView', frame)

        # We keep some double functinality for "backcompatibility"
        q = cv2.waitKey(1)
        if q == ord('w'):
            self.px += 0.05
        elif q == ord('s'):
            self.px -= 0.05
        elif q == ord('a'):
            self.py += 0.05
        elif q == ord('d'):
            self.py -= 0.05
        elif q == ord('e'):
            self.pz += 0.05
        elif q == ord('q'):
            exit()

        if not self.renderer is None:
            frames = self.renderer.render_robot_cameras(modes=('rgb'))
            #frames = -self.renderer.render_robot_cameras(modes=('3d'))[0][:, :, 2:3]
            if len(frames) > 0:
                frame = cv2.cvtColor(np.concatenate(frames, axis=1), cv2.COLOR_RGB2BGR)
                cv2.imshow('RGB', frame)

            #rgb = self.renderer.render_robot_cameras(modes=('rgb'))[0][:, :, :3]

            '''
            seg = self.renderer.render_robot_cameras(modes=('seg'))[0][:, :, 0:1]
            seg = np.clip(seg * 255.0 / 5, 0.0, 1.0)

            #all_but_goal = seg < 1 
            #seg[all_but_goal] = 0

            (x_avg, y_avg) = (np.mean(np.where(seg==1)[0]), np.mean(np.where(seg==1)[1]))

            if (not np.isnan(x_avg) and not np.isnan(y_avg)):
                seg[int(round(x_avg)), int(round(y_avg))] = 0

            depth = self.renderer.render_robot_cameras(modes=('3d'))
            #print(len(depth[0]))
            depth = -self.renderer.render_robot_cameras(modes=('3d'))[0][:, :, 2:3]

            depth[depth < 0.0] = 0.0
            depth[depth > 5.0] = 5.0

            # re-scale depth to [0.0, 1.0]
            depth /= 5.0
            '''
            #cv2.imshow('RGB', cv2.cvtColor(np.concatenate(rgb, axis=1), cv2.COLOR_RGB2BGR))
            #cv2.imshow('SEG', cv2.flip(cv2.rotate(cv2.cvtColor(np.concatenate(seg, axis=1), cv2.COLOR_RGB2BGR), cv2.ROTATE_90_CLOCKWISE),1))
            #cv2.imshow('DEPTH', cv2.flip(cv2.rotate(cv2.cvtColor(np.concatenate(depth, axis=1), cv2.COLOR_RGB2BGR),cv2.ROTATE_90_CLOCKWISE),1))


if __name__ == '__main__':
    viewer = Viewer()
    while True:
        viewer.update()

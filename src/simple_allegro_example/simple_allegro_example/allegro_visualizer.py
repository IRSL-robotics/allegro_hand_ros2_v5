import numpy as np
import viser
import yourdfpy
from pathlib import Path
from functools import partial
from .config import FINGERS, COLOR

class AllegroVisualizer:
    def __init__(self, urdf_path: Path):
        self.server = viser.ViserServer()

        self.urdf = yourdfpy.URDF.load(
            urdf_path,
            filename_handler=partial(
                yourdfpy.filename_handler_magic,
                dir=urdf_path.parent,
            ),
        )

        self.joint_names = self.urdf.actuated_joint_names

        self.sliders = {}
        self.on_q_update_callback = None

        self._suppress_slider_cb = False   # programmatic slider set 중 publish 방지
        self.synced = False                # 로봇 상태로 1회 이상 slider sync 되었는지
        self._init_gui()


    def _init_gui(self):
        with self.server.gui.add_folder("Joint position control"):
            for jname in self.joint_names:
                j = self.urdf.joint_map[jname]
                mid = (j.limit.lower + j.limit.upper) / 2

                slider = self.server.gui.add_slider(
                    label=jname,
                    min=j.limit.lower,
                    max=j.limit.upper,
                    step=1e-3,
                    initial_value=mid,
                )
                self.sliders[jname] = slider

                slider.on_update(lambda _: self._on_slider_update())

        self.reset_button = self.server.gui.add_button("Reset")
        @self.reset_button.on_click
        def _(_):
            q = self.get_q()
            for jname, val in zip(self.joint_names, q):
                self.sliders[jname].value = val
            self.draw(q)

    def sync_from_robot(self, q_robot: np.ndarray):
        self._suppress_slider_cb = True
        try:
            for jname, val in zip(self.joint_names, q_robot):
                self.sliders[jname].value = float(val)  # 여기서 on_update가 튈 수 있으니 suppress 필요
        finally:
            self._suppress_slider_cb = False

        self.synced = True
        self.draw(q_robot)

    def set_update_callback(self, fn):
        self.on_q_update_callback = fn

    def get_q(self):
        return np.array([self.sliders[jn].value for jn in self.joint_names])

    def _on_slider_update(self):
        if (not self.synced) or self._suppress_slider_cb:
            return

        q = self.get_q()
        if self.on_q_update_callback:
            self.on_q_update_callback(q)
        self.draw(q)

    def draw(self, q):
        self.urdf.update_cfg(q)

        # joint child link positions
        pts = {}
        for jname, j in self.urdf.joint_map.items():
            T = self.urdf.get_transform(j.child)
            pts[j.child] = T[:3, 3]

        pcd = np.array(list(pts.values()))
        self.server.scene.add_point_cloud("robot_points", pcd, (1, 0, 0), point_size=0.01)

        for fname, links in FINGERS.items():
            link_pts = np.array([pts[l] for l in links])
            seg = np.stack([link_pts[:-1], link_pts[1:]], axis=1)
            self.server.scene.add_line_segments(
                f"finger_{fname}", seg, colors=COLOR[fname], line_width=10.0
            )

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from pathlib import Path
import viser
import yourdfpy
from functools import partial

ALLEGRO_LEFT_URDF_PATH = Path(
    "/root/allegro_ws/src/allegro_hand_ros2_v5/src/allegro_hand_controllers/urdf/allegro_hand_description_left_B.urdf"
)
FINGERS = {
    "index":  ["link_0_0", "link_1_0", "link_2_0", "link_3_0", "link_3_0_tip"],
    "middle": ["link_4_0", "link_5_0", "link_6_0", "link_7_0", "link_7_0_tip"],
    "pinky":  ["link_8_0", "link_9_0", "link_10_0", "link_11_0", "link_11_0_tip"],
    "thumb":  ["link_12_0", "link_13_0", "link_14_0", "link_15_0", "link_15_0_tip"],
}
COLOR = {
    "index":  (1.0, 0.2, 0.2),
    "middle": (0.2, 1.0, 0.2),
    "pinky":  (0.2, 0.2, 1.0),
    "thumb":  (1.0, 0.7, 0.2),
}


class AllegroSimpleController(Node):
    def __init__(self):
        super().__init__('allegro_simple_controller')
        self.init_timer = self.create_timer(0.1, self.try_initial_sync)
        self.initialized = False
        
        self.NUM = 0
        self.cmd_topic = f'/allegroHand_{self.NUM}/joint_cmd'
        self.state_topic = f'/allegroHand_{self.NUM}/joint_states'
        
        self.publisher_ = self.create_publisher(JointState, self.cmd_topic, 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.state_topic,
            self.joint_state_callback,
            10
        )
        self.server = viser.ViserServer()
        self.urdf = yourdfpy.URDF.load(
                ALLEGRO_LEFT_URDF_PATH,
                filename_handler=partial(
                    yourdfpy.filename_handler_magic,
                    dir=ALLEGRO_LEFT_URDF_PATH.parent,
                ),
            )
        
        self.q_robot = None
        self.ready = False

        self.fingers = {
            "thumb": ["joint_12_0", "joint_13_0", "joint_14_0"]
        }
        self.sliders = {}
        self.joint_names = self.urdf.actuated_joint_names
        with self.server.gui.add_folder("Joint position control"):
            for jname in self.joint_names:
                joint: yourdfpy.Joint = self.urdf.joint_map[jname]
                central = (joint.limit.lower + joint.limit.upper)/2
                slider = self.server.gui.add_slider(
                    label=jname,
                    min=joint.limit.lower,
                    max=joint.limit.upper,
                    step=1e-3,
                    initial_value=central,
                )
                self.sliders[jname] = slider
                slider.on_update(
                    lambda _: self.on_slider_update_and_publish()
                )
                
        self.reset_button = self.server.gui.add_button("Reset")
        @self.reset_button.on_click
        def _(_):
            if self.q_robot is None:
                return

            for jname, val in zip(self.joint_names, self.q_robot):
                self.sliders[jname].value = float(val)
            self.draw_link_points(self.q_robot)
    
    @property
    def q_from_gui(self):
        q_list = [self.sliders[jname].value for jname in self.joint_names]
        return np.array(q_list)
    
    def try_initial_sync(self):
        if self.q_robot is None:
            return
        if self.initialized:
            return

        self.get_logger().info("Initializing sliders with real robot states...")

        for jname, val in zip(self.joint_names, self.q_robot):
            self.sliders[jname].value = float(val)

        self.draw_link_points(self.q_robot)
        self.initialized = True
        self.ready = True
        self.init_timer.cancel()
        print("ready")


    def joint_state_callback(self, msg: JointState):
        try:
            # order msg.position according to URDF joint_names
            name_to_pos = dict(zip(msg.name, msg.position))
            q = np.array([name_to_pos[j] for j in self.joint_names])
            self.q_robot = q

        except Exception as e:
            self.get_logger().error(f"joint_state_callback error: {e}")

    def draw_link_points(self, q):
        self.urdf.update_cfg(q)
        points = {}
        for jname, joint in self.urdf.joint_map.items():
            lname = joint.child
            T_parent_child = self.urdf.get_transform(joint.child)
            points[lname] = T_parent_child[:3, -1]
        pcd = np.array(list(points.values()))
        self.server.scene.add_point_cloud(
            "points", pcd, (1., 0, 0), point_size=0.01)
    
        for fname, links in FINGERS.items():
            pts = np.array([points[l] for l in links])
            segments = np.stack([pts[:-1], pts[1:]], axis=1)
            self.server.scene.add_line_segments(
                f"finger_{fname}",
                points=segments,
                colors=COLOR[fname],
                line_width=10.0
            )
    
    def on_slider_update_and_publish(self):
        if not self.ready:
            return
        
        q = self.q_from_gui
        self.publish_cmd(q)
        self.draw_link_points(q)
    
    def publish_cmd(self, q):
        msg = JointState()
        msg.name = self.joint_names
        msg.position = list(q)
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AllegroSimpleController()
    node.draw_link_points(node.q_from_gui)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
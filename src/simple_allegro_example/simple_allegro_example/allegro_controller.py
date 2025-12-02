import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class AllegroController(Node):
    def __init__(self, cmd_topic, state_topic, joint_names):
        super().__init__('allegro_controller')

        self.joint_names = joint_names
        self.q_robot = None
        self.ready = False

        self.pub = self.create_publisher(JointState, cmd_topic, 10)
        self.sub = self.create_subscription(
            JointState, state_topic, self._state_callback, 10
        )

        self.on_visual_initial_sync = None
        self.timer = self.create_timer(0.1, self._try_initial_sync)

    def set_initial_sync_callback(self, fn):
        """fn(q_robot) → visualizer.draw(q_robot)"""
        self.on_visual_initial_sync = fn

    def _state_callback(self, msg):
        try:
            name_to_pos = dict(zip(msg.name, msg.position))
            q = np.array([name_to_pos[j] for j in self.joint_names])
            self.q_robot = q
        except Exception as e:
            self.get_logger().error(f"state error: {e}")

    def _try_initial_sync(self):
        if self.q_robot is None or self.ready:
            return

        self.get_logger().info("Initial ROS→Visualizer sync done.")
        if self.on_visual_initial_sync:
            self.on_visual_initial_sync(self.q_robot)

        self.ready = True
        self.timer.cancel()

    def publish_cmd(self, q):
        if not self.ready:
            return
        msg = JointState()
        msg.name = self.joint_names
        msg.position = list(q)
        self.pub.publish(msg)

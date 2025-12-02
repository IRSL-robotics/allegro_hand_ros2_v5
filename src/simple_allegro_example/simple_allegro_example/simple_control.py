from .allegro_controller import AllegroController
from .allegro_visualizer import AllegroVisualizer
from .config import ALLEGRO_LEFT_URDF_PATH
import rclpy

class AllegroExampleRunner:
    def __init__(self, urdf_path, cmd_topic, state_topic):
        self.visualizer = AllegroVisualizer(urdf_path=urdf_path)
        self.controller = AllegroController(
            cmd_topic=cmd_topic,
            state_topic=state_topic,
            joint_names=self.visualizer.joint_names,
        )
        self.visualizer.set_update_callback(
            lambda q: self.controller.publish_cmd(q)
        )
        self.controller.set_initial_sync_callback(
            lambda q: self.visualizer.sync_from_robot(q))

        self.visualizer.draw(self.visualizer.get_q())

    def run(self):
        rclpy.spin(self.controller)
        self.controller.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()

    runner = AllegroExampleRunner(
        urdf_path=ALLEGRO_LEFT_URDF_PATH,
        cmd_topic="/allegroHand_0/joint_cmd",
        state_topic="/allegroHand_0/joint_states",
    )
    runner.run()


if __name__ == "__main__":
    main()

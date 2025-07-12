import rclpy
from rclpy.node import Node
from moveit2 import MoveIt2
import time

class MoveToJointPoses(Node):
    def __init__(self):
        super().__init__('move_to_joint_poses')

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                'joint_1', 'joint_2', 'joint_3',
                'joint_4', 'joint_5', 'joint_6'
            ],
        )

        # Define joint positions (radians)
        self.poses = {
            "home":  [0.0, -1.57, 1.57, 0.0, 1.57, 0.0],
            "pick":  [0.2, -1.2, 1.4, 0.0, 1.3, 0.0]
        }

        # Run the sequence
        self.execute_poses()

    def execute_poses(self):
        for name, joint_values in self.poses.items():
            self.get_logger().info(f"Moving to {name}")
            self.moveit2.move_to_joint_positions(joint_values)
            self.moveit2.wait_until_executed()
            time.sleep(2)  # Delay between poses

def main():
    rclpy.init()
    node = MoveToJointPoses()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

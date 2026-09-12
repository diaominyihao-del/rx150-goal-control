import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
from rx150_goal_control.kinematics import forward_kinematics
from rx150_goal_control.trajectory import cubic_trajectory
from rx150_goal_control.kinematics import inverse_kinematics_newton

class GoalControl(Node):
    def __init__(self):
        super().__init__("rx150_goal_controller")
        self.publisher = self.create_publisher(
            JointState,
            "/rx150/joint_states",
            10
        )
        self.start = [0.0,0.0,0.0,0.0,0.0]
        self.duration = 3.0
        self.steps = 150
        self.trajectory = []

        self.index = 0

        self.get_logger().info("RX-150 move_to_target 已启动")

    def publish_angles(self,waist_rad,shoulder_rad,elbow_rad,wrist_angle_rad,wrist_rotate_rad):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "waist",
            "shoulder",
            "elbow",
            "wrist_angle",
            "wrist_rotate",
        ]
        msg.position = [
            waist_rad,
            shoulder_rad,
            elbow_rad,
            wrist_angle_rad,
            wrist_rotate_rad,
        ]

        self.publisher.publish(msg)
        if self.index %10 == 0:
            self.get_logger().info(
                f"waist={math.degrees(waist_rad):6.1f} deg, "
                f"shoulder={math.degrees(shoulder_rad):6.1f} deg, "
                f"elbow={math.degrees(elbow_rad):6.1f} deg"
                f"wrist_angle={math.degrees(wrist_angle_rad):6.1f} deg"
                f"wrist_rotate = {math.degrees(wrist_rotate_rad):6.1f} deg"
            )

    def move_to_target(self,target):
        q_goal = inverse_kinematics_newton(self.start,target)
        if q_goal is None:
            print("目标不可达，取消运动")
            return
        
        q_goal[0] = (q_goal[0] + np.pi) % (2 * np.pi) - np.pi 
        q_goal[4] = (q_goal[4] + np.pi) % (2 * np.pi) - np.pi

        self.trajectory = cubic_trajectory(self.start, q_goal, self.duration, self.steps)
        x, y, z = forward_kinematics(*q_goal)
        error = np.linalg.norm([x - target[0], y - target[1], z - target[2]])

        if error > 0.02:
            print("目标不可达，取消运动")
            return

        self.index = 0
        for step in self.trajectory:
            self.publish_angles(step[0],step[1],step[2],step[3],step[4])
            self.index+=1
            time.sleep(self.duration / self.steps)
        self.index = 0
        self.start = q_goal
        
def main(args=None):
    rclpy.init(args=args)
    node = GoalControl()

    while True:
        text = input("请输入 x y z（用空格隔开，输入 q 退出）：")

        if text.strip() == "q":
            break

        x, y, z = [float(v) for v in text.split()]
        target = [x, y, z]

        node.move_to_target(target)
        rclpy.spin_once(node, timeout_sec=0.05)


    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
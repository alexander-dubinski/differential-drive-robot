import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster, TransformStamped

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from example_interfaces.srv import Trigger


class DiffDriveSimulator(Node):

    def __init__(self):
        super().__init__('diffdrive_sim')

        self.wheel_radius: float = 0.4
        self.base_length: float = 0.825

        self.X = np.array([0., 0., 0.])
        self.Q = np.array([0., 0.])

        self.cmd_vel_ = self.create_subscription(Twist, '/cmd_vel', self.velocity_cmd_callback, 10)

        qos_profile = QoSProfile(depth=10)

        self.tf_broadcaster_ = TransformBroadcaster(self, qos_profile)
        self.joint_state_publisher_ = self.create_publisher(JointState, '/joint_states', qos_profile)

        self.pose_reset_srv_ = self.create_service(Trigger, 'robot_pose_reset', self.reset_pose_callback)

    def velocity_cmd_callback(self, msg: Twist):
        pass

    def forward(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        u_l, u_r = u
        u_sum = np.sum(u)
        u_diff = u_r - u_l
        r_2 = self.wheel_radius / 2

        x_v = r_2 * u_sum * np.cos(x[2])
        y_v = r_2 * u_sum * np.sin(x[2])
        theta_v = (self.wheel_radius / self.base_length) * u_diff

        return np.array([x_v, y_v, theta_v])


    def inverse(self, x: np.ndarray, v: np.ndarray) -> np.ndarray:
        inv_wheel_radius =  1 / self.wheel_radius
        x_v, y_v, theta_v = v

        u_l = inv_wheel_radius * ()

    def pose_reset_callback(self, _, res: Trigger):
        self.X = np.array([0., 0., 0.])
        self.Q = np.array([0., 0.])
        res.success = True
        res.message = ''
        return res


def main(args=None):
    rclpy.init(args=args)

    diffdrive_sim = DiffDriveSimulator()

    rclpy.spin(diffdrive_sim)

    diffdrive_sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
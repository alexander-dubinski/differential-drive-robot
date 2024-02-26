import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster, TransformStamped

from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import JointState
from example_interfaces.srv import Trigger


class DiffDriveSimulator(Node):

    def __init__(self):
        super().__init__('diffdrive_sim')

        self.wheel_radius: float = 0.4
        self.wheel_base: float = 0.825

        self.X = np.array([0., 0., 0.])
        self.Q = np.array([0., 0.])

        self.vel_cmd_ = np.array([0., 0., 0.])

        self.cmd_vel_ = self.create_subscription(Twist, '/cmd_vel', self.velocity_cmd_callback, 10)

        qos_profile = QoSProfile(depth=10)

        self.tf_broadcaster_ = TransformBroadcaster(self, qos_profile)
        self.joint_state_publisher_ = self.create_publisher(JointState, '/joint_states', qos_profile)

        self.pose_reset_srv_ = self.create_service(Trigger, 'robot_pose_reset', self.reset_pose_callback)

        self.dt = 1 / 30.0
        self.clock = self.create_timer(self.dt, self.clock_callback)

    def velocity_cmd_callback(self, msg: Twist):
        self.vel_cmd_[0] = msg.linear.x
        self.vel_cmd_[1] = msg.linear.y
        self.vel_cmd_[2] = msg.angular.z

    def clock_callback(self):

        u_l, u_r = self.inverse(self.X, self.vel_cmd_)
        self.Q[0] += u_l * self.dt
        self.Q[1] += u_r * self.dt

        self.X[0] += self.vel_cmd_[0] * self.dt
        self.X[1] += self.vel_cmd_[1] * self.dt
        self.X[2] += self.vel_cmd_[2] * self.dt

        now = self.get_clock().now()

        odom_trans = TransformStamped()
        joint_state = JointState()

        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'chassis'
        odom_trans.transform.translation.x = self.X[0]
        odom_trans.transform.translation.y = self.X[1]
        odom_trans.transform.translation.z = 0
        odom_trans.transform.rotation = DiffDriveSimulator.euler_to_quaternion(0., 0., self.X[2])


        odom_trans.header.stamp = joint_state.header.stamp = now.to_msg()

        joint_state.name = ['chassis_to_left_wheel', 'chassis_to_right_wheel']
        joint_state.position = [self.Q[0], self.Q[1]]

        self.joint_state_publisher_.publish(joint_state)
        self.tf_broadcaster_.sendTransform(odom_trans)

    def forward(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        u_l, u_r = u
        u_sum = np.sum(u)
        u_diff = u_r - u_l
        r_2 = self.wheel_radius / 2

        x_v = r_2 * u_sum * np.cos(x[2])
        y_v = r_2 * u_sum * np.sin(x[2])
        theta_v = (self.wheel_radius / self.wheel_base) * u_diff

        return np.array([x_v, y_v, theta_v])

    def inverse(self, x: np.ndarray, v: np.ndarray) -> np.ndarray:
        inv_wheel_radius = 1 / self.wheel_radius
        x_v, y_v, theta_v = v
        sin_theta = np.sin(x[2])
        cos_theta = np.cos(x[2])

        pos_part = inv_wheel_radius * (x_v * cos_theta + y_v * sin_theta)
        theta_part = inv_wheel_radius * 0.5 * self.wheel_base * theta_v

        u_l = pos_part - theta_part
        u_r = pos_part + theta_part

        return np.array([u_l, u_r])

    def pose_reset_callback(self, _, res: Trigger):
        self.X = np.array([0., 0., 0.])
        self.Q = np.array([0., 0.])
        res.success = True
        res.message = ''
        return res

    @staticmethod
    def euler_to_quaternion(r: float, p: float, y: float) -> Quaternion:
        qx = np.sin(r / 2) * np.cos(p / 2) * np.cos(y / 2) - np.cos(r / 2) * np.sin(p / 2) * np.sin(y / 2)
        qy = np.cos(r / 2) * np.sin(p / 2) * np.cos(y / 2) + np.sin(r / 2) * np.cos(p / 2) * np.sin(y / 2)
        qz = np.cos(r / 2) * np.cos(p / 2) * np.sin(y / 2) - np.sin(r / 2) * np.sin(p / 2) * np.cos(y / 2)
        qw = np.cos(r / 2) * np.cos(p / 2) * np.cos(y / 2) + np.sin(r / 2) * np.sin(p / 2) * np.sin(y / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)

    diffdrive_sim = DiffDriveSimulator()

    rclpy.spin(diffdrive_sim)

    diffdrive_sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
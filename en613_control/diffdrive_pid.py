import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import Buffer, TransformListener, LookupException
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Vector3


class DiffDrivePID(Node):

    def __init__(self):
        super().__init__('diffdrive_pid')
        self.pose_offset = 0.4125
        self.dt = 1 / 30.0
        self.goal_pose = np.array([0., 0.])
        self.pose = np.array([0., 0., 0.])
        qos_profile = QoSProfile(depth=10)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        self.goal_pose_ = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.cmd_vel_ = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        self.clock = self.create_timer(self.dt, self.clock_callback)

    def clock_callback(self):
        try:
            when = rclpy.time.Time()
            trans = self.tf_buffer_.lookup_transform('odom', 'chassis', when, timeout=Duration(seconds=5.0))
        except LookupException:
            self.get_logger().info('Transform not ready, waiting...')
            return

        x, y = trans.transform.translation.x, trans.transform.translation.y
        theta = self.quaternion_to_yaw(trans.transform.rotation)

        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        x_p = x + self.pose_offset * cos_theta
        y_p = y + self.pose_offset * sin_theta
        x_vel = (x - self.pose[0]) / self.dt
        y_vel = (y - self.pose[1]) / self.dt
        theta_vel = (theta - self.pose[2]) / self.dt
        x_p_vel = x_vel - self.pose_offset * theta_vel * sin_theta
        y_p_vel = y_vel + self.pose_offset * theta_vel * cos_theta

        # TODO Fix these gains
        k_p = 0.75
        k_d = 1
        x_g, y_g = self.goal_pose

        rot_mat = np.array([[cos_theta, sin_theta],
                            [-sin_theta, cos_theta]])

        pd = -k_p*np.array([x_p - x_g, y_p - y_g]) - k_d * np.array([x_p_vel, y_p_vel])

        self.pose[0], self.pose[1], self.pose[2] = x, y, theta

        v_f, wl = rot_mat @ pd

        twist = Twist(linear=Vector3(x=v_f, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=wl))

        self.cmd_vel_.publish(twist)

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        self.goal_pose[0] = msg.pose.position.x
        self.goal_pose[1] = msg.pose.position.y

    @staticmethod
    def quaternion_to_yaw(q: Quaternion) -> float:
        return np.arctan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))


def main(args=None):
    rclpy.init(args=args)

    diffdrive_pid = DiffDrivePID()

    rclpy.spin(diffdrive_pid)

    diffdrive_pid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
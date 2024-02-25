import rclpy
from rclpy.node import Node


class DiffDrivePID(Node):

    def __init__(self):
        super().__init__('diffdrive_pid')


def main(args=None):
    rclpy.init(args=args)

    diffdrive_pid = DiffDrivePID()

    rclpy.spin(diffdrive_pid)

    diffdrive_pid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
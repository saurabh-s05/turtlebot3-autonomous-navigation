import math

import rclpy
from rclpy.node import Node

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

Z = 0.175

# name, start (x, y), end (x, y), speed [m/s], phase
OBSTACLES = [
    ('moving_obs_1', (-0.55, -1.3), (-0.55, 1.3), 0.10, 0.0),
    ('moving_obs_2', (-1.3, 0.55), (1.3, 0.55), 0.08, 0.3),
    ('moving_obs_3', (-0.9, -0.55), (0.9, -0.55), 0.12, 0.6),
    ('moving_obs_4', (1.65, -0.9), (1.65, 0.9), 0.10, 0.1),
]


class DynamicObstacles(Node):

    def __init__(self):
        super().__init__('dynamic_obstacles')
        self.declare_parameter('world', 'default')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('speed_scale', 1.0)

        world = self.get_parameter('world').value
        self.speed_scale = self.get_parameter('speed_scale').value
        self.service = f'/world/{world}/set_pose_vector'
        self.gz = GzNode()
        self.start = self.get_clock().now()
        self.failures = 0

        self.create_timer(1.0 / self.get_parameter('rate').value, self.update)
        self.get_logger().info(f'Moving {len(OBSTACLES)} obstacles')

    def update(self):
        t = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        req = Pose_V()
        for name, (x0, y0), (x1, y1), speed, phase in OBSTACLES:
            length = math.hypot(x1 - x0, y1 - y0)
            q = (speed * self.speed_scale * t / (2.0 * length) + phase) % 1.0
            s = 1.0 - abs(2.0 * q - 1.0)
            pose = req.pose.add()
            pose.name = name
            pose.position.x = x0 + s * (x1 - x0)
            pose.position.y = y0 + s * (y1 - y0)
            pose.position.z = Z
            pose.orientation.w = 1.0

        ok, rep = self.gz.request(self.service, req, Pose_V, Boolean, 200)
        if not (ok and rep.data):
            self.failures += 1
            if self.failures % 40 == 1:
                self.get_logger().warn(f'{self.service} call failed')


def main():
    rclpy.init()
    node = DynamicObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

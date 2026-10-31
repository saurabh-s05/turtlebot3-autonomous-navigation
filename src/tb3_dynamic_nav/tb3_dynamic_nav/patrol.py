import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# robot spawns at world (-2.0, -0.5), which is the map origin
WORLD_TO_MAP = (2.0, 0.5)

# x, y, yaw in Gazebo world coordinates
WAYPOINTS = [
    (-1.6, -1.2, 0.0),
    (0.55, -1.65, 0.0),
    (1.7, -0.55, 1.57),
    (1.7, 1.0, 1.57),
    (0.0, 1.75, 3.14),
    (-1.6, 0.9, -1.57),
    (0.55, 0.55, 0.0),
    (-2.0, -0.5, 0.0),
]


class Patrol(Node):

    def __init__(self):
        super().__init__('patrol')
        self.declare_parameter('loop', True)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('goal_timeout', 120.0)

        self.loop = self.get_parameter('loop').value
        self.max_retries = self.get_parameter('max_retries').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def wait_for(self, future, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def navigate(self, x, y, yaw):
        while rclpy.ok() and not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('Waiting for navigate_to_pose...')

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)
        send = self.client.send_goal_async(goal)
        if not self.wait_for(send, 5.0):
            return 'no response'
        handle = send.result()
        if not handle.accepted:
            return 'rejected'

        result = handle.get_result_async()
        start = time.monotonic()
        lost_since = None
        while rclpy.ok() and not result.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.monotonic() - start > self.goal_timeout:
                self.wait_for(handle.cancel_goal_async(), 5.0)
                return 'timed out'
            # if bt_navigator restarts, the result never arrives
            if self.client.server_is_ready():
                lost_since = None
            elif lost_since is None:
                lost_since = time.monotonic()
            elif time.monotonic() - lost_since > 5.0:
                return 'server lost'

        if not result.done():
            return 'interrupted'
        status = result.result().status
        return 'succeeded' if status == GoalStatus.STATUS_SUCCEEDED else f'status {status}'

    def run(self):
        goals = [(x + WORLD_TO_MAP[0], y + WORLD_TO_MAP[1], yaw) for x, y, yaw in WAYPOINTS]
        lap = 0
        while rclpy.ok():
            lap += 1
            for i, (x, y, yaw) in enumerate(goals, start=1):
                for attempt in range(self.max_retries + 1):
                    self.get_logger().info(
                        f'Lap {lap}, waypoint {i}/{len(goals)} ({x:.2f}, {y:.2f})'
                        + (f', retry {attempt}' if attempt else ''))
                    outcome = self.navigate(x, y, yaw)
                    if outcome == 'succeeded':
                        self.get_logger().info(f'Reached waypoint {i}')
                        break
                    self.get_logger().warn(f'Waypoint {i} failed: {outcome}')
                    time.sleep(2.0)
                else:
                    self.get_logger().error(f'Skipping waypoint {i}')
            if not self.loop:
                break


def main():
    rclpy.init()
    node = Patrol()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

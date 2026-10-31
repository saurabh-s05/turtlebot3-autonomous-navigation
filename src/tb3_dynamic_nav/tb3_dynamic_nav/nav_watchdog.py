import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from std_srvs.srv import Trigger

ACTIVE = State.PRIMARY_STATE_ACTIVE
INACTIVE = State.PRIMARY_STATE_INACTIVE
UNCONFIGURED = State.PRIMARY_STATE_UNCONFIGURED


class NavWatchdog(Node):
    """Restarts the Nav2 lifecycle nodes if a lifecycle manager stays down."""

    def __init__(self):
        super().__init__('nav_watchdog')
        self.declare_parameter('check_period', 5.0)
        self.declare_parameter('grace_period', 15.0)
        self.declare_parameter('localization_nodes', ['map_server', 'amcl'])
        self.declare_parameter('navigation_nodes', ['controller_server', 'planner_server',
                                                    'behavior_server', 'bt_navigator'])

        self.check_period = self.get_parameter('check_period').value
        self.grace_period = self.get_parameter('grace_period').value
        self.managers = {
            'lifecycle_manager_localization': self.get_parameter('localization_nodes').value,
            'lifecycle_manager_navigation': self.get_parameter('navigation_nodes').value,
        }
        self.seen_active = {m: False for m in self.managers}
        self.down_since = {m: None for m in self.managers}
        self.recoveries = 0
        self.service_clients = {}

    def call(self, srv_type, name, request, timeout=3.0):
        # reuse clients, a request on a freshly created client can get lost
        client = self.service_clients.get(name)
        if client is None:
            client = self.service_clients[name] = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            client.remove_pending_request(future)
            return None
        return future.result()

    def manager_active(self, manager):
        rep = self.call(Trigger, f'/{manager}/is_active', Trigger.Request())
        return None if rep is None else rep.success

    def node_state(self, node):
        rep = self.call(GetState, f'/{node}/get_state', GetState.Request())
        return rep.current_state.id if rep else None

    def transition(self, node, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        rep = self.call(ChangeState, f'/{node}/change_state', req, timeout=10.0)
        return bool(rep and rep.success)

    def healthy(self, manager):
        manager_active = self.manager_active(manager)
        if manager_active:
            return True
        # don't trust a single missed reply, ask the nodes directly
        states = [self.node_state(n) for n in self.managers[manager]]
        if all(s == ACTIVE for s in states):
            return True
        if any(s is not None and s != ACTIVE for s in states):
            return False
        return manager_active is None

    def recover(self, manager):
        self.recoveries += 1
        nodes = self.managers[manager]
        self.get_logger().warn(f'{manager} down for {self.grace_period:.0f}s, '
                               f'restarting its nodes (recovery {self.recoveries})')

        # STARTUP only works if every node is unconfigured
        for node in nodes:
            state = self.node_state(node)
            if state == ACTIVE:
                self.transition(node, Transition.TRANSITION_DEACTIVATE)
                state = self.node_state(node)
            if state == INACTIVE:
                self.transition(node, Transition.TRANSITION_CLEANUP)
                state = self.node_state(node)
            if state != UNCONFIGURED:
                self.get_logger().warn(f'{node} is in state {state}')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.STARTUP
        rep = self.call(ManageLifecycleNodes, f'/{manager}/manage_nodes', req, timeout=30.0)
        if rep and rep.success:
            self.get_logger().info(f'{manager}: nodes active again')
            self.down_since[manager] = None
        else:
            self.get_logger().error(f'{manager}: startup failed, will retry')
            self.down_since[manager] = time.monotonic()

    def check(self):
        for manager in self.managers:
            if self.healthy(manager):
                if not self.seen_active[manager]:
                    self.get_logger().info(f'{manager} is active')
                elif self.down_since[manager] is not None:
                    self.get_logger().info(f'{manager} recovered')
                self.seen_active[manager] = True
                self.down_since[manager] = None
                continue

            # still in the initial bringup
            if not self.seen_active[manager]:
                continue

            now = time.monotonic()
            if self.down_since[manager] is None:
                self.down_since[manager] = now
                self.get_logger().warn(f'{manager} is not active')
            elif now - self.down_since[manager] > self.grace_period:
                self.recover(manager)


def main():
    rclpy.init()
    node = NavWatchdog()
    try:
        while rclpy.ok():
            node.check()
            time.sleep(node.check_period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

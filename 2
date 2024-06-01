import argparse
import math

from enum import Enum

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path

# from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import SetBool

from warehouse_tasker_interfaces.srv import SendGoal, Register

class State(Enum):
    STANDBY = 1
    ACTIVE = 2
    RETURNING = 3

class AgentNode(Node):
    def __init__(self, name) -> None:
        super().__init__('agent_node')

        self.active_goal: PoseStamped | None = None
        self.state: State = State.STANDBY

        # Action Clients
        # TODO: Add back in the compute path for multi-robot things
        # self._compute_action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        # self.compute_feedback = ComputePathToPose.Feedback()
        
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_feedback: NavigateToPose.Feedback | None = None
        self.nav_is_complete = False

        # Services
        self.goal_service = self.create_service(SendGoal, 'send_goal', self.goal_service_callback)

        # Service Clients
        self.registration_client    = self.create_client(Register, 'register_agent')
        self.door_client            = self.create_client(SetBool, 'open_door')

        self.register_agent(name)
        self.loop_timer = self.create_timer(0.5, self.loop)

        self.callback_group = ReentrantCallbackGroup()

# -------------------------------------------------------------------------------------------

    def register_agent(self, name):
        while not self.registration_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        register_req = Register.Request()
        register_req.id = name

        self.get_logger().info(f'Registering agent {name}')

        future = self.registration_client.call_async(register_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# -------------------------------------------------------------------------------------------

    def activate_payload_mechanism(self):
        while not self.door_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        payload_req = SetBool.Request()
        payload_req.data = True

        self.get_logger().info(f'Calling service to payload mechanism...')

        future = self.door_client.call_async(payload_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# -------------------------------------------------------------------------------------------

    def goal_service_callback(self, request, response):
        self.active_goal = PoseStamped()
        self.active_goal.header.frame_id = 'map'
        self.active_goal.pose.orientation.w = 1.0

        self.active_goal.pose.position.x = request.x
        self.active_goal.pose.position.y = request.y

        self.get_logger().info(f'Received goal locations - x: {request.x}, y: {request.y}')
        self.state = State.ACTIVE

        response.success = True
        return response

# -------------------------------------------------------------------------------------------

    # def send_compute_goal(self, goal: PoseStamped):
    #     goal_msg = ComputePathToPose.Goal()
    #     goal_msg.goal = goal
    #
    #     self._compute_action_client.wait_for_server()
    #
    #     self._send_compute_goal_future = self._compute_action_client.send_goal_async(goal_msg)
    #     self._send_compute_goal_future.add_done_callback(self.compute_goal_response_callback)
    #
    # def compute_goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal rejected :(')
    #         return
    #
    #     self.get_logger().info('Goal accepted :)')
    #
    #     self._get_compute_result_future = goal_handle.get_result_async()
    #     self._get_compute_result_future.add_done_callback(self.get_compute_result_callback)
    #
    # def get_compute_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f'Result: {len(result.path.poses)}')
    #     self.get_logger().info(f'Path Length: {self.calculate_path_distance(result.path)}')
    #     
    #     time.sleep(0.5)
    #     return result

# -------------------------------------------------------------------------------------------

    def send_nav_goal(self, goal: PoseStamped) -> None:
        self.get_logger().info('Sending goal to action client...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self._nav_action_client.wait_for_server()
        self._nav_goal_handle = self._nav_action_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_feedback_callback
        )
        self._nav_goal_handle.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(self.get_nav_result_callback)

    def get_nav_result_callback(self, future) -> None:
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.nav_is_complete = True

    def nav_feedback_callback(self, feedback_msg) -> None:
        self._nav_feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {self._nav_feedback.distance_remaining}')

# -------------------------------------------------------------------------------------------

    def calculate_path_distance(self, path: Path) -> float:
        """Calculates the length of a path."""
        path_distance: float = 0.0
        for i, pose in enumerate(path.poses):
            prev_pose = path.poses[i-1].pose.position
            curr_pose = pose.pose.position
            path_segment_distance = math.sqrt((prev_pose.x - curr_pose.x)**2 + (prev_pose.y - curr_pose.y)**2)
            path_distance += path_segment_distance

        return path_distance

# -------------------------------------------------------------------------------------------

    def loop(self) -> None:
        self.get_logger().info(f'Current state is {self.state}, {self.nav_is_complete=}')

        if self.active_goal is not None:
            self.get_logger().info('Sending nav goal...')
            self.send_nav_goal(self.active_goal)
            self.active_goal = None

        if not self.nav_is_complete:
            return

        # change agent state once action is complete
        match self.state:
            case State.STANDBY:
                pass
            case State.ACTIVE:
                # NOTE: This needs to be run with a working service active
                self.activate_payload_mechanism()

                # TODO: add a wait or create an action_server for the payload_mechanism...

                self.active_goal = PoseStamped()
                self.active_goal.header.frame_id = 'map'
                self.active_goal.pose.position.x = -1.0
                self.active_goal.pose.position.y = -3.0
                self.active_goal.pose.orientation.w = 1.0

                self.get_logger().info('Sending a return goal to home...') 

                self.state = State.RETURNING
            case State.RETURNING:
                self.state = State.STANDBY

        self.nav_is_complete = False


# -------------------------------------------------------------------------------------------

def main(args = None) -> None:
    rclpy.init(args = args)

    # Argument stuff
    # FIX: change this so that arguments are parsed with ros-args
    # this will be where the namespace is entered
    parser = argparse.ArgumentParser(description='Mission node arguments.')
    parser.add_argument('name', help='namespace of robot')
    arguments = parser.parse_args()

    node = AgentNode(arguments.name)
    node.get_logger().info('Initialised agent node!')

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

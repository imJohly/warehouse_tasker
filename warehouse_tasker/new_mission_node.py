from dataclasses import dataclass

import math

import lkh
from lkh import LKHProblem

import rclpy
from rclpy.clock import Duration
from rclpy.node import Node, Subscription
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import Client, MultiThreadedExecutor

from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TransformStamped

from warehouse_tasker_interfaces.srv import Register, SendTask, SendPose

@dataclass
class Thing:
    id:             str
    pose:           Pose
    occupied:       bool = False

@dataclass
class Task:
    id:             int
    agent_id:       str
    goal:           list[str]

class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_node')

        # ROS Parameters
        self.declare_parameter('goal_count', 10)

        # Object variables
        self.current_state: MissionState    = IdleState(self)

        self._goals: dict[str, Thing]       = {}
        self._agents: dict[str, Thing]      = {}
        self._tasks: list[Task]             = []
        self._task_count: int               = 0
        self._current_task: Task | None     = None
        self._paths_to_execute: dict[str, list[Pose]]

        # Subscribers
        # HACK: Might even be able to delete this as there is no real need to store the subscribers
        self._agent_pose_subscribers: list[Subscription] = []
        self._agent_state_subscribers: list[Subscription] = []

        # Services
        self._registration_service              = self.create_service(Register, 'register_agent', self.registration_callback)
        self._task_service                      = self.create_service(SendTask, 'send_task', self.task_callback)

        # Service Clients
        self._agent_goal_clients: dict[str, Client] = {}

        # Transform buffer and listener to get marker tfs
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)

        self.initialise_goals(self.get_parameter('goal_count').get_parameter_value().integer_value)
        self.loop_timer = self.create_timer(1.0, self.run)
        self.callback_group = ReentrantCallbackGroup()

# ----------------------------------------------------------------------------

    def transition_to(self, new_state) -> None:
        """Transitions the current state of the Mission node.
    
        Arguments:
            new_state - The new state to transition into

        Returns:
            None
        """
        self.current_state.on_exit()
        self.current_state = new_state
        self.current_state.on_enter()

    def run(self) -> None:
        """Begins state machine execution"""
        self.current_state.execute()

    def initialise_goals(self, goal_count: int) -> None:
        """Initialises the goal transforms.

        Arguments:
            goal_count - amount of goals to initialise

        Returns:
            None
        """
        if goal_count <= 0:
            self.get_logger().error('Failed to initialise goals! Shutting down...')
            rclpy.shutdown()
            return

        for goal_index in range(goal_count):
            goal_tf = self.get_marker_transform(f'{goal_index}')

            if goal_tf is None:
                self.get_logger().warn(f'Failed to initialise goal {goal_index}, could not find goal tf!')
                continue

            goal_pose = Pose()
            goal_pose.position.x = goal_tf.transform.translation.x
            goal_pose.position.y = goal_tf.transform.translation.y
            goal_pose.position.z = goal_tf.transform.translation.z

            goal_pose.orientation = goal_tf.transform.rotation

            self._goals[f'{goal_index}'] = Thing(id=str(goal_index), pose=goal_pose)

        self.get_logger().info(f'Initialised {goal_count} goals.')

    def get_marker_transform(self, id: str) -> TransformStamped | None:
        """Gets marker transform from ID of marker."""
        attempts = 0
        while attempts < 5:
            try:
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    target_frame='map',
                    source_frame=f'marker{id}',
                    time=self.get_clock().now(),
                    timeout=Duration(nanoseconds=50000)
                )
                self.get_logger().info(f'Successfully found tf from "map" to "marker{id}" frame!')
                return transform

            except BaseException as e:
                self.get_logger().error(f'Error getting transform from "map" to "marker{id}" frame!')
                rclpy.spin_once(self)
            
            attempts += 1

# ----------------------------------------------------------------------------

    # NOTE: TOPIC CALLBACKS

    def agent_pose_callback(self, msg: PoseWithCovarianceStamped, agent: str) -> None:
        self.get_logger().info(f'Got new pose [x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y}] from {agent}')
        self._agents[agent].pose = msg.pose.pose

    def agent_state_callback(self, msg: Bool, agent: str) -> None:
        self.get_logger().info(f'Got new state, {msg.data} of agent {agent}')
        self._agents[agent].occupied = msg.data

# ----------------------------------------------------------------------------

    # NOTE: SERVICE CALLBACKS

    def registration_callback(self, request: Register.Request, response: Register.Response):
        if request.id in self._agents:
            self.get_logger().warn(f'Agent {request.id} already registered! Skipping...')
        else:
            new_goal_client = self.create_client(SendPose, f'{request.id}/send_goal')
            self._agent_goal_clients[f'{request.id}'] = new_goal_client

            # Create a subscriber
            pose_topic = f'{request.id}/amcl_pose'
            self._agent_pose_subscribers.append(self.create_subscription(
                msg_type=PoseWithCovarianceStamped,
                topic=pose_topic,
                callback=lambda msg, agent=f'{request.id}': self.agent_pose_callback(msg, agent),
                qos_profile=10
            ))
            self.get_logger().info(f'Subscribed to {pose_topic}')

            # Create a subscriber
            state_topic = f'{request.id}/agent_state'
            self._agent_pose_subscribers.append(self.create_subscription(
                msg_type=Bool,
                topic=state_topic,
                callback=lambda msg, agent=f'{request.id}': self.agent_state_callback(msg, agent),
                qos_profile=10
            ))
            self.get_logger().info(f'Subscribed to {state_topic}')

            new_agent = Thing(id=request.id, pose=Pose())
            self._agents[request.id] = new_agent
            self.get_logger().info(f'Successfully registered Agent {request.id} and its services!')

        response.success = True
        return response

    def task_callback(self, request: SendTask.Request, response: SendTask.Response):
        # Receive tasks and store them
        # Request is a list
        if request.goal == '':
            self.get_logger().error('No goals requested in task, ignoring task...')
            response.success = False
            return

        self._task_count += 1
        self._tasks.append(Task(
            id=self._task_count,
            agent_id=request.agent,
            goal=request.goal
        ))

        response.success = True
        return response

# ----------------------------------------------------------------------------

    # NOTE: SERVICE CLIENT CALL FUNCTIONS

    def send_goals(self, agent_id: str, goals: list[Pose]) -> None:
        agent_client = self._agent_goal_clients[f'{agent_id}']
        if agent_client is None:
            self.get_logger().error(f'No client found with agent id {agent_id}')
            return

        self.get_logger().warn(f'Waiting for {agent_client.srv_name}...')
        while not agent_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{agent_client.srv_name} service not available, waiting again...')

        req = SendPose.Request()
        req.pose = goals
        self.get_logger().info(f'Sending Goal {goals} request to agent {agent_id}...')

        agent_client.call_async(req)

# ----------------------------------------------------------------------------

    # HACK: Is it possible to set LKH to calculate with set starting nodes? Depots only allow one
    def create_tsp(self, current_task: Task) -> tuple[LKHProblem, dict[int, str]]:
        """Create a TSP from the input task."""
        node_count:  int            = 1
        association: dict[int, str] = {}

        tsp = LKHProblem()
        tsp.name                    = 'problem'
        tsp.salesmen                = len(self._agents)
        tsp.type                    = 'TSP'
        tsp.edge_weight_type        = 'EUC_2D'

        node_coords: dict[int, tuple[float, float]] = {}
        for id in current_task.goal:
            pose = self._goals[id].pose
            node_coords[node_count] = (pose.position.x, pose.position.y)
            association[node_count] = id
            node_count += 1

        tsp.dimension = len(node_coords)
        tsp.node_coords = node_coords

        return tsp, association

    # HACK: This will find the robot that is closest to the first node and assign it that path
    def assign_paths(self, paths: list[list[int]]) -> dict[str, list[str]]:
        """Assigns the input paths to the closest agents."""
        paths_to_execute = {}
        for path in paths:
            self.get_logger().info(f'Finding appropriate agent for path {path}...')
            first_goal_position = self._goals[f'{path[0]}'].pose.position

            # find closest agent
            smallest_dist: float = 1000.0
            closest_agent: Thing | None = None
            free_agents: list[Thing] = [agent for agent in self._agents.values() if not agent.occupied]
            for agent in free_agents:
                new_dist = math.dist([first_goal_position.x, first_goal_position.y], 
                                     [agent.pose.position.x, agent.pose.position.y])

                if new_dist < smallest_dist:
                    smallest_dist = new_dist
                    closest_agent = agent

            if closest_agent is None:
                self.get_logger().warn('Could not find a free agent! Skipping...')
                continue

            paths_to_execute[closest_agent.id] = path
            self._agents[closest_agent.id].occupied = True
            self.get_logger().info(f'Assigning {closest_agent.id} to {path}!')

        return paths_to_execute

# ----------------------------------------------------------------------------

class MissionState:
    def __init__(self, node) -> None:
        self.node: MissionNode = node

    def on_enter(self):
        pass

    def on_exit(self):
        pass

    def execute(self):
        pass

class IdleState(MissionState):
    def execute(self):
        self.node.get_logger().info('Idle: Waiting for tasks...')
        
        if self.node._tasks:
            self.node.transition_to(TaskSelectionState(self.node))

class TaskSelectionState(MissionState):
    def execute(self):
        self.node.get_logger().info('TaskSelection: Selecting a task')

        if self.node._tasks:
            self.node._current_task = self.node._tasks.pop(0)
            self.node.transition_to(ComputeGoalsState(self.node))
        else:
            self.node.transition_to(IdleState(self.node))

# FIX: When there are less than 3 goals, assign multiple. Currently only
# assigns the closest single robot.
class ComputeGoalsState(MissionState):
    def execute(self):
        self.node.get_logger().info(f'ComputeGoals: Computing goals for task {self.node._current_task}')

        if self.node._current_task is None:
            self.node.get_logger().error('No current task selected. Going back to Idle state...')
            self.node.transition_to(IdleState)
            return

        all_paths = []
        if len(self.node._current_task.goal) < 3:
            all_paths = [[int(goal) for goal in self.node._current_task.goal]]
        else: 
            tsp, thing_association = self.node.create_tsp(self.node._current_task)
            solution = lkh.solve(problem=tsp)

            # convert nodes into goal index
            for node_path in solution:
                all_paths.append([thing_association[node] for node in node_path])

        # assign paths based on how close agents are to the first node in a path
        assigned_node_paths = self.node.assign_paths(all_paths)
        self.node.get_logger().info(f'Nodes to execute: {assigned_node_paths}')

        # convert goals to poses
        self.node._paths_to_execute = {}
        for agent, goals in assigned_node_paths.items():
            self.node._paths_to_execute[agent] = [self.node._goals[str(i)].pose for i in goals]

        self.node.get_logger().info(f'Assigned Pose List: {self.node._paths_to_execute}')

        self.node.transition_to(ExecuteTaskState(self.node))
   
class ExecuteTaskState(MissionState):
    def execute(self):
        self.node.get_logger().info(f'TaskCompleted: Completing task {self.node._current_task}')


        self.node.get_logger().info(f'Task has {self.node._paths_to_execute}')
        for agent, path in self.node._paths_to_execute.items():
            self.node.send_goals(agent_id=agent, goals=path)

        self.node._current_task = None

        self.node.transition_to(TaskSelectionState(self.node))

# ----------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)

    node = MissionNode()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

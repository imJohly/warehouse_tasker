from dataclasses import dataclass

import math
from os import close

import lkh
from lkh import LKHProblem

import rclpy
from rclpy.clock import Duration
from rclpy.node import Node, Subscription
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import Client, Future, MultiThreadedExecutor

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, TransformStamped

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
    goal_id:        list[str]
    complete:       bool = False    # FIX: Check if I need them.

@dataclass
class Path:
    path_length:    float
    agent_id:       str
    goal_id:        str

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
        self._paths_to_execute: dict[str, list[str]]

        # Subscribers
        # HACK: Might even be able to delete this as there is no real need to store the subscribers
        self._agent_pose_subscribers: list[Subscription] = []

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
            new_state - The new state to transition to

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
        """Initialises the goal transforms."""
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

    def compute_tsp_solution(self, agents: list[Thing], goals: list[Thing]) -> list[str]:

        solution: list[str] = []

        return solution

# ----------------------------------------------------------------------------

    # NOTE: TOPIC CALLBACKS

    def agent_pose_callback(self, msg: PoseWithCovarianceStamped, agent: str) -> None:
        self.get_logger().info(f'Got new pose [x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y}] from {agent}')
        self._agents[agent].pose = msg.pose.pose

# ----------------------------------------------------------------------------

    # NOTE: SERVICE CALLBACKS

    def registration_callback(self, request: Register.Request, response: Register.Response):
        if request.id in self._agents:
            self.get_logger().warn(f'Agent {request.id} already registered! Skipping...')
        else:
            new_goal_client = self.create_client(SendPose, f'{request.id}/send_goal')
            self._agent_goal_clients[f'{request.id}'] = new_goal_client

            # Create a subscriber
            topic = f'{request.id}/amcl_pose'
            self._agent_pose_subscribers.append(self.create_subscription(
                msg_type=PoseWithCovarianceStamped,
                topic=topic,
                callback=lambda msg, agent=f'{request.id}': self.agent_pose_callback(msg, agent),
                qos_profile=10
            ))
            self.get_logger().info(f'Subscribed to {topic}')

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
            goal_id=request.goal
        ))

        response.success = True
        return response

# ----------------------------------------------------------------------------

    # NOTE: SERVICE CLIENT CALL FUNCTIONS

    def send_goal(self, agent_id: str, goals: list[str], goal_pose: PoseStamped) -> None:
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

# FIX: Is it possible to set LKH to calculate with set starting nodes? Depots only allow one
class ComputeGoalsState(MissionState):
    def execute(self):
        if self.node._current_task is None:
            self.node.get_logger().error('No current task selected. Going back to Idle state...')
            self.node.transition_to(IdleState)
            return

        self.node.get_logger().info(f'ComputeGoals: Computing goals for task {self.node._current_task}')

        # Keep track of what node is associated with which agent or goal
        thing_association: dict[int, str] = {}

        # Create TSProblem
        p = LKHProblem()
        p.name = 'problem'
        p.salesmen = len(self.node._agents)
        p.type = 'TSP'
        
        p.edge_weight_type = 'EUC_2D'

        node_count = 1

        for id in self.node._current_task.goal_id:
            pose = self.node._goals[id].pose
            p.node_coords[node_count] = [pose.position.x, pose.position.y]
            thing_association[node_count] = id

            node_count += 1
            self.node.get_logger().info(f'Added goal {id} to TSP!')

        p.dimension = len(p.node_coords)

        solution = lkh.solve(problem=p)

        # HACK: This will find the robot that is closest to the first node and assign it that path
        self._paths_to_execute = {}
        for path in solution:
            self.node.get_logger().info(f'Finding appropriate agent for path {path}...')
            first_goal_position = self.node._goals[thing_association[path[0]]].pose.position

            # find closest agent
            smallest_dist: float = 1000.0
            closest_agent: Thing | None = None
            free_agents: list[Thing] = [agent for agent in self.node._agents.values() if not agent.occupied]
            for agent in free_agents:
                new_dist = math.dist([first_goal_position.x, first_goal_position.y], 
                                     [agent.pose.position.x, agent.pose.position.y])

                if new_dist < smallest_dist:
                    smallest_dist = new_dist
                    closest_agent = agent

            if closest_agent is None:
                self.node.get_logger().warn('Could not find a free agent! Skipping...')
                continue

            self.node._paths_to_execute[closest_agent.id] = path
            self.node._agents[closest_agent.id].occupied = True
            self.node.get_logger().info(f'Assigning {closest_agent.id} to {path}!')

        self.node.transition_to(ExecuteTaskState(self.node))

# INFO: has multiple lists of paths to execute, sends each to the appropriate agent
class ExecuteTaskState(MissionState):
    def execute(self):
        self.node.get_logger().info(f'TaskCompleted: Completed task {self.node._current_task}')

        for agent, path in self.node._paths_to_execute.items():
            self.node.send_goal(agent, path)
    

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

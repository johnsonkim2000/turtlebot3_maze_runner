import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from std_srvs.srv import Empty
from math import sqrt, pow
import numpy as np
import random

# Define constants
FORWARD = 0
LEFT = 1
RIGHT = 2
ACTIONS = [FORWARD, LEFT, RIGHT]
LINEAR_SPEED = 0.1
ANGULAR_SPEED = 0.4
COLLISION_DISTANCE = 0.14

class QLearningMazeRunner(Node):
    def __init__(self):
        super().__init__('q_learning_maze_runner')

        # Publishers and Subscribers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Initialize state variables
        self.lidar_data = None
        self.crash = False

        # Initialize Q-learning parameters
        self.epsilon = 0.2
        self.alpha = 0.2
        self.gamma = 0.9
        self.q_table = np.zeros((16, len(ACTIONS)))  # Adjust dimensions as needed

        # Timer for control loop
        self.timer = self.create_timer(0.5, self.control_loop)

    def lidar_callback(self, msg):
        """Process LIDAR data."""
        self.lidar_data = np.array(msg.ranges)
        self.crash = np.min(self.lidar_data) < COLLISION_DISTANCE

    def compute_reward(self):
        """Compute reward for Q-learning."""
        if self.crash:
            return -100
        return 0  # Neutral reward for now

    def get_next_action(self):
        """Select next action using epsilon-greedy strategy."""
        if random.uniform(0, 1) < self.epsilon:
            return random.choice(ACTIONS)
        return np.argmax(self.q_table[0])  # Simplified for testing

    def control_loop(self):
        """Main control loop."""
        if self.lidar_data is None:
            return  # Wait for LIDAR data

        # If a crash is detected, reset simulation
        if self.crash:
            self.reset_simulation()
            return

        # Select and execute the next action
        action = self.get_next_action()
        self.execute_action(action)

    def execute_action(self, action):
        """Execute the given action."""
        twist = Twist()
        if action == FORWARD:
            twist.linear.x = LINEAR_SPEED
        elif action == LEFT:
            twist.angular.z = ANGULAR_SPEED
        elif action == RIGHT:
            twist.angular.z = -ANGULAR_SPEED
        self.vel_pub.publish(twist)

    def reset_simulation(self):
        """Reset the simulation and reposition the robot."""
        self.get_logger().warn("Crash detected. Resetting simulation...")

        # Step 1: Reset simulation
        reset_client = self.create_client(Empty, '/reset_simulation')
        while not reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for reset_simulation service...")

        reset_request = Empty.Request()
        reset_future = reset_client.call_async(reset_request)
        rclpy.spin_until_future_complete(self, reset_future)

        if reset_future.result() is not None:
            self.get_logger().info("Simulation reset successfully.")
        else:
            self.get_logger().error("Failed to reset simulation.")
            return

        # Step 2: Reposition the robot
        spawn_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for set_entity_state service...")

        initial_state = EntityState()
        initial_state.name = "turtlebot3_burger"
        initial_state.pose.position.x = 0.877488
        initial_state.pose.position.y = 0.000047
        initial_state.pose.position.z = 0.008533
        initial_state.pose.orientation.w = 1.0

        spawn_request = SetEntityState.Request()
        spawn_request.state = initial_state

        spawn_future = spawn_client.call_async(spawn_request)
        rclpy.spin_until_future_complete(self, spawn_future)

        if spawn_future.result() is not None:
            self.get_logger().info("Robot repositioned to the starting position.")
        else:
            self.get_logger().error("Failed to reposition the robot.")


def main(args=None):
    rclpy.init(args=args)
    node = QLearningMazeRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


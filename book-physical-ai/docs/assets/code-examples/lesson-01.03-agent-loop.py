"""
Lesson 1.3: Simple Agent Thinking Loop
A basic implementation of the perception-decision-action (PDA) loop

This simulation demonstrates:
- A 2D grid environment with obstacles and a goal
- An agent that senses its environment, makes decisions, and takes actions
- The continuous PDA loop that forms the foundation of Physical AI

No external dependencies required - pure Python!
"""

class Environment:
    """Represents the 2D world where the agent operates."""

    def __init__(self, width, height, obstacles, goal):
        """
        Initialize the environment.

        Args:
            width (int): Width of the grid
            height (int): Height of the grid
            obstacles (list): List of (x, y) tuples representing obstacle positions
            goal (tuple): (x, y) position of the goal
        """
        self.width = width
        self.height = height
        self.obstacles = set(obstacles)
        self.goal = goal

    def is_valid_position(self, x, y):
        """
        Check if a position is valid (within bounds and not an obstacle).

        Args:
            x (int): X coordinate
            y (int): Y coordinate

        Returns:
            bool: True if position is valid, False otherwise
        """
        # Check bounds
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False

        # Check obstacles
        if (x, y) in self.obstacles:
            return False

        return True

    def get_state(self):
        """Return the current state of the environment."""
        return {
            'obstacles': self.obstacles,
            'goal': self.goal,
            'dimensions': (self.width, self.height)
        }

    def visualize(self, agent):
        """
        Print a text-based visualization of the environment.

        Args:
            agent: The agent object to display
        """
        print()  # Blank line for readability

        for y in range(self.height):
            row = ""
            for x in range(self.width):
                if (x, y) == (agent.x, agent.y):
                    row += "A "  # Agent
                elif (x, y) == self.goal:
                    row += "G "  # Goal
                elif (x, y) in self.obstacles:
                    row += "# "  # Obstacle
                else:
                    row += ". "  # Empty space
            print(row)


class Agent:
    """
    An intelligent agent that navigates toward a goal using the PDA loop.
    """

    def __init__(self, start_x, start_y):
        """
        Initialize the agent at a starting position.

        Args:
            start_x (int): Starting X coordinate
            start_y (int): Starting Y coordinate
        """
        self.x = start_x
        self.y = start_y
        self.step_count = 0

    def sense(self, environment):
        """
        PERCEPTION: Gather information about the environment.

        In a real robot, this would read from cameras, LIDAR, GPS, etc.
        Here, we simulate sensors that detect:
        - Distance and direction to the goal
        - Whether adjacent cells are safe to move into

        Args:
            environment: The Environment object

        Returns:
            dict: Sensor readings
        """
        sensor_data = {}

        # Sensor 1: Distance and direction to goal (like GPS + compass)
        dx = environment.goal[0] - self.x
        dy = environment.goal[1] - self.y
        distance = (dx**2 + dy**2)**0.5

        sensor_data['distance_to_goal'] = distance
        sensor_data['direction_to_goal'] = (dx, dy)

        # Sensor 2: Obstacle detection in each direction (like proximity sensors)
        sensor_data['can_move_up'] = environment.is_valid_position(self.x, self.y - 1)
        sensor_data['can_move_down'] = environment.is_valid_position(self.x, self.y + 1)
        sensor_data['can_move_left'] = environment.is_valid_position(self.x - 1, self.y)
        sensor_data['can_move_right'] = environment.is_valid_position(self.x + 1, self.y)

        return sensor_data

    def think(self, sensor_data):
        """
        DECISION: Decide which action to take based on sensor data.

        This is where AI/ML would typically be used in advanced systems.
        Here, we use simple rule-based logic:
        - Try to move toward the goal
        - Avoid obstacles
        - If blocked, try alternative directions

        Args:
            sensor_data (dict): Readings from the sense() method

        Returns:
            str: Action to take ('UP', 'DOWN', 'LEFT', 'RIGHT', 'WAIT')
        """
        # Extract sensor readings
        can_move = {
            'UP': sensor_data['can_move_up'],
            'DOWN': sensor_data['can_move_down'],
            'LEFT': sensor_data['can_move_left'],
            'RIGHT': sensor_data['can_move_right']
        }

        direction_to_goal = sensor_data['direction_to_goal']
        dx, dy = direction_to_goal

        # Decision logic: prefer moving toward goal, but avoid obstacles

        # Horizontal movement (X-axis)
        if dx > 0 and can_move['RIGHT']:
            # Goal is to the right and path is clear
            return 'RIGHT'
        elif dx < 0 and can_move['LEFT']:
            # Goal is to the left and path is clear
            return 'LEFT'

        # Vertical movement (Y-axis)
        if dy > 0 and can_move['DOWN']:
            # Goal is below and path is clear
            return 'DOWN'
        elif dy < 0 and can_move['UP']:
            # Goal is above and path is clear
            return 'UP'

        # If preferred directions are blocked, try any available move
        # (This is a simple obstacle avoidance strategy)
        for action in ['RIGHT', 'LEFT', 'DOWN', 'UP']:
            if can_move[action]:
                return action

        # If completely blocked, wait
        return 'WAIT'

    def act(self, environment, action):
        """
        ACTION: Execute the chosen action by updating position.

        In a real robot, this would send commands to motors/actuators.
        Here, we update the agent's position in the grid.

        Args:
            environment: The Environment object
            action (str): Action to execute

        Returns:
            bool: True if action was successful, False otherwise
        """
        old_x, old_y = self.x, self.y

        # Calculate new position based on action
        if action == 'UP':
            new_x, new_y = self.x, self.y - 1
        elif action == 'DOWN':
            new_x, new_y = self.x, self.y + 1
        elif action == 'LEFT':
            new_x, new_y = self.x - 1, self.y
        elif action == 'RIGHT':
            new_x, new_y = self.x + 1, self.y
        else:  # WAIT
            new_x, new_y = self.x, self.y

        # Validate and execute movement
        if environment.is_valid_position(new_x, new_y):
            self.x, self.y = new_x, new_y
            self.step_count += 1
            return True
        else:
            # Movement blocked - stay in place
            return False

    def get_position(self):
        """Return current position."""
        return (self.x, self.y)


def run_simulation(environment, agent, max_steps=50):
    """
    Run the main simulation loop.

    This demonstrates the continuous PDA cycle:
    1. Sense the environment
    2. Think about what to do
    3. Act on that decision
    4. Repeat

    Args:
        environment: The Environment object
        agent: The Agent object
        max_steps (int): Maximum number of steps before giving up
    """
    print("=" * 50)
    print("AGENT SIMULATION: PERCEPTION-DECISION-ACTION LOOP")
    print("=" * 50)
    print(f"Agent starting at {agent.get_position()}")
    print(f"Goal location: {environment.goal}")
    print(f"Number of obstacles: {len(environment.obstacles)}")
    print("=" * 50)

    step = 0

    while step < max_steps:
        print(f"\n=== Step {step} ===")

        # Visualize current state
        environment.visualize(agent)

        # 1. SENSE: Gather sensor data
        sensor_data = agent.sense(environment)

        print(f"\nAgent at {agent.get_position()} | Goal at {environment.goal} | Distance: {sensor_data['distance_to_goal']:.1f}")
        print(f"Sensors: {sensor_data}")

        # 2. THINK: Decide what to do
        action = agent.think(sensor_data)
        print(f"Decision: Moving {action} toward goal")

        # 3. ACT: Execute the action
        success = agent.act(environment, action)

        if not success and action != 'WAIT':
            print(f"Warning: Could not execute action {action}")

        # Check if goal reached
        if agent.get_position() == environment.goal:
            print("\n" + "=" * 50)
            print(f"SUCCESS! Goal reached in {step + 1} steps!")
            print("=" * 50)
            environment.visualize(agent)
            return True

        step += 1

    print("\n" + "=" * 50)
    print(f"Simulation ended after {max_steps} steps without reaching goal.")
    print(f"Final position: {agent.get_position()}")
    print(f"Goal position: {environment.goal}")
    print("=" * 50)
    return False


def main():
    """
    Set up and run the simulation.

    Feel free to modify the environment configuration to experiment!
    """
    # Environment configuration
    width = 10
    height = 7

    # Define obstacles (walls the agent must navigate around)
    obstacles = [
        (2, 2), (2, 3), (2, 4),  # Vertical wall
        (3, 2), (3, 3), (3, 4),  # Vertical wall
    ]

    # Goal position
    goal = (9, 5)

    # Create environment
    env = Environment(width, height, obstacles, goal)

    # Create agent at starting position
    agent = Agent(start_x=0, start_y=5)

    # Run simulation
    run_simulation(env, agent, max_steps=50)

    # Experiment suggestions:
    print("\n" + "=" * 50)
    print("EXPERIMENT IDEAS:")
    print("=" * 50)
    print("1. Try different obstacle configurations")
    print("2. Change the goal position")
    print("3. Modify the agent's decision logic in think()")
    print("4. Add more sensors (e.g., look 2 steps ahead)")
    print("5. Track and display performance metrics")
    print("6. Create a more complex maze")
    print("=" * 50)


if __name__ == "__main__":
    main()

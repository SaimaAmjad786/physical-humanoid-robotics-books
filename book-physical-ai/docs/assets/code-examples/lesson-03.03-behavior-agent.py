"""
Behavior-Based Agent Simulation
Lesson 3.3: Hands-On Behavior-Based Agent

A complete simulation demonstrating behavior-based control with multiple behaviors,
behavior arbitration, and simple learning capabilities.

This simulation implements:
- BehaviorBasedAgent with subsumption-style architecture
- Multiple behaviors: AvoidObstacle, SeekGoal, Wander
- Simple environment with obstacles and goal
- Q-learning for behavior weight adaptation
- ASCII visualization

No external dependencies - pure Python only.
"""

import random
import math


# ============================================================================
# Environment: Represents the physical world
# ============================================================================

class Environment:
    """
    A 2D grid environment with obstacles and a goal.
    """

    def __init__(self, width=20, height=20):
        self.width = width
        self.height = height
        self.obstacles = []
        self.goal = (width - 2, height - 2)
        self._generate_obstacles()

    def _generate_obstacles(self):
        """Generate random obstacles in the environment."""
        num_obstacles = int(self.width * self.height * 0.1)  # 10% coverage
        for _ in range(num_obstacles):
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            # Don't place obstacles at start (0,0) or goal
            if (x, y) != (0, 0) and (x, y) != self.goal:
                self.obstacles.append((x, y))

    def is_obstacle(self, x, y):
        """Check if position (x, y) contains an obstacle."""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True  # Walls are obstacles
        return (x, y) in self.obstacles

    def is_goal(self, x, y):
        """Check if position (x, y) is the goal."""
        return (x, y) == self.goal

    def get_sensor_readings(self, x, y):
        """
        Simulate proximity sensors in 8 directions.
        Returns dict with distance to nearest obstacle in each direction.
        """
        directions = {
            'N': (0, -1),
            'NE': (1, -1),
            'E': (1, 0),
            'SE': (1, 1),
            'S': (0, 1),
            'SW': (-1, 1),
            'W': (-1, 0),
            'NW': (-1, -1)
        }

        sensors = {}
        max_range = 3  # Sensor range

        for direction, (dx, dy) in directions.items():
            distance = max_range
            for dist in range(1, max_range + 1):
                check_x = x + dx * dist
                check_y = y + dy * dist
                if self.is_obstacle(check_x, check_y):
                    distance = dist
                    break
            sensors[direction] = distance

        return sensors


# ============================================================================
# Behaviors: Individual reactive behaviors
# ============================================================================

class Behavior:
    """Base class for all behaviors."""

    def __init__(self, name, priority=0):
        self.name = name
        self.priority = priority  # Higher priority = more important

    def compute_action(self, agent_state, sensor_data, environment):
        """
        Compute desired action based on current state and sensors.
        Returns (dx, dy, strength) where strength is confidence (0.0-1.0)
        """
        raise NotImplementedError


class AvoidObstacle(Behavior):
    """Reactive behavior: Avoid obstacles detected by sensors."""

    def __init__(self):
        super().__init__(name="AvoidObstacle", priority=3)  # Highest priority (safety)
        self.activation_threshold = 2  # Activate when obstacle within 2 units

    def compute_action(self, agent_state, sensor_data, environment):
        """Move away from nearest obstacle."""
        x, y = agent_state['position']

        # Find closest obstacle
        min_distance = min(sensor_data.values())

        if min_distance >= self.activation_threshold:
            return (0, 0, 0.0)  # No obstacle nearby, behavior inactive

        # Calculate repulsion vector (move away from obstacles)
        repulsion_x = 0
        repulsion_y = 0

        direction_vectors = {
            'N': (0, -1), 'NE': (1, -1), 'E': (1, 0), 'SE': (1, 1),
            'S': (0, 1), 'SW': (-1, 1), 'W': (-1, 0), 'NW': (-1, -1)
        }

        for direction, distance in sensor_data.items():
            if distance < self.activation_threshold:
                dx, dy = direction_vectors[direction]
                # Stronger repulsion from closer obstacles
                strength = (self.activation_threshold - distance) / self.activation_threshold
                repulsion_x -= dx * strength
                repulsion_y -= dy * strength

        # Normalize
        magnitude = math.sqrt(repulsion_x**2 + repulsion_y**2)
        if magnitude > 0:
            repulsion_x /= magnitude
            repulsion_y /= magnitude

        # Strength based on urgency
        strength = 1.0 - (min_distance / self.activation_threshold)

        return (repulsion_x, repulsion_y, strength)


class SeekGoal(Behavior):
    """Deliberative behavior: Move toward the goal."""

    def __init__(self):
        super().__init__(name="SeekGoal", priority=2)  # Medium priority

    def compute_action(self, agent_state, sensor_data, environment):
        """Move toward goal location."""
        x, y = agent_state['position']
        goal_x, goal_y = environment.goal

        # Calculate direction to goal
        dx = goal_x - x
        dy = goal_y - y

        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.1:
            return (0, 0, 0.0)  # Already at goal

        # Normalize direction
        dx /= distance
        dy /= distance

        # Strength increases as we get closer to goal
        strength = 0.7  # Constant moderate strength

        return (dx, dy, strength)


class Wander(Behavior):
    """Exploratory behavior: Random wandering."""

    def __init__(self):
        super().__init__(name="Wander", priority=1)  # Lowest priority
        self.wander_direction = (random.uniform(-1, 1), random.uniform(-1, 1))
        self.steps_until_change = 0

    def compute_action(self, agent_state, sensor_data, environment):
        """Move in a random direction, changing occasionally."""
        # Change direction periodically
        if self.steps_until_change <= 0:
            self.wander_direction = (random.uniform(-1, 1), random.uniform(-1, 1))
            self.steps_until_change = random.randint(3, 8)

        self.steps_until_change -= 1

        dx, dy = self.wander_direction
        magnitude = math.sqrt(dx**2 + dy**2)
        if magnitude > 0:
            dx /= magnitude
            dy /= magnitude

        strength = 0.3  # Low strength - easily overridden

        return (dx, dy, strength)


# ============================================================================
# Behavior Coordination: Arbitration and learning
# ============================================================================

class BehaviorCoordinator:
    """
    Coordinates multiple behaviors using a weighted voting mechanism.
    Includes Q-learning to adapt behavior weights over time.
    """

    def __init__(self, behaviors):
        self.behaviors = behaviors
        # Initialize Q-values for behavior weights in different situations
        # State: (obstacle_nearby, near_goal)
        # Action: which behavior weights to use (conservative, balanced, aggressive)
        self.q_table = {}
        self.learning_rate = 0.1
        self.discount_factor = 0.9
        self.epsilon = 0.2  # Exploration rate
        self.last_state = None
        self.last_action = None

    def _get_state_key(self, sensor_data, agent_state, environment):
        """Convert continuous state to discrete state for Q-learning."""
        # Discretize: obstacle nearby (yes/no), near goal (yes/no)
        min_distance = min(sensor_data.values())
        obstacle_nearby = min_distance < 2

        x, y = agent_state['position']
        goal_x, goal_y = environment.goal
        dist_to_goal = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
        near_goal = dist_to_goal < 5

        return (obstacle_nearby, near_goal)

    def _get_q_value(self, state, action):
        """Get Q-value for state-action pair."""
        return self.q_table.get((state, action), 0.0)

    def _choose_action(self, state):
        """Choose coordination strategy using epsilon-greedy."""
        actions = ['conservative', 'balanced', 'aggressive']

        if random.random() < self.epsilon:
            return random.choice(actions)  # Explore
        else:
            # Exploit: choose best action
            q_values = [self._get_q_value(state, a) for a in actions]
            max_q = max(q_values)
            # Handle ties randomly
            best_actions = [a for a, q in zip(actions, q_values) if q == max_q]
            return random.choice(best_actions)

    def _get_behavior_weights(self, action):
        """Convert action to behavior weight modifiers."""
        weights = {
            'conservative': {'AvoidObstacle': 1.5, 'SeekGoal': 0.8, 'Wander': 0.5},
            'balanced': {'AvoidObstacle': 1.0, 'SeekGoal': 1.0, 'Wander': 1.0},
            'aggressive': {'AvoidObstacle': 0.7, 'SeekGoal': 1.5, 'Wander': 0.3}
        }
        return weights[action]

    def coordinate(self, agent_state, sensor_data, environment):
        """
        Combine all behavior outputs using weighted voting.
        Returns final (dx, dy) action.
        """
        # Get current state for Q-learning
        state = self._get_state_key(sensor_data, agent_state, environment)

        # Choose coordination strategy
        action = self._choose_action(state)
        behavior_weights = self._get_behavior_weights(action)

        # Collect behavior outputs
        behavior_outputs = []
        for behavior in self.behaviors:
            dx, dy, strength = behavior.compute_action(agent_state, sensor_data, environment)
            weight = behavior_weights.get(behavior.name, 1.0)
            behavior_outputs.append({
                'name': behavior.name,
                'vector': (dx, dy),
                'strength': strength,
                'weight': weight,
                'priority': behavior.priority
            })

        # Weighted voting: combine behaviors based on strength, weight, and priority
        final_dx = 0
        final_dy = 0
        total_influence = 0

        for output in behavior_outputs:
            dx, dy = output['vector']
            influence = output['strength'] * output['weight'] * output['priority']
            final_dx += dx * influence
            final_dy += dy * influence
            total_influence += influence

        # Normalize
        if total_influence > 0:
            final_dx /= total_influence
            final_dy /= total_influence

        # Update Q-values if we have previous experience
        if self.last_state is not None and self.last_action is not None:
            # Calculate reward based on outcome
            reward = self._calculate_reward(agent_state, sensor_data, environment)

            # Q-learning update
            old_q = self._get_q_value(self.last_state, self.last_action)
            max_next_q = max([self._get_q_value(state, a)
                             for a in ['conservative', 'balanced', 'aggressive']])
            new_q = old_q + self.learning_rate * (reward + self.discount_factor * max_next_q - old_q)
            self.q_table[(self.last_state, self.last_action)] = new_q

        # Remember for next update
        self.last_state = state
        self.last_action = action

        return (final_dx, final_dy, action, behavior_outputs)

    def _calculate_reward(self, agent_state, sensor_data, environment):
        """Calculate reward for the current state."""
        x, y = agent_state['position']
        goal_x, goal_y = environment.goal

        # Reward for being closer to goal
        dist_to_goal = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
        goal_reward = -dist_to_goal * 0.1

        # Penalty for being near obstacles
        min_distance = min(sensor_data.values())
        if min_distance < 1:
            obstacle_penalty = -2.0
        elif min_distance < 2:
            obstacle_penalty = -0.5
        else:
            obstacle_penalty = 0

        # Large reward for reaching goal
        if environment.is_goal(x, y):
            goal_reward += 10.0

        return goal_reward + obstacle_penalty


# ============================================================================
# Agent: The complete behavior-based agent
# ============================================================================

class BehaviorBasedAgent:
    """
    A complete behavior-based agent with multiple behaviors and learning.
    """

    def __init__(self, environment, start_position=(0, 0)):
        self.environment = environment
        self.position = start_position
        self.step_count = 0
        self.reached_goal = False

        # Create behaviors
        self.behaviors = [
            AvoidObstacle(),
            SeekGoal(),
            Wander()
        ]

        # Create coordinator with learning
        self.coordinator = BehaviorCoordinator(self.behaviors)

    def get_state(self):
        """Get current agent state."""
        return {
            'position': self.position,
            'step_count': self.step_count,
            'reached_goal': self.reached_goal
        }

    def sense(self):
        """Get sensor readings from environment."""
        x, y = self.position
        return self.environment.get_sensor_readings(x, y)

    def decide(self, sensor_data):
        """Use behavior coordination to decide action."""
        agent_state = self.get_state()
        return self.coordinator.coordinate(agent_state, sensor_data, self.environment)

    def act(self, action):
        """Execute action (move in environment)."""
        dx, dy, _, _ = action

        # Discretize to grid movement
        move_x = 1 if dx > 0.3 else (-1 if dx < -0.3 else 0)
        move_y = 1 if dy > 0.3 else (-1 if dy < -0.3 else 0)

        # Try to move
        new_x = self.position[0] + move_x
        new_y = self.position[1] + move_y

        # Check if move is valid (not into obstacle)
        if not self.environment.is_obstacle(new_x, new_y):
            self.position = (new_x, new_y)

        # Check if reached goal
        if self.environment.is_goal(new_x, new_y):
            self.reached_goal = True

        self.step_count += 1

    def step(self):
        """Execute one sense-decide-act cycle."""
        sensor_data = self.sense()
        action = self.decide(sensor_data)
        self.act(action)
        return action


# ============================================================================
# Visualization and Simulation
# ============================================================================

def visualize_environment(environment, agent):
    """Create ASCII visualization of the environment and agent."""
    # Create grid
    grid = [[' ' for _ in range(environment.width)] for _ in range(environment.height)]

    # Place obstacles
    for x, y in environment.obstacles:
        grid[y][x] = '#'

    # Place goal
    goal_x, goal_y = environment.goal
    grid[goal_y][goal_x] = 'G'

    # Place agent
    agent_x, agent_y = agent.position
    if agent.reached_goal:
        grid[agent_y][agent_x] = '*'  # Agent at goal
    else:
        grid[agent_y][agent_x] = 'A'

    # Print grid
    print('\n' + '=' * (environment.width + 2))
    for row in grid:
        print('|' + ''.join(row) + '|')
    print('=' * (environment.width + 2))
    print(f"Legend: A=Agent, G=Goal, #=Obstacle, *=Success")


def run_simulation(max_steps=100, visualize_every=10):
    """Run the complete behavior-based agent simulation."""
    print("=" * 60)
    print("BEHAVIOR-BASED AGENT SIMULATION")
    print("=" * 60)
    print("\nInitializing environment and agent...")

    # Create environment and agent
    env = Environment(width=20, height=20)
    agent = BehaviorBasedAgent(env, start_position=(1, 1))

    print(f"Environment: {env.width}x{env.height}")
    print(f"Obstacles: {len(env.obstacles)}")
    print(f"Goal: {env.goal}")
    print(f"Start: {agent.position}")
    print(f"Behaviors: {', '.join([b.name for b in agent.behaviors])}")

    # Initial visualization
    print("\n--- Initial State ---")
    visualize_environment(env, agent)

    print(f"\nRunning simulation (max {max_steps} steps)...\n")

    # Run simulation
    for step in range(max_steps):
        # Execute one step
        action = agent.step()
        dx, dy, strategy, behavior_outputs = action

        # Periodic visualization
        if step % visualize_every == 0:
            print(f"\n--- Step {step} ---")
            print(f"Position: {agent.position}")
            print(f"Strategy: {strategy}")
            print(f"Action: dx={dx:.2f}, dy={dy:.2f}")
            print("\nBehavior Activations:")
            for output in behavior_outputs:
                print(f"  {output['name']}: strength={output['strength']:.2f}, "
                      f"weight={output['weight']:.2f}, priority={output['priority']}")
            visualize_environment(env, agent)

        # Check if goal reached
        if agent.reached_goal:
            print(f"\n{'='*60}")
            print(f"SUCCESS! Goal reached in {agent.step_count} steps!")
            print(f"{'='*60}")
            visualize_environment(env, agent)
            return True

    # Failed to reach goal
    print(f"\n{'='*60}")
    print(f"Simulation ended after {max_steps} steps without reaching goal.")
    print(f"Final position: {agent.position}")
    print(f"Distance to goal: {math.sqrt((env.goal[0]-agent.position[0])**2 + (env.goal[1]-agent.position[1])**2):.2f}")
    print(f"{'='*60}")
    visualize_environment(env, agent)

    # Show learned Q-values
    print("\nLearned Q-values (behavior coordination strategies):")
    for state_action, q_value in sorted(agent.coordinator.q_table.items()):
        state, action = state_action
        print(f"  State {state}, Action '{action}': Q={q_value:.3f}")

    return False


# ============================================================================
# Main execution
# ============================================================================

if __name__ == "__main__":
    # Run the simulation
    random.seed(42)  # For reproducibility
    success = run_simulation(max_steps=100, visualize_every=10)

    print("\n" + "="*60)
    print("SIMULATION COMPLETE")
    print("="*60)
    print("\nKey Takeaways:")
    print("1. Reactive behaviors (AvoidObstacle) provide immediate safety")
    print("2. Deliberative behaviors (SeekGoal) drive goal achievement")
    print("3. Exploratory behaviors (Wander) enable exploration")
    print("4. Behavior coordination balances competing objectives")
    print("5. Q-learning adapts coordination strategy over time")
    print("\nExperiment: Try running multiple times to see learning in action!")

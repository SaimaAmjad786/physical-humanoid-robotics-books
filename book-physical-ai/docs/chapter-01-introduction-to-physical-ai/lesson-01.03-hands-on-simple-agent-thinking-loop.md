---
sidebar_position: 3
title: "Lesson 1.3: Hands-On Simple Agent Thinking Loop"
keywords: [agent loop, Python simulation, sense-think-act, hands-on coding, agent architecture]
description: Build your first Physical AI agent by implementing a simple perception-decision-action loop in Python with a practical simulation.
lesson_type: hands-on
estimated_time: 45 minutes
difficulty: beginner
---

# Lesson 1.3: Hands-On Simple Agent Thinking Loop

## Prerequisites

- Completion of Lesson 1.1 and Lesson 1.2
- Basic Python programming knowledge (variables, functions, classes, loops)
- Python 3.7 or higher installed on your computer
- A text editor or IDE (VS Code, PyCharm, or even IDLE)
- Understanding of the perception-decision-action loop concept

## What You'll Learn

By the end of this lesson, you will be able to:

- Implement a basic perception-decision-action (PDA) loop in Python
- Create a simple simulated agent with sensors, decision-making logic, and actions
- Understand how state management works in agent systems
- See the PDA loop in action through a practical simulation
- Experiment with agent behaviors by modifying code
- Debug and extend a basic agent architecture

## Introduction

In Lessons 1.1 and 1.2, we explored what Physical AI is and learned about the perception-decision-action loop that powers all embodied agents. Now it's time to get your hands dirty with code!

In this hands-on lesson, you'll build your first intelligent agent—a simple simulated robot that navigates toward a goal while avoiding obstacles. While this is a simulation (not controlling real hardware), the code structure and thinking loop you'll implement is exactly the same pattern used in real Physical AI systems.

Think of this as your first step from understanding Physical AI conceptually to building it practically. By the end of this lesson, you'll have working code that demonstrates the core principles of embodied intelligence.

Let's build something!

## The Scenario: A Goal-Seeking Agent

We'll create a simple 2D simulation where:

- An **agent** (let's call it a robot) exists on a grid
- The agent has a **goal position** it wants to reach
- There are **obstacles** in the environment the agent must avoid
- The agent uses **sensors** to detect obstacles and its distance to the goal
- The agent makes **decisions** about which direction to move
- The agent **acts** by moving one step in the chosen direction
- The loop repeats until the agent reaches its goal

This scenario, while simple, contains all the essential elements of Physical AI: perception, decision-making, action, and continuous adaptation.

## The Code Structure

Our implementation will have three main components:

1. **Environment**: Represents the 2D world with obstacles and a goal
2. **Agent**: The intelligent entity with sensors, decision-making, and actuators
3. **Simulation Loop**: Runs the PDA loop repeatedly until the goal is reached

Let's build each component step by step.

<div className="hands-on-section">

## Step-by-Step Instructions

### Step 1: Download the Code File

First, you'll need to get the Python code file. The complete implementation is available at:

**File location:** `E:\Users\hp\Documents\my-book\book-physical-ai\docs\assets\code-examples\lesson-01.03-agent-loop.py`

You can either:
- Copy the code from the file viewer below
- Download the file directly from the course materials
- Type it out yourself for maximum learning (recommended for beginners!)

### Step 2: Review the Code Structure

Open the file and you'll see it's organized into clear sections:

```python
# 1. Environment class - represents the world
# 2. Agent class - implements sense-think-act loop
# 3. Simulation function - runs the main loop
# 4. Main execution - sets up and runs the simulation
```

Let's examine each part:

### Step 3: Understand the Environment

The `Environment` class represents the 2D world:

```python
class Environment:
    def __init__(self, width, height, obstacles, goal):
        # Grid dimensions
        # List of obstacle positions
        # Goal position
```

Key methods:
- `is_valid_position(x, y)`: Checks if a position is within bounds and not an obstacle
- `get_state()`: Returns the current world state
- `visualize()`: Prints a text-based visualization of the world

### Step 4: Understand the Agent

The `Agent` class is where the magic happens. It implements the three stages of the PDA loop:

```python
class Agent:
    def sense(self, environment):
        # PERCEPTION: Gather sensor data

    def think(self, sensor_data):
        # DECISION: Choose best action based on sensor data

    def act(self, environment, action):
        # ACTION: Execute the chosen action
```

**Key insight:** Notice how `sense()`, `think()`, and `act()` are separate methods. This separation is a fundamental design pattern in Physical AI—it makes the code modular, testable, and easier to understand.

### Step 5: Run the Basic Simulation

Now let's run the code and see it in action!

**On Windows:**
```bash
python "E:\Users\hp\Documents\my-book\book-physical-ai\docs\assets\code-examples\lesson-01.03-agent-loop.py"
```

**On Mac/Linux:**
```bash
python3 /path/to/lesson-01.03-agent-loop.py
```

You should see output like this:

```
=== Step 0 ===
. . . . . . . . . .
. . . . . . . . . .
. . # # . . . . . .
. . # # . . . . . .
. . . . . . . . . .
A . . . . . . . . G
. . . . . . . . . .

Agent at (0, 5) | Goal at (9, 5) | Distance: 9.0
Sensors: {'distance_to_goal': 9.0, 'can_move_up': True, 'can_move_down': True, ...}
Decision: Moving RIGHT toward goal

=== Step 1 ===
. . . . . . . . . .
. . . . . . . . . .
. . # # . . . . . .
. . # # . . . . . .
. . . . . . . . . .
. A . . . . . . . G
. . . . . . . . . .

Agent at (1, 5) | Goal at (9, 5) | Distance: 8.0
...
```

**What's happening:**
- The agent (A) starts at the left side
- The goal (G) is on the right side
- Obstacles (#) are in the middle
- Each step shows the agent's sensors, decision, and resulting movement
- The agent navigates around obstacles to reach the goal

### Step 6: Analyze the Perception Stage

Let's look closely at the `sense()` method:

```python
def sense(self, environment):
    """
    PERCEPTION: Gather information about the environment.
    In a real robot, this would read from cameras, LIDAR, GPS, etc.
    """
    sensor_data = {}

    # Sensor 1: Distance and direction to goal
    dx = environment.goal[0] - self.x
    dy = environment.goal[1] - self.y
    sensor_data['distance_to_goal'] = (dx**2 + dy**2)**0.5
    sensor_data['direction_to_goal'] = (dx, dy)

    # Sensor 2: Obstacle detection in each direction
    sensor_data['can_move_up'] = environment.is_valid_position(self.x, self.y - 1)
    sensor_data['can_move_down'] = environment.is_valid_position(self.x, self.y + 1)
    sensor_data['can_move_left'] = environment.is_valid_position(self.x - 1, self.y)
    sensor_data['can_move_right'] = environment.is_valid_position(self.x + 1, self.y)

    return sensor_data
```

**Key concepts:**
- **Multiple sensors**: Just like a real robot uses cameras, LIDAR, etc., our agent combines multiple sources of information
- **Sensor abstraction**: The agent doesn't directly access the environment's internal state—it only uses sensor readings
- **Structured data**: Sensors return a dictionary that the thinking stage can easily process

**Real-world parallel:** This is similar to how a self-driving car might combine GPS (distance/direction to destination) with proximity sensors (obstacles in each direction).

### Step 7: Analyze the Decision Stage

Now let's examine the `think()` method:

```python
def think(self, sensor_data):
    """
    DECISION: Decide which action to take based on sensor data.
    This is where AI/ML would typically be used in advanced systems.
    """
    # Extract sensor readings
    can_move = {
        'UP': sensor_data['can_move_up'],
        'DOWN': sensor_data['can_move_down'],
        'LEFT': sensor_data['can_move_left'],
        'RIGHT': sensor_data['can_move_right']
    }

    direction_to_goal = sensor_data['direction_to_goal']

    # Simple decision logic: prefer moving toward goal, but avoid obstacles
    if direction_to_goal[0] > 0 and can_move['RIGHT']:
        return 'RIGHT'
    elif direction_to_goal[0] < 0 and can_move['LEFT']:
        return 'LEFT'
    elif direction_to_goal[1] > 0 and can_move['DOWN']:
        return 'DOWN'
    elif direction_to_goal[1] < 0 and can_move['UP']:
        return 'UP'

    # If preferred direction is blocked, try alternatives
    for action in ['RIGHT', 'LEFT', 'DOWN', 'UP']:
        if can_move[action]:
            return action

    return 'WAIT'
```

**Key concepts:**
- **Goal-directed behavior**: The agent tries to move toward the goal
- **Obstacle avoidance**: It checks if the preferred direction is safe
- **Fallback logic**: If the direct path is blocked, it tries alternative directions
- **Reactive decision-making**: Decisions are based on current sensor readings

**Important note:** This is a simple rule-based system. In advanced Physical AI, this `think()` method might use machine learning, path planning algorithms, or reinforcement learning. But the structure remains the same!

### Step 8: Analyze the Action Stage

Finally, the `act()` method:

```python
def act(self, environment, action):
    """
    ACTION: Execute the chosen action by updating position.
    In a real robot, this would send commands to motors/actuators.
    """
    old_x, old_y = self.x, self.y

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
        return True
    return False
```

**Key concepts:**
- **Action validation**: The agent double-checks that the action is safe before executing
- **State update**: The agent's position changes based on the action
- **Failure handling**: If the action can't be executed, the method returns `False`

**Real-world parallel:** This is like sending motor commands to a robot's wheels—the robot calculates the desired wheel speeds and sends electrical signals to make it happen.

### Step 9: See the Complete Loop

The main simulation loop ties everything together:

```python
def run_simulation(environment, agent, max_steps=50):
    step = 0

    while step < max_steps:
        # 1. SENSE
        sensor_data = agent.sense(environment)

        # 2. THINK
        action = agent.think(sensor_data)

        # 3. ACT
        success = agent.act(environment, action)

        # Visualize and check goal
        environment.visualize(agent)

        # Check if goal reached
        if (agent.x, agent.y) == environment.goal:
            print(f"\nGoal reached in {step + 1} steps!")
            break

        step += 1
```

**This is the PDA loop in action!** Each iteration:
1. Senses the environment
2. Thinks about what to do
3. Acts on that decision
4. Repeats

### Step 10: Experiment and Extend

Now comes the fun part—experimenting with the code! Try these modifications:

#### Experiment 1: Change the Environment

Modify the obstacle positions or goal location:

```python
# In the main() function, try different configurations:

# Easy: No obstacles
obstacles = []

# Hard: Create a maze
obstacles = [(2,2), (2,3), (2,4), (2,5), (4,0), (4,1), (4,2), (4,3)]

# Different goal
goal = (5, 0)  # Top-middle instead of right-middle
```

**Question to explore:** How does the agent's behavior change with different obstacle configurations?

#### Experiment 2: Improve the Decision Logic

The current `think()` method is very simple. Can you make it smarter?

Try adding:
- **Look-ahead**: Check if moving in a direction will lead to a dead end
- **Distance weighting**: Prefer actions that reduce distance to goal most
- **Memory**: Remember recently visited positions to avoid getting stuck in loops

```python
def think(self, sensor_data):
    # Add your improved logic here!
    # Ideas:
    # - Calculate distance to goal for each possible move
    # - Avoid moving back to previous position
    # - Handle situations where agent gets stuck
    pass
```

#### Experiment 3: Add More Sensors

What if the agent could see multiple steps ahead?

```python
def sense(self, environment):
    sensor_data = {}

    # ... existing sensors ...

    # NEW: Look 2 steps ahead in each direction
    sensor_data['clear_ahead_right'] = (
        environment.is_valid_position(self.x + 1, self.y) and
        environment.is_valid_position(self.x + 2, self.y)
    )

    # Add similar for other directions

    return sensor_data
```

**Question to explore:** How does additional sensor information improve the agent's navigation?

#### Experiment 4: Track Performance Metrics

Add code to measure how efficiently the agent reaches the goal:

```python
# Track total distance traveled
total_distance = 0

# Track number of times agent got stuck
stuck_count = 0

# Calculate optimality (shortest possible path vs. actual path)
optimal_path_length = abs(start[0] - goal[0]) + abs(start[1] - goal[1])
efficiency = optimal_path_length / step
```

## Expected Output

When you run the basic simulation, you should see:

1. **Initial state visualization** showing agent (A), goal (G), and obstacles (#)
2. **Step-by-step progression** with:
   - Current positions
   - Sensor readings
   - Decision made
   - Updated visualization
3. **Goal reached message** with total steps taken

**Successful output example:**

```
=== Step 0 ===
. . . . . . . . . .
. . . . . . . . . .
. . # # . . . . . .
. . # # . . . . . .
. . . . . . . . . .
A . . . . . . . . G
. . . . . . . . . .

...

=== Step 12 ===
. . . . . . . . . .
. . . . . . . . . .
. . # # . . . . . .
. . # # . . . . . .
. . . . . . . . . .
. . . . . . . . . A

Goal reached in 13 steps!
```

## Troubleshooting

### Problem: "ModuleNotFoundError" or "ImportError"

**Cause:** Python is not finding the script or has incorrect path

**Solution:**
- Make sure you're in the correct directory
- Use the full absolute path to the Python file
- Check that Python is properly installed: `python --version`

### Problem: Agent gets stuck and doesn't reach goal

**Cause:** The simple decision logic can get trapped in local minima (corners, U-shaped obstacles)

**Solution:**
- This is actually expected behavior with simple rule-based logic!
- The `max_steps` parameter prevents infinite loops
- Try implementing one of the improvements suggested in Step 10

### Problem: Agent takes a very inefficient path

**Cause:** The greedy algorithm only looks at immediate next step

**Solution:**
- This is a learning opportunity—simple reactive behavior isn't always optimal
- Try implementing look-ahead logic or path planning
- This demonstrates why real robots use more sophisticated algorithms

### Problem: Visualization is hard to read

**Cause:** Terminal window might be too small or formatting issues

**Solution:**
- Maximize your terminal window
- Try reducing the grid size in the `main()` function
- You can modify the visualization characters for better clarity

## Connecting to Real Physical AI

You might be thinking: "This is just a simulation—how does it relate to real robots?"

**Great question!** The code structure you just implemented is directly applicable to real Physical AI:

### Real Robot Equivalent

**Simulation**:
```python
sensor_data['can_move_right'] = environment.is_valid_position(self.x + 1, self.y)
```

**Real Robot**:
```python
sensor_data['can_move_right'] = (lidar.get_distance(90_degrees) > 0.5)  # 0.5 meters clearance
```

---

**Simulation**:
```python
if action == 'RIGHT':
    self.x += 1
```

**Real Robot**:
```python
if action == 'RIGHT':
    motor_controller.set_wheel_speeds(left=0.5, right=0.5)  # Move forward
    time.sleep(0.1)  # Execute for 100ms
```

### The Pattern Remains the Same

Whether you're controlling a:
- Simulated 2D agent (like you just built)
- Simple wheeled robot
- Sophisticated humanoid robot
- Autonomous vehicle

**The pattern is always:**
1. **Sense**: Read from sensors (simulated grid check → real LIDAR/camera)
2. **Think**: Process data and decide (same logic, more sophisticated algorithms)
3. **Act**: Execute action (position update → motor commands)
4. **Repeat**: Continuous loop

This is the power of the PDA architecture—it scales from simple simulations to complex real-world systems!

</div>

## Self-Assessment

Test your understanding by answering these questions:

<details>
<summary><strong>Question 1:</strong> In the code, why are `sense()`, `think()`, and `act()` implemented as separate methods instead of one big method?</summary>

**Answer:**

Separating the PDA loop into distinct methods provides several critical benefits:

1. **Modularity**: Each method has a single, clear responsibility, making the code easier to understand and maintain
2. **Testability**: You can test each stage independently (e.g., test decision logic without worrying about sensors)
3. **Flexibility**: You can swap out decision algorithms or sensor types without affecting other parts
4. **Reusability**: The same sensing code can be used with different decision strategies
5. **Real-world alignment**: This matches how real robots are designed—perception, planning, and control are often separate subsystems

This design pattern is fundamental to robotics software architecture and is used in frameworks like ROS (Robot Operating System).
</details>

<details>
<summary><strong>Question 2:</strong> What happens if you remove the `environment.is_valid_position()` check in the `act()` method? Why is this validation important?</summary>

**Answer:**

Without the validation check in `act()`, the agent could:
- Move outside the grid boundaries (causing crashes or invalid array access)
- Move through obstacles (violating physics and collision constraints)
- Enter invalid states that break the simulation

**Why validation is important:**

In simulation: Prevents bugs and maintains world consistency

In real robots: This represents **safety-critical** validation. Real robots must verify:
- Movement commands won't cause collisions
- Actions respect physical limits (max speed, joint angles, weight capacity)
- Commands won't damage the robot or environment
- Emergency stops can override dangerous commands

This double-checking (decision stage suggests action, action stage validates before executing) is a key safety pattern in Physical AI—never blindly execute commands without verification!
</details>

<details>
<summary><strong>Question 3:</strong> The current `think()` method uses simple if-else rules. How would you modify it to handle the case where the agent gets stuck in a corner? Describe your approach.</summary>

**Answer:**

Several approaches to handle getting stuck:

**Approach 1: Memory-based (simple)**
```python
def __init__(self):
    self.previous_positions = []

def think(self, sensor_data):
    # ... normal logic ...

    # If all preferred moves lead to recently visited positions, try random direction
    if current_position in self.previous_positions[-5:]:
        return random.choice([d for d in directions if can_move[d]])
```

**Approach 2: Stuck detection**
```python
def think(self, sensor_data):
    if self.distance_to_goal == self.last_distance:
        self.stuck_counter += 1
    else:
        self.stuck_counter = 0

    # If stuck for 3+ steps, try a random different direction
    if self.stuck_counter > 3:
        return self.explore_random_direction()
```

**Approach 3: Escape behavior**
```python
def think(self, sensor_data):
    # If stuck, temporarily move AWAY from goal to escape local minimum
    if self.is_stuck():
        # Move perpendicular to goal direction
        return self.find_escape_direction()
```

The best approach depends on the environment complexity, but all share the idea of detecting stuck states and breaking out of them with exploratory behavior—a common pattern in robotics called "escape behaviors" or "random exploration."
</details>

## What's Next

Congratulations! You've just implemented your first intelligent agent with a complete perception-decision-action loop. You've moved from theory to practice and created working code that demonstrates the fundamental architecture of Physical AI systems.

This is just the beginning. In the chapters ahead, you'll:

- Learn about more sophisticated sensor processing and computer vision
- Explore advanced decision-making algorithms including machine learning and reinforcement learning
- Dive into robot motion planning and control
- Understand how to work with real hardware and handle real-world complexities
- Build increasingly sophisticated agents that can handle more complex tasks

**For now, take time to:**
- Experiment with the code modifications suggested in Step 10
- Try to break the agent's behavior and then fix it
- Think about how you might apply this pattern to different problems
- Share your experiments with others learning Physical AI

Remember: every complex Physical AI system, from Mars rovers to surgical robots, is built on the same fundamental pattern you just implemented. You're well on your way to building amazing things!

## Summary

Let's recap the key concepts and skills from this hands-on lesson:

- **You implemented a complete PDA loop** with three distinct methods (`sense()`, `think()`, `act()`) that work together in a continuous cycle—this is the exact pattern used in real Physical AI systems

- **The Environment class** represents the external world with state, physics rules, and validation logic—separating the environment from the agent is crucial for modularity and testing

- **Perception (sensing)** combines multiple information sources (distance to goal, obstacle detection in all directions) into structured sensor data, demonstrating the principle of sensor fusion

- **Decision-making (thinking)** processes sensor data using goal-directed logic with fallback behaviors—while this implementation uses simple rules, the structure is identical to what machine learning-based systems use

- **Action execution** validates commands before executing them and updates the agent's state—this double-checking pattern is critical for safety in real robots

- **The simulation loop** runs the PDA cycle repeatedly until the goal is reached or max steps exceeded, showing how continuous adaptation enables goal achievement in dynamic environments

- **Modularity and separation of concerns** make the code testable, extensible, and easy to understand—these software engineering principles are essential for building reliable Physical AI systems

- **From simulation to reality**: The code pattern you implemented translates directly to real robots by swapping simulated sensors/actuators with real hardware interfaces

You now have hands-on experience with the core architecture that powers every Physical AI system. This foundation will serve you throughout your journey in building intelligent embodied agents!

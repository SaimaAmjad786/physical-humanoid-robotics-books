---
title: "Lesson 3.3: Hands-On - Behavior-Based Agent"
chapter: 3
lesson: 3
description: "Build a complete behavior-based agent with multiple behaviors, coordination mechanisms, and simple learning. This final hands-on lesson integrates everything you've learned across all three chapters."
learning_objectives:
  - Implement a complete behavior-based agent architecture
  - Create multiple reactive behaviors with priority-based coordination
  - Integrate Q-learning for adaptive behavior selection
  - Understand how perception, decision-making, and action work together in practice
keywords: [behavior-based systems, subsumption architecture, Q-learning, agent simulation, reactive control, behavior coordination]
estimated_time: "60-75 minutes"
difficulty: intermediate
prerequisites:
  - Completion of Lessons 3.1 and 3.2
  - Understanding of control architectures and learning paradigms
  - Basic Python programming skills
---

# Lesson 3.3: Hands-On - Behavior-Based Agent

## Prerequisites

Before starting this lesson, you should:

- Have completed Lessons 3.1 (Control Systems) and 3.2 (Learning in Physical Agents)
- Understand reactive and deliberative control paradigms
- Be familiar with reinforcement learning basics (Q-learning)
- Have Python 3.7+ installed on your system
- Be comfortable reading and modifying Python code

## What You'll Learn

By the end of this lesson, you will be able to:

1. **Build a complete behavior-based agent** with multiple behaviors organized in a subsumption-style architecture
2. **Implement behavior coordination** using weighted voting and priority-based arbitration
3. **Integrate Q-learning** to adapt behavior weights based on experience
4. **Visualize agent behavior** and understand how different behaviors interact in real-time

## Introduction

Throughout this book, you've learned the fundamental concepts of Physical AI:

- **Chapter 1**: The Perception-Decision-Action loop and embodied intelligence
- **Chapter 2**: Sensors for perception and actuators for action
- **Chapter 3**: Control systems and learning algorithms

Now, it's time to bring it all together. In this hands-on lesson, you'll build a **behavior-based agent** that navigates a 2D environment with obstacles to reach a goal. The agent will have multiple competing behaviors—avoiding obstacles (reactive/safety), seeking the goal (deliberative/task-oriented), and wandering (exploratory)—that must be coordinated intelligently.

Most importantly, your agent will **learn** from experience using Q-learning to adapt its behavior coordination strategy over time. This exercise demonstrates the core principles of Physical AI: grounding intelligence in physical interaction with the environment.

## The Simulation Architecture

Our behavior-based agent consists of several key components:

### 1. Environment

Represents the physical world:
- 2D grid with obstacles and a goal
- Provides sensor readings (proximity in 8 directions)
- Validates agent movements

### 2. Behaviors (Reactive and Deliberative)

Three independent behaviors that run in parallel:

**AvoidObstacle (Reactive, Priority 3 - Highest)**
- Detects nearby obstacles using proximity sensors
- Generates repulsion vectors to move away from obstacles
- Highest priority for safety

**SeekGoal (Deliberative, Priority 2 - Medium)**
- Calculates direction toward goal
- Generates attraction vector toward goal location
- Medium priority for task achievement

**Wander (Exploratory, Priority 1 - Lowest)**
- Generates random movement directions
- Changes direction periodically
- Lowest priority, enables exploration when other behaviors are inactive

### 3. Behavior Coordinator (with Q-Learning)

Combines behavior outputs using weighted voting:
- Each behavior produces a movement vector and strength (confidence)
- Coordinator assigns weights based on current strategy (conservative, balanced, aggressive)
- Q-learning adapts strategy selection based on rewards (progress toward goal, avoiding collisions)

### 4. Agent (Sense-Decide-Act Loop)

Orchestrates the complete PDA loop:
- **Sense**: Get proximity sensor readings from environment
- **Decide**: Coordinate behaviors to select action
- **Act**: Execute movement in environment

<div className="hands-on-section">

## Step-by-Step Instructions

### Step 1: Download the Code

The complete simulation is provided in a single Python file with no external dependencies.

**File location**: `E:\Users\hp\Documents\my-book\book-physical-ai\docs\assets\code-examples\lesson-03.03-behavior-agent.py`

You can also create the file manually by copying the code from the repository or course materials.

### Step 2: Understand the Code Structure

Open the file and examine its structure. It's organized into clear sections:

```python
# 1. Environment class - the physical world
class Environment:
    # Grid world with obstacles and goal
    # Provides sensor readings

# 2. Behavior classes - individual behaviors
class Behavior:              # Base class
class AvoidObstacle:         # Reactive safety behavior
class SeekGoal:              # Deliberative goal-seeking
class Wander:                # Exploratory behavior

# 3. Coordination - combining behaviors
class BehaviorCoordinator:
    # Weighted voting
    # Q-learning for strategy adaptation

# 4. Agent - complete PDA loop
class BehaviorBasedAgent:
    # sense(), decide(), act()

# 5. Simulation and visualization
def run_simulation():
    # Main simulation loop
```

**Take a moment to read through the code comments**—they explain each component's purpose and design rationale.

### Step 3: Run the Basic Simulation

Navigate to the directory containing the file and run it:

```bash
cd E:\Users\hp\Documents\my-book\book-physical-ai\docs\assets\code-examples
python lesson-03.03-behavior-agent.py
```

You should see ASCII visualization of the agent (A) navigating through obstacles (#) to reach the goal (G).

### Step 4: Analyze the Output

The simulation displays:

**Environment Layout:**
```
====================
|A                 |
|  ###    ##       |
|    #  #  #       |
|      #           |
|   ##      ##     |
|          #  #    |
|              G   |
====================
```

**Behavior Activations at Each Step:**
```
Step 10
Position: (5, 3)
Strategy: balanced
Action: dx=0.85, dy=0.45

Behavior Activations:
  AvoidObstacle: strength=0.00, weight=1.00, priority=3
  SeekGoal: strength=0.70, weight=1.00, priority=2
  Wander: strength=0.30, weight=1.00, priority=1
```

**Key observations:**

1. **AvoidObstacle** has strength=0.00 when no obstacles are nearby, so it doesn't influence the action
2. **SeekGoal** has moderate strength (0.70) and is actively pulling the agent toward the goal
3. **Wander** has low strength (0.30) and provides gentle exploration
4. The **final action** (dx, dy) is a weighted combination of all active behaviors

### Step 5: Observe Learning in Action

The coordinator uses Q-learning to adapt its coordination strategy:

**Three strategies:**
- **Conservative**: High weight on AvoidObstacle, low on SeekGoal (safe but slow)
- **Balanced**: Equal weights on all behaviors
- **Aggressive**: High weight on SeekGoal, low on AvoidObstacle (fast but risky)

**Learning process:**

1. Initially, Q-values are zero, so strategy selection is random (exploration)
2. As the agent experiences different situations, it learns which strategies work best
3. Over time, Q-values converge to reflect strategy effectiveness

**To see learning clearly, run the simulation multiple times:**

```bash
python lesson-03.03-behavior-agent.py
python lesson-03.03-behavior-agent.py
python lesson-03.03-behavior-agent.py
```

After several runs, examine the learned Q-values printed at the end:

```
Learned Q-values (behavior coordination strategies):
  State (True, False), Action 'conservative': Q=0.234
  State (True, False), Action 'balanced': Q=-0.145
  State (True, False), Action 'aggressive': Q=-0.567
  State (False, True), Action 'aggressive': Q=0.812
  ...
```

**Interpretation:**
- State (True, False) = obstacle nearby, not near goal
- High Q-value for 'conservative' suggests this strategy works well when obstacles are nearby
- State (False, True) = no obstacle nearby, near goal
- High Q-value for 'aggressive' suggests this strategy works well when the path is clear near the goal

## Expected Output

A successful run should look like this:

```
============================================================
BEHAVIOR-BASED AGENT SIMULATION
============================================================

Initializing environment and agent...
Environment: 20x20
Obstacles: 40
Goal: (18, 18)
Start: (1, 1)
Behaviors: AvoidObstacle, SeekGoal, Wander

--- Initial State ---
====================
| A                |
|   ##    #        |
...
|               G  |
====================
Legend: A=Agent, G=Goal, #=Obstacle, *=Success

Running simulation (max 100 steps)...

--- Step 0 ---
Position: (1, 1)
Strategy: balanced
Action: dx=0.87, dy=0.87
...

--- Step 40 ---
Position: (18, 18)

============================================================
SUCCESS! Goal reached in 42 steps!
============================================================

Learned Q-values (behavior coordination strategies):
  State (False, False), Action 'balanced': Q=0.156
  State (True, False), Action 'conservative': Q=0.289
  ...

============================================================
SIMULATION COMPLETE
============================================================

Key Takeaways:
1. Reactive behaviors (AvoidObstacle) provide immediate safety
2. Deliberative behaviors (SeekGoal) drive goal achievement
3. Exploratory behaviors (Wander) enable exploration
4. Behavior coordination balances competing objectives
5. Q-learning adapts coordination strategy over time
```

## Experimentation Ideas

Now that you understand the basic simulation, try these modifications:

### Experiment 1: Adjust Behavior Priorities

**Modify the behavior priority values:**

In the code, find the behavior definitions:

```python
class AvoidObstacle(Behavior):
    def __init__(self):
        super().__init__(name="AvoidObstacle", priority=3)  # Try changing to 1

class SeekGoal(Behavior):
    def __init__(self):
        super().__init__(name="SeekGoal", priority=2)  # Try changing to 3
```

**What happens when you:**
- Set AvoidObstacle priority to 1 (lowest)? → Agent may collide more often
- Set SeekGoal priority to 3 (highest)? → Agent pursues goal more aggressively
- Make all priorities equal? → Pure weighted voting without priority bias

### Experiment 2: Tune Learning Parameters

**Modify Q-learning hyperparameters:**

```python
class BehaviorCoordinator:
    def __init__(self, behaviors):
        # ...
        self.learning_rate = 0.1      # Try 0.01 (slow) or 0.5 (fast)
        self.discount_factor = 0.9    # Try 0.5 (short-term) or 0.99 (long-term)
        self.epsilon = 0.2            # Try 0.5 (more exploration) or 0.05 (more exploitation)
```

**Observe:**
- How does higher learning_rate affect convergence speed?
- How does discount_factor affect strategy preference (immediate vs. long-term rewards)?
- How does epsilon affect exploration behavior?

### Experiment 3: Add a New Behavior

**Create a "StayOnPath" behavior** that prefers certain areas:

```python
class StayOnPath(Behavior):
    """Prefer staying in open areas (high sensor readings)."""

    def __init__(self):
        super().__init__(name="StayOnPath", priority=1)

    def compute_action(self, agent_state, sensor_data, environment):
        # Find direction with most open space
        max_distance = max(sensor_data.values())
        best_directions = [d for d, dist in sensor_data.items() if dist == max_distance]

        if not best_directions:
            return (0, 0, 0.0)

        # Convert to vector
        direction_vectors = {
            'N': (0, -1), 'NE': (1, -1), 'E': (1, 0), 'SE': (1, 1),
            'S': (0, 1), 'SW': (-1, 1), 'W': (-1, 0), 'NW': (-1, -1)
        }

        chosen = random.choice(best_directions)
        dx, dy = direction_vectors[chosen]

        return (dx, dy, 0.4)  # Moderate strength
```

**Add it to the agent:**

```python
class BehaviorBasedAgent:
    def __init__(self, environment, start_position=(0, 0)):
        # ...
        self.behaviors = [
            AvoidObstacle(),
            SeekGoal(),
            Wander(),
            StayOnPath()  # Add new behavior
        ]
```

**Question:** How does this new behavior affect navigation efficiency?

### Experiment 4: Visualize Learning Progress

**Track success rate over multiple runs:**

Add this after the main simulation code:

```python
def test_learning(num_trials=10):
    """Run multiple trials to see learning improvement."""
    success_rates = []
    env = Environment(width=20, height=20)
    agent = BehaviorBasedAgent(env, start_position=(1, 1))

    for trial in range(num_trials):
        # Reset agent position
        agent.position = (1, 1)
        agent.reached_goal = False
        agent.step_count = 0

        # Run trial
        for step in range(100):
            agent.step()
            if agent.reached_goal:
                break

        success = agent.reached_goal
        success_rates.append(1 if success else 0)

        print(f"Trial {trial+1}: {'Success' if success else 'Failed'} "
              f"({agent.step_count} steps)")

    print(f"\nSuccess rate: {sum(success_rates)/num_trials*100:.1f}%")
    print(f"Average steps when successful: "
          f"{sum(s.step_count for s in [agent] if agent.reached_goal)/max(1, sum(success_rates)):.1f}")

# Run it
test_learning(num_trials=20)
```

**Observe:** Does success rate improve as the agent learns?

## Troubleshooting

### Issue: Agent gets stuck in corners

**Cause:** Obstacle avoidance may create local minima where all directions have obstacles.

**Solution:** Increase Wander behavior strength or add a "stuck detection" behavior that activates when the agent hasn't moved for several steps.

### Issue: Agent doesn't reach goal within 100 steps

**Cause:** Environment may have difficult obstacle configuration, or SeekGoal behavior is too weak.

**Solutions:**
1. Increase max_steps to 200
2. Increase SeekGoal priority or weight
3. Reduce environment obstacle density: `num_obstacles = int(self.width * self.height * 0.05)` (5% instead of 10%)

### Issue: Learning doesn't seem to improve behavior

**Cause:** Q-learning may need more trials, or epsilon (exploration) is too high.

**Solutions:**
1. Run more trials (20-50 instead of just a few)
2. Decrease epsilon to 0.1 for more exploitation
3. Increase learning_rate to 0.3 for faster learning
4. Check that rewards are properly differentiated (goal reached should give much higher reward than incremental progress)

### Issue: Python version compatibility

**Error:** `SyntaxError` or `TypeError`

**Solution:** Ensure you're using Python 3.7+. Check version:

```bash
python --version
```

If you have multiple Python versions, try:

```bash
python3 lesson-03.03-behavior-agent.py
```

## Connecting to Real Physical Agents

This simulation demonstrates principles that transfer directly to real robots:

### Environment → Real Sensors

**Simulation:**
```python
sensor_data = environment.get_sensor_readings(x, y)
```

**Real robot:**
```python
sensor_data = {
    'N': ultrasonic_sensor_front.read_distance(),
    'NE': lidar.read_angle(45),
    # ... other sensors
}
```

### Behaviors → Real Actuators

**Simulation:**
```python
self.position = (new_x, new_y)  # Teleport to new position
```

**Real robot:**
```python
motor_left.set_speed(base_speed + turn_adjustment)
motor_right.set_speed(base_speed - turn_adjustment)
```

### Coordination → Real-Time Execution

**Simulation:** Discrete steps

**Real robot:** Continuous control loop running at fixed frequency (e.g., 10Hz, 50Hz)

```python
import time

CONTROL_FREQUENCY = 10  # Hz
PERIOD = 1.0 / CONTROL_FREQUENCY

while not goal_reached:
    start_time = time.time()

    # PDA loop
    sensor_data = robot.sense()
    action = robot.decide(sensor_data)
    robot.act(action)

    # Maintain fixed frequency
    elapsed = time.time() - start_time
    time.sleep(max(0, PERIOD - elapsed))
```

### Key Differences

| Aspect | Simulation | Real Robot |
|--------|-----------|------------|
| **Sensors** | Perfect, discrete | Noisy, continuous |
| **Actions** | Deterministic | Uncertain, affected by dynamics |
| **State** | Fully known | Partially observable |
| **Time** | Discrete steps | Continuous time |
| **Safety** | Consequence-free failures | Hardware damage, safety risks |

**Bridge the gap:**
1. Add noise to simulated sensors: `reading + random.gauss(0, noise_level)`
2. Add action uncertainty: executed action differs slightly from commanded
3. Use continuous state representation (floats instead of grid cells)
4. Add physics simulation (velocity, acceleration, momentum)

## What's Next: Your Physical AI Journey

**Congratulations!** You've completed all three chapters of the Physical AI book. Let's reflect on your journey:

### What You've Learned

**Chapter 1: Foundations**
- The embodied nature of Physical AI
- The Perception-Decision-Action loop
- How physical grounding differs from pure software AI

**Chapter 2: Perception and Action**
- Sensor types and their applications (proximity, vision, IMU, touch, environmental)
- Actuator mechanisms (DC motors, servos, stepper motors, pneumatics)
- Sensor fusion and multi-modal perception

**Chapter 3: Control and Learning**
- Reactive, deliberative, and hybrid control architectures
- Reinforcement learning, imitation learning, and online adaptation
- Behavior-based systems and coordination (this lesson!)

### From Simulation to Reality

You're now equipped to build real Physical AI systems. Here's your roadmap:

**1. Hardware Projects (Start Simple)**

**Project: Line-Following Robot**
- Sensors: IR reflectance sensors
- Actuators: Two DC motors with motor driver
- Control: Reactive (sensor → motor mapping)
- Platform: Arduino, Raspberry Pi

**Project: Obstacle-Avoiding Rover**
- Sensors: Ultrasonic sensors (like your simulation!)
- Actuators: DC motors with encoders
- Control: Subsumption architecture (avoid → explore)
- Platform: Raspberry Pi with camera (optional)

**Project: Arm Manipulator**
- Sensors: Joint encoders, camera for object detection
- Actuators: Servo motors
- Control: Inverse kinematics + visual servoing
- Platform: ROS (Robot Operating System)

**2. Learning Projects**

**Project: Maze Solver with Q-Learning**
- Extend your simulation to learn optimal paths
- Transfer learned policy to a physical robot
- Compare sim-to-real performance

**Project: Imitation Learning for Pick-and-Place**
- Collect demonstrations using teleoperation
- Train policy with behavioral cloning
- Deploy on robot arm

**3. Advanced Challenges**

**Project: Multi-Agent Coordination**
- Multiple agents with behaviors
- Coordination protocols
- Swarm robotics

**Project: Vision-Based Navigation**
- Camera as primary sensor
- Deep learning for perception
- End-to-end learning (image → action)

### Recommended Resources

**Online Courses:**
- **CS188 (Berkeley)**: Artificial Intelligence—covers planning, learning, and control
- **ROS Tutorials**: Learn the standard robotics middleware
- **Coursera: Modern Robotics**: Kinematics, dynamics, and control

**Hardware Platforms:**
- **Beginner**: Arduino starter kits, BBC micro:bit
- **Intermediate**: Raspberry Pi robots, TurtleBot
- **Advanced**: ROS2-compatible robots, custom builds

**Simulation Tools:**
- **Gazebo**: 3D robot simulator (integrates with ROS)
- **PyBullet**: Python physics simulation
- **MuJoCo**: Fast physics for RL research
- **Webots**: User-friendly robot simulator

**Communities:**
- **ROS Discourse**: robotics questions and discussions
- **r/robotics**: Reddit community
- **Robotics Stack Exchange**: Q&A for specific problems
- **DIY Robotics Forums**: Hardware and project help

**Books:**
- *Probabilistic Robotics* (Thrun, Burgard, Fox): The robotics bible
- *Introduction to Autonomous Mobile Robots* (Siegwart, Nourbakhsh): Comprehensive foundation
- *Reinforcement Learning: An Introduction* (Sutton, Barto): RL fundamentals

### Final Thoughts

Physical AI is where artificial intelligence meets the real world—and it's one of the most exciting frontiers in technology. The principles you've learned apply to:

- **Autonomous vehicles** navigating cities
- **Warehouse robots** optimizing logistics
- **Surgical robots** assisting in operations
- **Drones** delivering packages and inspecting infrastructure
- **Humanoid robots** assisting in homes and hospitals
- **Agricultural robots** tending crops
- **Space rovers** exploring other planets

The field is rapidly evolving, and there's room for newcomers to make meaningful contributions. Start with small projects, iterate quickly, and don't be discouraged by initial failures—every roboticist has seen their creation drive into a wall or fall over unexpectedly!

**Remember the core loop:** Sense → Decide → Act. Master this cycle, and you can build intelligence that operates in the physical world.

**Now go build something amazing.**

</div>

## Summary

In this final hands-on lesson, you learned:

1. **Complete behavior-based agent implementation** with Environment, Behaviors (AvoidObstacle, SeekGoal, Wander), BehaviorCoordinator, and Agent classes, demonstrating the full Perception-Decision-Action loop in practice.

2. **Behavior coordination mechanisms** using weighted voting where behaviors produce vectors with strength values, and a coordinator combines them based on priority, weights, and learned strategy preferences.

3. **Q-learning integration** for adaptive behavior selection, where the coordinator learns which coordination strategies (conservative, balanced, aggressive) work best in different situations (obstacle nearby vs. clear path, near goal vs. far).

4. **Simulation-to-reality principles** including how simulated sensors/actuators map to real hardware, the importance of adding noise and uncertainty, and the transition from discrete steps to continuous control loops.

5. **Experimentation and debugging** skills through modifying priorities, tuning learning parameters, adding new behaviors, and visualizing learning progress across multiple trials.

6. **Your path forward** in Physical AI, from simple hardware projects (line-followers, obstacle-avoiders) to advanced learning systems (imitation learning, vision-based navigation), using platforms like Arduino, Raspberry Pi, and ROS.

**This concludes your journey through the Physical AI book. You now have the foundational knowledge to build intelligent systems that perceive, decide, and act in the real world. The rest is up to you—go create!**

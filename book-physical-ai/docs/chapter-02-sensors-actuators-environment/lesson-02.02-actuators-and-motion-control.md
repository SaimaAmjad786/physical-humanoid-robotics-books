---
title: "Lesson 2.2: Actuators and Motion Control"
description: "Learn how robots transform decisions into physical actions through actuators including motors, servos, and grippers"
keywords: ["actuators", "motors", "servos", "grippers", "motion control", "robotics", "end-effector"]
chapter: 2
lesson: 2
difficulty: beginner
estimated_time: "45 minutes"
prerequisites:
  - "Lesson 2.1: Sensors in Physical AI"
  - "Understanding of the perception-decision-action loop"
learning_objectives:
  - "Understand how actuators enable the action phase of Physical AI"
  - "Distinguish between different actuator types and their applications"
  - "Learn control principles that translate decisions into precise movements"
---

# Lesson 2.2: Actuators and Motion Control

## Prerequisites

Before starting this lesson, you should:
- Have completed Lesson 2.1 on sensors
- Understand the perception-decision-action loop
- Be familiar with how robots perceive their environment

## What You'll Learn

By the end of this lesson, you will be able to:

1. Explain the role of actuators in the action phase of Physical AI systems
2. Compare different actuator types (DC motors, servo motors, stepper motors, grippers) and select appropriate actuators for specific tasks
3. Understand fundamental motion control principles including feedback loops and precision control
4. Recognize how sensors and actuators work together in closed-loop control systems

## Introduction

In Lesson 2.1, we explored how robots perceive their environment through sensors. They can measure distances with ultrasonic sensors, see with cameras, track orientation with IMUs, and feel contact with touch sensors. But perception alone doesn't accomplish tasks—a robot that can see an object but cannot reach for it is functionally limited.

This is where **actuators** enter the picture. If sensors are a robot's senses, actuators are its muscles. They transform electrical signals into physical motion, enabling robots to move, manipulate objects, and interact with their environment.

Consider a robotic arm in a manufacturing facility: cameras identify the part that needs assembly, AI algorithms decide the optimal grip position and approach angle, but it's the actuators—motors in each joint and the gripper mechanism—that actually move the arm into position and secure the part. Without actuators, all that perception and decision-making remains abstract, unable to produce real-world results.

In this lesson, we'll explore the main actuator types used in Physical AI systems, understand how they work, and learn the control principles that translate high-level decisions ("pick up that object") into precise physical movements.

## The Action Phase: From Decisions to Movement

Let's revisit our perception-decision-action loop:

1. **Perception**: Sensors gather environmental data
2. **Decision**: AI processes data and determines actions
3. **Action**: **Actuators execute the decision**

The action phase is where intentions become reality. When a self-driving car's AI decides to "turn left 15 degrees," that decision must be translated into:
- Calculating the required steering wheel angle
- Sending electrical signals to the steering motor
- Continuously monitoring the actual steering angle (using sensors!)
- Adjusting motor power until the target angle is achieved

This process—deciding on an action, executing it with actuators, monitoring progress with sensors, and making corrections—is called **closed-loop control** or **feedback control**, and it's fundamental to reliable robotic systems.

## DC Motors: Continuous Rotation and Drive Systems

### How DC Motors Work

**DC (Direct Current) motors** are the workhorses of robotics. They convert electrical energy into rotational motion through electromagnetic forces. When current flows through a coil inside a magnetic field, the coil experiences a force that causes it to spin.

The basic principle is simple: apply voltage, the motor spins. The **speed** of rotation is roughly proportional to the applied voltage, and the **torque** (rotational force) is roughly proportional to the current.

**Key characteristics**:
- **Speed**: Typically hundreds to thousands of RPM (rotations per minute)
- **Torque**: Available in wide ranges depending on motor size and design
- **Control**: Speed controlled by voltage; direction controlled by polarity

### Types of DC Motors in Robotics

1. **Brushed DC Motors**: Simple, inexpensive, easy to control. Used in hobby robots, toys, and applications where cost is critical. The "brushes" are physical contacts that wear out over time.

2. **Brushless DC Motors (BLDC)**: More efficient, longer lifespan, higher performance. Used in drones, electric vehicles, and professional robotics. Require more complex control electronics.

### Applications and Use Cases

DC motors excel in applications requiring continuous rotation:

1. **Wheel Drive Systems**: Mobile robots use DC motors coupled to wheels for movement. A differential drive robot (like a vacuum cleaner) uses two independently controlled motors—spinning them at the same speed moves forward, at different speeds turns the robot.

2. **Propellers and Fans**: Drones use brushless DC motors to spin propellers. Varying the speed of each motor controls the drone's movement—speed up the right motors to tilt left, speed up all motors to ascend.

3. **Conveyor Belts**: Industrial robots use DC motors to drive conveyor systems that move parts through assembly lines.

4. **Pumps and Compressors**: Robots that handle fluids or pneumatics use DC motors to drive pumps.

### Control Challenges and Solutions

The main challenge with basic DC motors is **lack of position control**. When you apply voltage, the motor spins, but you don't inherently know:
- How fast it's actually spinning (speed varies with load)
- How far it has rotated
- Its current position

**Solution 1: Encoders**
Attach an **encoder** (a sensor that counts rotations) to the motor shaft. Now the control system can:
- Measure actual speed and adjust voltage to maintain target speed
- Count rotations to know total distance traveled
- Implement closed-loop speed control

**Example**: A mobile robot needs to move exactly 2 meters forward. With encoders on wheel motors, the controller:
1. Calculates required wheel rotations (distance ÷ wheel circumference)
2. Commands motors forward
3. Continuously counts encoder pulses
4. Stops motors when the target rotation count is reached

**Solution 2: Gearboxes**
DC motors typically spin too fast and with too little torque for direct use. A **gearbox** reduces speed while increasing torque. A 100:1 gearbox reduces motor speed by 100× and increases torque by approximately 100×.

## Servo Motors: Precise Position Control

### How Servo Motors Work

A **servo motor** is actually a complete control system in a compact package, consisting of:
1. A DC motor (usually brushed)
2. A gearbox to reduce speed and increase torque
3. A position sensor (potentiometer) that measures the output shaft angle
4. Control electronics that maintain the desired position

You don't send voltage to a servo motor; instead, you send a **position command** (typically via PWM signal), and the internal controller automatically adjusts motor power to reach and maintain that position.

**Key characteristics**:
- **Position control**: Extremely precise angular positioning
- **Limited range**: Typically 0-180 degrees or 0-360 degrees (continuous rotation servos)
- **Holding torque**: Actively maintains position even under external forces
- **Feedback**: Built-in closed-loop control system

### Standard vs. Continuous Rotation Servos

1. **Standard Servos**: Rotate to a commanded angle (0-180° typical) and hold position. Used when precise angular control is needed.

2. **Continuous Rotation Servos**: Modified to spin continuously like a DC motor, but with better speed control. The position command becomes a speed command.

### Applications and Use Cases

Servo motors are ideal for applications requiring precise positioning:

1. **Robot Joints**: Humanoid robots, robot arms, and quadruped robots use servos in their joints. Each joint is a servo positioned at a specific angle to achieve the desired overall pose.

   **Example**: A robotic arm picking up an object requires:
   - Shoulder servo at 45° to raise the arm
   - Elbow servo at 90° to position the gripper
   - Wrist servo at 30° to orient the gripper correctly

2. **Camera Gimbals**: Pan-tilt mechanisms use servos to point cameras in precise directions. Security cameras and drone gimbals stabilize footage by adjusting servo angles hundreds of times per second.

3. **Control Surfaces**: Small robots with steering mechanisms (like RC cars) use servos to control steering angle precisely.

4. **Animatronics**: Robots designed to mimic living creatures use many servos to create realistic movements (facial expressions, body language).

### Position Control Through Feedback

Servos demonstrate **closed-loop control** perfectly:

1. **Command**: You send "Go to 90 degrees"
2. **Action**: Internal controller applies power to the motor
3. **Sensing**: Potentiometer measures actual shaft position
4. **Comparison**: Controller compares actual position to target position
5. **Adjustment**: If position error exists, adjust motor power
6. **Repeat**: Continue adjusting until position error is near zero

This feedback loop happens hundreds of times per second, providing precise, stable position control.

**Example**: A servo commanded to 90° starts at 0°:
- Error = 90° (target) - 0° (actual) = 90° → Apply maximum power
- After 100ms: Error = 90° - 70° = 20° → Reduce power
- After 150ms: Error = 90° - 88° = 2° → Minimal power
- After 200ms: Error = 90° - 90° = 0° → Just enough power to hold position

## Stepper Motors: Precise Positioning Without Feedback

### How Stepper Motors Work

**Stepper motors** move in discrete steps rather than continuous rotation. A typical stepper might have 200 steps per revolution, meaning each step rotates the shaft exactly 1.8 degrees (360° ÷ 200 = 1.8°).

Unlike DC motors or servos, steppers are controlled by sending a sequence of electrical pulses. Each pulse advances the motor by one step. Send 200 pulses, the motor completes one full rotation. Send 100 pulses, it rotates exactly 180 degrees.

**Key characteristics**:
- **Open-loop control**: No position sensor needed (assumes each pulse = one step)
- **Holding torque**: When powered but not stepping, holds position firmly
- **Precise positioning**: Excellent repeatability
- **No feedback**: Doesn't inherently know if it missed steps (needs encoders for critical applications)

### Applications and Use Cases

Stepper motors are common in applications requiring precise, repeatable positioning:

1. **3D Printers and CNC Machines**: Stepper motors drive the X, Y, and Z axes with high precision. To move the print head 10mm, the controller calculates required steps (based on belt/screw pitch) and sends exactly that many pulses.

2. **Camera Sliders and Turntables**: Photography robots use steppers to create smooth, programmable camera movements for time-lapse or product photography.

3. **Robotic Arms (Educational)**: Lower-cost educational robot arms often use steppers instead of servos for joint control.

4. **Automated Lab Equipment**: Pipetting robots, sample handlers, and other lab automation use steppers for precise, repeatable movements.

### Stepper vs. Servo: When to Use Each

| Feature | Stepper Motor | Servo Motor |
|---------|--------------|-------------|
| Position control | Open-loop (counts pulses) | Closed-loop (measures position) |
| Holding torque | Excellent when stationary | Good, actively corrects |
| Speed | Lower max speed | Higher speed capability |
| Cost | Moderate | Higher (built-in controller) |
| Complexity | Simple control (pulse sequence) | Simple interface (position command) |
| Feedback | None (unless encoder added) | Built-in position feedback |
| Use case | Precise positioning, moderate speed | Dynamic positioning, holding against forces |

**Rule of thumb**: Use steppers when you need precise positioning along a predictable path (3D printer). Use servos when external forces might disrupt position or rapid position changes are needed (robot arm joint).

## Grippers and End-Effectors: Robot Manipulation

### What is an End-Effector?

An **end-effector** is the device at the end of a robotic arm that interacts with the environment. While the arm positions the end-effector in space, the end-effector performs the actual task. Think of it as the robot's "hand"—but unlike human hands that are general-purpose, robot end-effectors are often task-specific.

**Common end-effector types**:
- **Grippers**: For grasping objects
- **Welding torches**: For joining metal
- **Paint sprayers**: For coating surfaces
- **Drills and mills**: For material removal
- **Suction cups**: For picking up flat objects
- **Magnetic grippers**: For ferrous materials

### Gripper Types and Mechanisms

Grippers are the most common end-effectors, and they come in various configurations:

#### 1. Parallel Jaw Grippers

Two fingers move in parallel, opening and closing like a vice. These are the most common industrial grippers.

**Actuation methods**:
- **Pneumatic**: Compressed air drives the gripper—simple, powerful, fast
- **Electric**: Motor-driven (often servo or stepper)—more precise force control
- **Hydraulic**: For very high force applications

**Applications**: Assembly tasks, pick-and-place operations, machine tending

**Example**: An assembly robot picks up a cylindrical part. The parallel jaws close around the cylinder from opposite sides, with force sensors ensuring secure grip without crushing the part.

#### 2. Angular/Pivot Grippers

Fingers pivot around a hinge point, similar to how your fingers curl. Better for grasping objects from above.

**Applications**: Picking up objects of varying sizes, reaching into confined spaces

#### 3. Three-Finger Grippers

Three fingers arranged at 120° intervals provide more stable grasping of irregular shapes and better centering of objects.

**Applications**: Handling complex geometries, self-centering grips

#### 4. Soft/Compliant Grippers

Made from flexible materials (silicone, rubber), these grippers conform to object shape, ideal for delicate or irregular items.

**Applications**: Food handling, agricultural harvesting (fruits, vegetables), packaging

**Example**: A robot harvesting strawberries uses soft gripper fingers that gently envelope the berry without bruising it, adapting to each berry's unique shape.

#### 5. Vacuum Grippers

Use suction to lift objects with flat surfaces. No mechanical fingers needed.

**Applications**: Picking up boxes, glass sheets, semiconductor wafers

### Gripper Control and Sensing

Modern grippers incorporate sensors for intelligent manipulation:

1. **Force/Torque Sensors**: Measure grip force to prevent crushing or dropping objects. The control system adjusts actuator power to maintain target force.

2. **Tactile Sensors**: Detect contact location and pressure distribution, enabling:
   - Slip detection: If the object starts sliding, increase grip force
   - Object recognition: Identify objects by their tactile signature
   - Safe human interaction: Detect and respond to human touch

3. **Proximity Sensors**: Detect when fingers are approaching the object, allowing pre-shaping of the grip.

### Degrees of Freedom (DOF)

The capability of a robotic system depends largely on its **degrees of freedom**—the number of independent ways it can move.

**Examples**:
- **2-DOF gripper on a fixed mount**: Can open/close and rotate—limited capability
- **6-DOF robotic arm + 1-DOF gripper = 7 DOF total**: Can position and orient the gripper anywhere in its workspace, then grasp—high capability
- **Human arm (shoulder to fingertips)**: Approximately 27 DOF—extreme capability and dexterity

Each DOF requires at least one actuator, so higher DOF means more complex control but greater flexibility.

## Motion Control Principles: Making Actuators Work Together

### Open-Loop vs. Closed-Loop Control

**Open-Loop Control**: Send a command and hope for the best.
- Example: Apply voltage to a DC motor and assume it reaches the desired speed
- Problem: Doesn't account for variations (battery voltage drops, mechanical load increases)
- Use case: Simple, non-critical applications where approximate results are acceptable

**Closed-Loop Control**: Continuously measure results and adjust actions.
- Example: Command a motor to 1000 RPM, measure actual speed with encoder, adjust voltage if actual ≠ target
- Advantage: Automatically compensates for disturbances and variations
- Use case: Any application requiring precision or reliability

The difference is **feedback**—closed-loop systems use sensors to verify that actions achieve intended results.

### PID Control: The Workhorse of Motion Control

**PID (Proportional-Integral-Derivative)** controllers are the most common algorithm for closed-loop control. They adjust actuator commands based on three factors:

1. **Proportional (P)**: Error magnitude—how far are we from the target?
   - Large error → large correction
   - Small error → small correction

2. **Integral (I)**: Accumulated error over time—have we been consistently off-target?
   - Corrects for persistent biases (like gravity constantly pulling a robot arm down)

3. **Derivative (D)**: Rate of error change—are we approaching the target quickly or slowly?
   - Prevents overshooting by reducing action as we near the target
   - Provides damping

**Example**: A drone maintaining altitude at 10 meters:
- **P**: Currently at 9.5m → error = 0.5m → increase thrust proportionally
- **I**: Has been below 10m for past 2 seconds → accumulated error → increase thrust further
- **D**: Altitude is rising at 0.3 m/s → approaching target → reduce thrust slightly to avoid overshoot

PID controllers are everywhere: cruise control in cars, temperature control in ovens, and position control in robot joints.

### Coordinated Multi-Actuator Control

Real robots have many actuators that must work together. A 6-axis robot arm has 6 motors, and moving the end-effector from point A to point B requires coordinating all 6 simultaneously.

**Inverse Kinematics**: The mathematical process of calculating required joint angles to achieve a desired end-effector position and orientation.

**Example**: To move a robotic arm's gripper to coordinates (X=0.5m, Y=0.3m, Z=0.2m):
1. Inverse kinematics calculates: shoulder=35°, elbow=65°, wrist=45°, etc.
2. Each joint's servo receives its target angle
3. All servos move simultaneously
4. The gripper reaches the target position

**Trajectory Planning**: Defining the path an actuator follows, not just the destination.
- **Point-to-point**: Fastest path from A to B (may be jerky)
- **Linear interpolation**: Straight-line path in space (smoother)
- **Spline paths**: Smooth, curved paths through multiple waypoints

### Safety and Limits

Motion control systems must incorporate safety mechanisms:

1. **Software Limits**: Define maximum/minimum positions, speeds, and forces
2. **Hardware Limits**: Physical switches that cut power if limits are exceeded
3. **Emergency Stop**: Immediate halt of all motion (legally required in industrial settings)
4. **Collision Detection**: Monitor actuator current—sudden increases indicate collisions
5. **Force Limiting**: Collaborative robots limit force to safe levels for human interaction

## Integrating Sensors and Actuators: Closed-Loop Control in Action

The true power of Physical AI emerges when sensors and actuators work together in feedback loops. Let's explore a complete example:

### Example: Robotic Arm Picking Up an Object

**Phase 1: Visual Servoing (Camera + Motor Control)**
1. Camera identifies object location
2. AI calculates required arm movement
3. Motors move arm toward object
4. Camera continuously updates object position (target might move!)
5. Control system adjusts trajectory in real-time

**Phase 2: Approach (Proximity Sensor + Gripper)**
1. Proximity sensor detects object is within reach
2. Gripper pre-opens to appropriate width (based on object size from vision)
3. Arm slows approach speed for precise positioning

**Phase 3: Grasping (Touch Sensors + Force Control)**
1. Gripper closes until touch sensors detect contact
2. Force sensors measure grip pressure
3. Control system adjusts to target force (secure but not crushing)
4. Continuous monitoring detects slipping and increases force if needed

**Phase 4: Lifting (IMU + Current Sensing)**
1. Arm motors lift the object
2. IMU detects unexpected tilt (load is off-center)
3. Control system adjusts joint torques to maintain level orientation
4. Motor current sensing confirms object was successfully lifted (current increases due to added load)

**Phase 5: Transport and Release**
1. Inverse kinematics calculates trajectory to destination
2. Multiple joint servos coordinate to follow smooth path
3. At destination, force sensors confirm stable placement
4. Gripper opens, touch sensors confirm object release

This single task demonstrates the seamless integration of multiple sensor types (camera, proximity, touch, force, IMU, current) and actuator types (motors, servos, gripper), all coordinated through feedback control loops.

## Self-Assessment

Test your understanding with these questions:

<details>
<summary>Question 1: A mobile robot uses DC motors with encoders for its wheels. Explain why encoders are necessary and what would happen without them.</summary>

**Answer**: Encoders are necessary because basic DC motors provide continuous rotation but no inherent position or speed feedback. Without encoders, the robot would face several problems:

1. **Uncertain Distance**: The robot couldn't measure how far it has traveled. Commanding "move 2 meters" would be impossible—the robot could only run motors for a guessed duration.

2. **Unequal Wheel Speeds**: Manufacturing variations and load differences mean two motors at the same voltage spin at slightly different speeds. Without encoders, the robot would drift in curved paths even when commanded straight.

3. **Speed Variations**: Motor speed varies with battery voltage and mechanical load. A motor running on fresh batteries at 12V spins faster than the same motor on depleted batteries at 10V. Without encoders measuring actual speed, the robot's movement would be inconsistent.

4. **No Closed-Loop Control**: The robot operates open-loop, unable to correct errors or adapt to changing conditions (uphill slopes, rough terrain).

**With encoders**, the control system can:
- Count wheel rotations to measure distance accurately
- Continuously adjust motor voltages to maintain equal wheel speeds (for straight movement)
- Implement closed-loop speed control to maintain consistent velocity regardless of battery state
- Detect slippage or mechanical problems (expected vs. actual encoder readings differ)
</details>

<details>
<summary>Question 2: Why might you choose a servo motor instead of a stepper motor for a robotic arm joint that needs to hold a heavy object?</summary>

**Answer**: A **servo motor** is better suited for this application for several reasons:

1. **Closed-Loop Feedback**: Servos continuously measure their actual position and actively correct any deviations. If the heavy object causes the arm to droop, the servo's internal controller detects the position error and applies more power to return to the target angle.

2. **Dynamic Load Response**: The servo automatically adjusts torque based on the load. When holding a heavy object, it provides more holding torque. Steppers provide constant current regardless of load (though "holding torque" is good, they don't adaptively adjust).

3. **Energy Efficiency with Variable Loads**: Servos apply only the current needed to maintain position against the actual load. Steppers typically draw full current continuously when holding position, even if little torque is needed, wasting energy.

4. **Guaranteed Position**: If an external force (the heavy object's weight) overcomes a stepper's holding torque, the stepper will lose steps, and the controller won't know—it still thinks it's at the commanded position. A servo's position sensor detects this immediately and corrects.

5. **Safety**: If the load is too heavy and the servo cannot hold position, the position feedback will indicate the error, allowing the system to detect the problem and take action (alarm, safe shutdown). A stepper might silently fail (lose steps) without detection.

**When steppers are appropriate**: For applications with predictable, constant loads and defined motion paths (like a 3D printer axis), steppers work well and are often more cost-effective. But for dynamic, variable loads like holding objects, servos' closed-loop control provides essential reliability and safety.
</details>

<details>
<summary>Question 3: A robotic gripper has both force sensors and touch sensors. Explain how each sensor type contributes differently to successful grasping, and give a scenario where having both is essential.</summary>

**Answer**: **Touch sensors** and **force sensors** provide complementary information:

**Touch Sensors** (binary or simple pressure):
- Detect **when** contact occurs (gripper has reached the object)
- Determine **where** contact occurs on the gripper surface (for sensor arrays)
- Confirm contact before applying significant force
- Detect unexpected contacts (collision avoidance)

**Force Sensors** (measure force magnitude):
- Measure **how much** force is being applied
- Enable precise force control (grip firmly but don't crush)
- Detect object slipping (force decreases as object moves)
- Measure object weight (force required to lift)

**Scenario where both are essential**: Picking up a delicate item of unknown weight, such as an egg.

1. **Approach Phase**: Gripper moves toward egg
   - **Touch sensor**: Detects initial contact with egg shell
   - **System response**: Immediately stops gripper closing to prevent crushing

2. **Grasping Phase**: Gripper applies minimal force
   - **Force sensor**: Measures current grip force (starts at ~0.5N)
   - **System response**: Slowly increases force until secure

3. **Lift Test Phase**: Arm begins to lift
   - **Force sensor**: Monitors grip force—if egg starts slipping, force decreases
   - **Touch sensor**: If slipping is severe, touch sensors lose contact
   - **System response**: If force decreases but touch sensors still detect contact, slightly increase grip force. If touch sensors lose contact, egg is slipping—grip harder or abandon grasp.

4. **Hold Phase**: Transporting the egg
   - **Force sensor**: Continuously monitors grip force at ~1.5N (enough to hold but not crush)
   - **Touch sensor**: Confirms continuous contact

Without **touch sensors**, the gripper might apply force before actually contacting the egg, or might not detect position of contact for proper grip. Without **force sensors**, the gripper couldn't measure whether it's applying egg-crushing force (3N+) or insufficient holding force (0.5N, leading to slipping). Both sensors working together enable reliable, gentle manipulation.
</details>

## What's Next

You now understand both sides of the Physical AI equation: sensors for perception (Lesson 2.1) and actuators for action (this lesson). The decision-making that connects perception to action often happens in software, but hands-on experience is invaluable.

In **Lesson 2.3: Hands-On Virtual Robot Interaction**, you'll:
- Build a simulated robot environment in pure Python
- Program virtual sensors and actuators
- Implement closed-loop control to complete tasks
- Experience the perception-decision-action loop in practice

This hands-on exercise will cement your understanding of how sensors, decision algorithms, and actuators work together to create intelligent physical systems.

## Summary

In this lesson, you've learned how actuators enable Physical AI systems to act on their decisions:

1. **Actuators transform decisions into physical actions**: They are the "muscles" of robots, converting electrical signals into motion, completing the perception-decision-action loop.

2. **Different actuator types serve different purposes**:
   - **DC motors**: Continuous rotation for wheels, propellers, and drive systems; require encoders for position feedback
   - **Servo motors**: Precise angular positioning with built-in feedback control for robot joints and pan-tilt mechanisms
   - **Stepper motors**: Discrete-step positioning without feedback sensors for 3D printers, CNC machines, and predictable motion paths
   - **Grippers and end-effectors**: Task-specific tools for manipulation, available in many configurations (parallel jaw, soft grippers, vacuum, etc.)

3. **Closed-loop control creates reliable systems**: Feedback from sensors allows actuators to continuously verify and correct their actions. PID controllers are the standard algorithm for implementing smooth, stable control.

4. **Sensors and actuators work together**: Real Physical AI systems integrate multiple sensors and actuators in coordinated feedback loops. A robot arm picking up an object uses cameras, proximity sensors, touch sensors, force sensors, and IMUs to guide motors, servos, and grippers through a complex, adaptive task.

With your understanding of sensors (perception) and actuators (action), you're ready to bring these concepts to life through hands-on programming in the next lesson.

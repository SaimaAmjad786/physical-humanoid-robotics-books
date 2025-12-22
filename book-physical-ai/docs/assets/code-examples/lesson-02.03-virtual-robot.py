"""
Virtual Robot Simulation for Physical AI Learning
Lesson 2.3: Hands-On Virtual Robot Interaction

This pure-Python simulation demonstrates:
- Multiple sensor types (ultrasonic, camera, IMU, touch)
- Multiple actuator types (motors, servos, gripper)
- Closed-loop control with sensor feedback
- Perception-decision-action loop

No external dependencies required - runs with standard Python 3.6+
"""

import math
import random
from typing import List, Tuple, Dict, Optional


class Vector2D:
    """Simple 2D vector for position and movement."""

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float):
        return Vector2D(self.x * scalar, self.y * scalar)

    def magnitude(self) -> float:
        """Calculate vector length."""
        return math.sqrt(self.x**2 + self.y**2)

    def normalize(self):
        """Return unit vector in same direction."""
        mag = self.magnitude()
        if mag == 0:
            return Vector2D(0, 0)
        return Vector2D(self.x / mag, self.y / mag)

    def __repr__(self):
        return f"({self.x:.2f}, {self.y:.2f})"


class Environment:
    """Virtual environment containing obstacles and objects."""

    def __init__(self, width: float = 10.0, height: float = 10.0):
        self.width = width
        self.height = height
        self.obstacles: List[Dict] = []
        self.objects: List[Dict] = []

        # Add some obstacles (walls)
        self.obstacles.append({"pos": Vector2D(5.0, 3.0), "radius": 0.5, "type": "wall"})
        self.obstacles.append({"pos": Vector2D(7.0, 6.0), "radius": 0.4, "type": "wall"})

        # Add target objects to pick up
        self.objects.append({
            "pos": Vector2D(8.0, 4.0),
            "radius": 0.2,
            "type": "cube",
            "picked": False
        })

    def get_distance_to_obstacle(self, position: Vector2D, angle: float,
                                  max_range: float) -> float:
        """
        Simulate ultrasonic sensor - find distance to nearest obstacle.
        Returns distance in meters, or max_range if no obstacle detected.
        """
        # Cast a ray in the given direction
        direction = Vector2D(math.cos(angle), math.sin(angle))

        min_distance = max_range

        # Check distance to all obstacles
        for obstacle in self.obstacles:
            # Vector from position to obstacle
            to_obstacle = obstacle["pos"] - position

            # Project onto ray direction
            projection = (to_obstacle.x * direction.x +
                         to_obstacle.y * direction.y)

            if projection > 0:  # Obstacle is in front of sensor
                # Find closest point on ray to obstacle center
                closest_point = position + (direction * projection)
                distance_to_center = (obstacle["pos"] - closest_point).magnitude()

                if distance_to_center <= obstacle["radius"]:
                    # Ray intersects obstacle
                    actual_distance = projection - math.sqrt(
                        obstacle["radius"]**2 - distance_to_center**2
                    )
                    min_distance = min(min_distance, max(0, actual_distance))

        # Check environment boundaries
        if direction.x > 0:
            dist_to_right = (self.width - position.x) / direction.x
            min_distance = min(min_distance, dist_to_right)
        elif direction.x < 0:
            dist_to_left = -position.x / direction.x
            min_distance = min(min_distance, dist_to_left)

        if direction.y > 0:
            dist_to_top = (self.height - position.y) / direction.y
            min_distance = min(min_distance, dist_to_top)
        elif direction.y < 0:
            dist_to_bottom = -position.y / direction.y
            min_distance = min(min_distance, dist_to_bottom)

        return min_distance

    def detect_objects_in_view(self, position: Vector2D, angle: float,
                               fov: float, max_range: float) -> List[Dict]:
        """
        Simulate camera - detect objects within field of view.
        Returns list of detected objects with distance and relative angle.
        """
        detected = []

        for obj in self.objects:
            if obj["picked"]:
                continue  # Object already picked up

            to_object = obj["pos"] - position
            distance = to_object.magnitude()

            if distance > max_range:
                continue

            # Calculate angle to object
            object_angle = math.atan2(to_object.y, to_object.x)
            angle_diff = ((object_angle - angle + math.pi) % (2 * math.pi)) - math.pi

            if abs(angle_diff) <= fov / 2:
                detected.append({
                    "type": obj["type"],
                    "distance": distance,
                    "angle": angle_diff,
                    "position": obj["pos"],
                    "reference": obj  # Reference to actual object
                })

        return detected


class UltrasonicSensor:
    """Simulates an ultrasonic distance sensor."""

    def __init__(self, max_range: float = 3.0, mounting_angle: float = 0.0):
        self.max_range = max_range
        self.mounting_angle = mounting_angle  # Relative to robot heading
        self.last_reading = max_range

    def read(self, robot_pos: Vector2D, robot_angle: float,
             environment: Environment) -> float:
        """
        Get distance reading.
        Returns distance in meters, or max_range if no obstacle.
        """
        actual_angle = robot_angle + self.mounting_angle

        # Add small noise to simulate real sensor
        noise = random.gauss(0, 0.02)

        distance = environment.get_distance_to_obstacle(
            robot_pos, actual_angle, self.max_range
        )

        self.last_reading = max(0, distance + noise)
        return self.last_reading


class CameraSensor:
    """Simulates a camera with object detection capabilities."""

    def __init__(self, fov: float = math.pi/3, max_range: float = 5.0):
        self.fov = fov  # Field of view in radians (60 degrees default)
        self.max_range = max_range
        self.detected_objects = []

    def capture(self, robot_pos: Vector2D, robot_angle: float,
                environment: Environment) -> List[Dict]:
        """
        Capture image and detect objects.
        Returns list of detected objects.
        """
        self.detected_objects = environment.detect_objects_in_view(
            robot_pos, robot_angle, self.fov, self.max_range
        )
        return self.detected_objects

    def get_nearest_object(self) -> Optional[Dict]:
        """Get the closest detected object."""
        if not self.detected_objects:
            return None
        return min(self.detected_objects, key=lambda obj: obj["distance"])


class IMU:
    """Simulates an Inertial Measurement Unit (accelerometer + gyroscope)."""

    def __init__(self):
        self.orientation = 0.0  # Current heading in radians
        self.angular_velocity = 0.0  # Rotation rate
        self.acceleration = Vector2D(0, 0)  # Linear acceleration

    def update(self, new_orientation: float, velocity: Vector2D, dt: float = 0.1):
        """Update IMU readings based on robot motion."""
        # Calculate angular velocity
        angle_change = ((new_orientation - self.orientation + math.pi) %
                       (2 * math.pi)) - math.pi
        self.angular_velocity = angle_change / dt

        # Update orientation
        self.orientation = new_orientation

        # Acceleration (simplified - would be more complex in real IMU)
        # Add small noise
        noise_x = random.gauss(0, 0.05)
        noise_y = random.gauss(0, 0.05)
        self.acceleration = Vector2D(
            velocity.x / dt + noise_x,
            velocity.y / dt + noise_y
        )

    def get_heading(self) -> float:
        """Get current heading in degrees."""
        return math.degrees(self.orientation) % 360

    def get_angular_velocity(self) -> float:
        """Get rotation rate in degrees/second."""
        return math.degrees(self.angular_velocity)


class TouchSensor:
    """Simulates a touch/contact sensor."""

    def __init__(self):
        self.is_pressed = False
        self.force = 0.0  # Force in Newtons

    def set_state(self, pressed: bool, force: float = 0.0):
        """Update touch sensor state."""
        self.is_pressed = pressed
        self.force = force if pressed else 0.0

    def is_touching(self) -> bool:
        """Check if sensor detects contact."""
        return self.is_pressed


class Motor:
    """Simulates a DC motor with encoder."""

    def __init__(self, max_speed: float = 1.0):
        self.max_speed = max_speed  # meters per second
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.encoder_count = 0
        self.total_distance = 0.0

    def set_speed(self, speed: float):
        """Set target speed (-1.0 to 1.0, normalized)."""
        self.target_speed = max(-1.0, min(1.0, speed))

    def update(self, dt: float = 0.1):
        """Update motor state (simple acceleration model)."""
        # Gradually approach target speed
        acceleration = (self.target_speed - self.current_speed) * 5.0
        self.current_speed += acceleration * dt

        # Clamp to max speed
        self.current_speed = max(-self.max_speed,
                                 min(self.max_speed, self.current_speed))

        # Update encoder
        distance = self.current_speed * dt
        self.total_distance += abs(distance)
        self.encoder_count += int(abs(distance) * 100)  # 100 counts per meter

        return self.current_speed

    def get_speed(self) -> float:
        """Get current speed in m/s."""
        return self.current_speed

    def reset_encoder(self):
        """Reset encoder count to zero."""
        self.encoder_count = 0
        self.total_distance = 0.0


class ServoMotor:
    """Simulates a servo motor with position control."""

    def __init__(self, min_angle: float = 0, max_angle: float = 180):
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = 90  # Start at middle position
        self.target_angle = 90

    def set_angle(self, angle: float):
        """Set target angle in degrees."""
        self.target_angle = max(self.min_angle, min(self.max_angle, angle))

    def update(self, dt: float = 0.1):
        """Update servo position (moves toward target)."""
        # Move toward target at fixed speed
        max_change = 180 * dt  # 180 degrees per second
        angle_diff = self.target_angle - self.current_angle

        if abs(angle_diff) < max_change:
            self.current_angle = self.target_angle
        else:
            self.current_angle += math.copysign(max_change, angle_diff)

        return self.current_angle

    def get_angle(self) -> float:
        """Get current angle in degrees."""
        return self.current_angle

    def at_target(self) -> bool:
        """Check if servo has reached target position."""
        return abs(self.current_angle - self.target_angle) < 1.0


class Gripper:
    """Simulates a robotic gripper with force sensing."""

    def __init__(self):
        self.servo = ServoMotor(min_angle=0, max_angle=90)  # 0=closed, 90=open
        self.force_sensor = TouchSensor()
        self.holding_object = None

    def open(self):
        """Open gripper."""
        self.servo.set_angle(90)

    def close(self):
        """Close gripper."""
        self.servo.set_angle(0)

    def set_position(self, position: float):
        """Set gripper opening (0.0=closed, 1.0=fully open)."""
        angle = position * 90
        self.servo.set_angle(angle)

    def update(self, dt: float = 0.1):
        """Update gripper state."""
        self.servo.update(dt)

        # Simulate force based on gripper position and held object
        if self.holding_object and self.servo.get_angle() < 30:
            self.force_sensor.set_state(True, force=2.0)  # Gripping with force
        elif self.servo.get_angle() < 10:
            self.force_sensor.set_state(True, force=0.5)  # Touching
        else:
            self.force_sensor.set_state(False)

    def is_closed(self) -> bool:
        """Check if gripper is closed."""
        return self.servo.get_angle() < 20

    def is_gripping(self) -> bool:
        """Check if gripper is holding an object."""
        return self.holding_object is not None

    def get_grip_force(self) -> float:
        """Get current grip force in Newtons."""
        return self.force_sensor.force


class VirtualRobot:
    """
    Complete virtual robot with multiple sensors and actuators.
    Demonstrates the perception-decision-action loop.
    """

    def __init__(self, environment: Environment):
        self.environment = environment

        # Robot state
        self.position = Vector2D(2.0, 2.0)  # Starting position
        self.heading = 0.0  # Heading in radians (0 = East)
        self.velocity = Vector2D(0, 0)

        # Sensors
        self.ultrasonic_front = UltrasonicSensor(max_range=3.0, mounting_angle=0)
        self.ultrasonic_left = UltrasonicSensor(max_range=2.0,
                                               mounting_angle=math.pi/2)
        self.ultrasonic_right = UltrasonicSensor(max_range=2.0,
                                                mounting_angle=-math.pi/2)
        self.camera = CameraSensor(fov=math.pi/3, max_range=5.0)
        self.imu = IMU()

        # Actuators
        self.motor_left = Motor(max_speed=1.0)
        self.motor_right = Motor(max_speed=1.0)
        self.gripper = Gripper()

        # State
        self.time = 0.0
        self.dt = 0.1  # Time step in seconds

    def read_sensors(self) -> Dict:
        """
        PERCEPTION phase: Read all sensors.
        Returns dictionary of sensor readings.
        """
        sensor_data = {
            "ultrasonic": {
                "front": self.ultrasonic_front.read(self.position, self.heading,
                                                    self.environment),
                "left": self.ultrasonic_left.read(self.position, self.heading,
                                                 self.environment),
                "right": self.ultrasonic_right.read(self.position, self.heading,
                                                   self.environment)
            },
            "camera": self.camera.capture(self.position, self.heading,
                                         self.environment),
            "imu": {
                "heading": self.imu.get_heading(),
                "angular_velocity": self.imu.get_angular_velocity()
            },
            "gripper": {
                "force": self.gripper.get_grip_force(),
                "is_gripping": self.gripper.is_gripping()
            },
            "position": self.position,
            "time": self.time
        }
        return sensor_data

    def set_motor_speeds(self, left_speed: float, right_speed: float):
        """
        ACTION phase: Set motor speeds.
        Differential drive: left/right speeds control forward motion and turning.
        """
        self.motor_left.set_speed(left_speed)
        self.motor_right.set_speed(right_speed)

    def move_forward(self, speed: float = 0.5):
        """Move forward at given speed."""
        self.set_motor_speeds(speed, speed)

    def turn_left(self, speed: float = 0.3):
        """Turn left."""
        self.set_motor_speeds(-speed, speed)

    def turn_right(self, speed: float = 0.3):
        """Turn right."""
        self.set_motor_speeds(speed, -speed)

    def stop(self):
        """Stop all motors."""
        self.set_motor_speeds(0, 0)

    def update(self):
        """Update robot physics and actuators."""
        # Update motors
        left_speed = self.motor_left.update(self.dt)
        right_speed = self.motor_right.update(self.dt)

        # Differential drive kinematics
        forward_speed = (left_speed + right_speed) / 2.0
        rotation_speed = (right_speed - left_speed) / 0.5  # 0.5m wheelbase

        # Update heading
        old_heading = self.heading
        self.heading += rotation_speed * self.dt
        self.heading = self.heading % (2 * math.pi)

        # Update position
        old_position = self.position
        movement = Vector2D(
            math.cos(self.heading) * forward_speed * self.dt,
            math.sin(self.heading) * forward_speed * self.dt
        )
        self.position = self.position + movement

        # Keep robot in bounds
        self.position.x = max(0.1, min(self.environment.width - 0.1,
                                       self.position.x))
        self.position.y = max(0.1, min(self.environment.height - 0.1,
                                       self.position.y))

        # Calculate velocity
        self.velocity = (self.position - old_position) * (1.0 / self.dt)

        # Update IMU
        self.imu.update(self.heading, self.velocity, self.dt)

        # Update gripper
        self.gripper.update(self.dt)

        # Check if gripper can pick up objects
        if self.gripper.is_closed() and not self.gripper.is_gripping():
            for obj in self.environment.objects:
                if not obj["picked"]:
                    distance = (obj["pos"] - self.position).magnitude()
                    if distance < 0.3:  # Within reach
                        self.gripper.holding_object = obj
                        obj["picked"] = True
                        break

        # Update time
        self.time += self.dt

    def get_status(self) -> str:
        """Get formatted status string."""
        status = f"\n{'='*60}\n"
        status += f"Time: {self.time:.1f}s | Position: {self.position} | "
        status += f"Heading: {self.imu.get_heading():.1f}°\n"
        status += f"{'='*60}\n"

        status += f"SENSORS:\n"
        sensors = self.read_sensors()
        status += f"  Ultrasonic - Front: {sensors['ultrasonic']['front']:.2f}m, "
        status += f"Left: {sensors['ultrasonic']['left']:.2f}m, "
        status += f"Right: {sensors['ultrasonic']['right']:.2f}m\n"

        if sensors['camera']:
            status += f"  Camera - Detected {len(sensors['camera'])} object(s):\n"
            for obj in sensors['camera']:
                status += f"    - {obj['type']} at {obj['distance']:.2f}m, "
                status += f"angle {math.degrees(obj['angle']):.1f}°\n"
        else:
            status += f"  Camera - No objects detected\n"

        status += f"  IMU - Heading: {sensors['imu']['heading']:.1f}°, "
        status += f"Angular velocity: {sensors['imu']['angular_velocity']:.1f}°/s\n"

        status += f"\nACTUATORS:\n"
        status += f"  Motors - Left: {self.motor_left.get_speed():.2f} m/s, "
        status += f"Right: {self.motor_right.get_speed():.2f} m/s\n"
        status += f"  Gripper - Position: {self.gripper.servo.get_angle():.1f}°, "
        status += f"Force: {sensors['gripper']['force']:.2f}N, "
        status += f"Holding: {sensors['gripper']['is_gripping']}\n"

        return status


# ==============================================================================
# DEMO MISSIONS
# ==============================================================================

def mission_obstacle_avoidance(robot: VirtualRobot, steps: int = 50):
    """
    Demo 1: Simple obstacle avoidance using ultrasonic sensors.
    Robot moves forward and turns away from obstacles.
    """
    print("\n" + "="*60)
    print("MISSION 1: OBSTACLE AVOIDANCE")
    print("="*60)
    print("Robot will move forward and avoid obstacles using ultrasonic sensors.")
    print("Press Enter to start...")
    input()

    for i in range(steps):
        # PERCEPTION: Read sensors
        sensors = robot.read_sensors()

        # DECISION: Simple obstacle avoidance logic
        front_distance = sensors['ultrasonic']['front']
        left_distance = sensors['ultrasonic']['left']
        right_distance = sensors['ultrasonic']['right']

        # ACTION: Control motors based on sensor readings
        if front_distance < 0.5:
            # Obstacle ahead - turn toward more open side
            if left_distance > right_distance:
                robot.turn_left(0.3)
                action = "Turning LEFT (obstacle ahead)"
            else:
                robot.turn_right(0.3)
                action = "Turning RIGHT (obstacle ahead)"
        elif left_distance < 0.3:
            # Obstacle on left - turn right
            robot.turn_right(0.2)
            action = "Turning RIGHT (obstacle on left)"
        elif right_distance < 0.3:
            # Obstacle on right - turn left
            robot.turn_left(0.2)
            action = "Turning LEFT (obstacle on right)"
        else:
            # Path clear - move forward
            robot.move_forward(0.5)
            action = "Moving FORWARD (path clear)"

        # Update robot physics
        robot.update()

        # Display status every 10 steps
        if i % 10 == 0:
            print(robot.get_status())
            print(f"DECISION: {action}\n")

    robot.stop()
    print("\nMission 1 complete!\n")


def mission_object_detection(robot: VirtualRobot, steps: int = 50):
    """
    Demo 2: Object detection and tracking using camera.
    Robot searches for objects and approaches them.
    """
    print("\n" + "="*60)
    print("MISSION 2: OBJECT DETECTION AND TRACKING")
    print("="*60)
    print("Robot will search for objects using camera and approach them.")
    print("Press Enter to start...")
    input()

    object_found = False

    for i in range(steps):
        # PERCEPTION
        sensors = robot.read_sensors()
        detected_objects = sensors['camera']

        # DECISION
        if detected_objects:
            # Object visible - approach it
            nearest = robot.camera.get_nearest_object()
            object_found = True

            # Calculate turn needed
            angle_error = nearest['angle']
            distance = nearest['distance']

            # ACTION: Proportional control for turning
            if abs(angle_error) > 0.1:  # Not aligned
                turn_speed = max(-0.5, min(0.5, angle_error * 2.0))
                robot.set_motor_speeds(-turn_speed, turn_speed)
                action = f"Turning to align with {nearest['type']}"
            elif distance > 0.5:  # Aligned but far
                robot.move_forward(0.4)
                action = f"Approaching {nearest['type']} ({distance:.2f}m away)"
            else:  # Close enough
                robot.stop()
                action = f"Reached {nearest['type']}!"
                print(robot.get_status())
                print(f"DECISION: {action}\n")
                break
        else:
            # No object visible - search by rotating
            robot.turn_left(0.2)
            action = "Searching for objects (rotating)"
            if not object_found:
                object_found = False

        robot.update()

        # Display status every 10 steps
        if i % 10 == 0:
            print(robot.get_status())
            print(f"DECISION: {action}\n")

    robot.stop()
    print("\nMission 2 complete!\n")


def mission_pick_and_place(robot: VirtualRobot, steps: int = 100):
    """
    Demo 3: Complete pick and place using camera, navigation, and gripper.
    Demonstrates full perception-decision-action loop.
    """
    print("\n" + "="*60)
    print("MISSION 3: PICK AND PLACE")
    print("="*60)
    print("Robot will locate, approach, and pick up an object using")
    print("camera, ultrasonic sensors, and gripper.")
    print("Press Enter to start...")
    input()

    # Mission states
    STATE_SEARCH = 0
    STATE_APPROACH = 1
    STATE_ALIGN = 2
    STATE_PICK = 3
    STATE_DONE = 4

    state = STATE_SEARCH

    # Open gripper initially
    robot.gripper.open()

    for i in range(steps):
        # PERCEPTION
        sensors = robot.read_sensors()
        detected_objects = sensors['camera']
        front_distance = sensors['ultrasonic']['front']

        # DECISION & ACTION (state machine)
        if state == STATE_SEARCH:
            if detected_objects:
                state = STATE_APPROACH
                action = "Object detected! Switching to APPROACH mode"
            else:
                robot.turn_left(0.2)
                action = "SEARCH: Rotating to find object"

        elif state == STATE_APPROACH:
            if not detected_objects:
                state = STATE_SEARCH
                action = "Lost object! Switching to SEARCH mode"
            else:
                nearest = robot.camera.get_nearest_object()
                angle_error = nearest['angle']
                distance = nearest['distance']

                if abs(angle_error) > 0.15:
                    # Not aligned - turn
                    turn_speed = max(-0.4, min(0.4, angle_error * 2.0))
                    robot.set_motor_speeds(-turn_speed, turn_speed)
                    action = f"APPROACH: Aligning (angle error: {math.degrees(angle_error):.1f}°)"
                elif distance > 0.4:
                    # Aligned but far - move forward
                    robot.move_forward(0.3)
                    action = f"APPROACH: Moving forward ({distance:.2f}m to go)"
                else:
                    # Close enough - prepare to pick
                    robot.stop()
                    state = STATE_ALIGN
                    action = "APPROACH: Reached object! Switching to ALIGN mode"

        elif state == STATE_ALIGN:
            # Fine alignment before picking
            if front_distance > 0.25:
                robot.move_forward(0.1)
                action = f"ALIGN: Fine positioning ({front_distance:.2f}m)"
            else:
                robot.stop()
                state = STATE_PICK
                action = "ALIGN: Positioned! Switching to PICK mode"

        elif state == STATE_PICK:
            # Close gripper
            robot.gripper.close()
            action = f"PICK: Closing gripper (angle: {robot.gripper.servo.get_angle():.1f}°)"

            if robot.gripper.is_closed():
                if robot.gripper.is_gripping():
                    state = STATE_DONE
                    action = "PICK: Object grasped! Mission SUCCESS!"
                    print(robot.get_status())
                    print(f"DECISION: {action}\n")
                    robot.update()
                    break
                else:
                    action = "PICK: Gripper closed but no object detected"

        elif state == STATE_DONE:
            action = "Mission complete!"
            break

        robot.update()

        # Display status every 10 steps
        if i % 10 == 0 or state in [STATE_ALIGN, STATE_PICK]:
            print(robot.get_status())
            print(f"STATE: {['SEARCH', 'APPROACH', 'ALIGN', 'PICK', 'DONE'][state]}")
            print(f"DECISION: {action}\n")

    print("\nMission 3 complete!\n")


def main():
    """Main demo program."""
    print("\n" + "="*60)
    print("VIRTUAL ROBOT SIMULATION")
    print("Physical AI - Lesson 2.3")
    print("="*60)
    print("\nThis simulation demonstrates:")
    print("  - Multiple sensor types (ultrasonic, camera, IMU, touch)")
    print("  - Multiple actuators (motors, servos, gripper)")
    print("  - Perception-Decision-Action loop")
    print("  - Closed-loop control with sensor feedback")

    # Create environment and robot
    env = Environment(width=10.0, height=10.0)
    robot = VirtualRobot(env)

    print("\n" + "="*60)
    print("AVAILABLE MISSIONS:")
    print("="*60)
    print("1. Obstacle Avoidance - Navigate using ultrasonic sensors")
    print("2. Object Detection - Find and approach objects with camera")
    print("3. Pick and Place - Complete manipulation task")
    print("4. Run All Missions")
    print("5. Exit")

    while True:
        choice = input("\nSelect mission (1-5): ").strip()

        if choice == "1":
            mission_obstacle_avoidance(robot, steps=50)
        elif choice == "2":
            # Reset robot for new mission
            robot.position = Vector2D(2.0, 2.0)
            robot.heading = 0.0
            mission_object_detection(robot, steps=50)
        elif choice == "3":
            # Reset robot for new mission
            robot.position = Vector2D(2.0, 2.0)
            robot.heading = 0.0
            robot.gripper.open()
            # Reset objects
            for obj in env.objects:
                obj["picked"] = False
            mission_pick_and_place(robot, steps=100)
        elif choice == "4":
            mission_obstacle_avoidance(robot, steps=50)
            robot.position = Vector2D(2.0, 2.0)
            robot.heading = 0.0
            mission_object_detection(robot, steps=50)
            robot.position = Vector2D(2.0, 2.0)
            robot.heading = 0.0
            robot.gripper.open()
            for obj in env.objects:
                obj["picked"] = False
            mission_pick_and_place(robot, steps=100)
        elif choice == "5":
            print("\nExiting simulation. Goodbye!")
            break
        else:
            print("Invalid choice. Please select 1-5.")


if __name__ == "__main__":
    main()

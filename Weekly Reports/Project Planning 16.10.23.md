## Adaptive Navigation of Land Robots in Tight Spaces and Dynamic Environments using RL Weekly Report 16.10.2023 / Project Planning

### Table of Contents
- [Adaptive Navigation of Land Robots in Tight Spaces and Dynamic Environments using RL Weekly Report 16.10.2023 / Project Planning](#adaptive-navigation-of-land-robots-in-tight-spaces-and-dynamic-environments-using-rl-weekly-report-16102023--project-planning)
  - [Table of Contents](#table-of-contents)
  - [Project Summary](#project-summary)
  - [Shortages of Traditional Algorithms and RL's Advantages for Navigation](#shortages-of-traditional-algorithms-and-rls-advantages-for-navigation)
  - [Robot Overview](#robot-overview)
    - [Wheel Choice](#wheel-choice)
    - [Sensors](#sensors)
    - [Localization](#localization)
    - [Mapping](#mapping)
    - [Wheels Motor Control](#wheels-motor-control)
  - [Nav2 \& RL Combination: A New Frontier in Robotic Navigation](#nav2--rl-combination-a-new-frontier-in-robotic-navigation)
    - [The Challenge](#the-challenge)
    - [Possible Approaches](#possible-approaches)
    - [Current Trends](#current-trends)

### Project Summary
In our control & automation bachelor's design project, we aim to simulate a robot, using Gazebo, that can navigate tight and dynamically-obstructed warehouses or production lines. Combining the capabilities of ROS 2 tools like Nav2 and Cartographer with Reinforcement Learning, the robot will adaptively navigate between waypoints, especially in scenarios where traditional algorithms might be challenged. The robot's operations and decisions will be visualized in real-time using Rviz, offering an innovative approach to advanced robotics in complex environments.

### Shortages of Traditional Algorithms and RL's Advantages for Navigation

Traditional navigation algorithms, like those implemented in nav2, often employ deterministic methods to determine paths for robotic systems. While these methods are generally reliable in well-defined environments, their deterministic nature might cause them to falter in complex and tight settings, leading the robot into a state where it might get stuck or struggle to find a viable path.

**Example 1**: Imagine a robot that enters a tight aisle in a warehouse to reach a specific waypoint and deliver a package. Using nav2 with a differential wheeled robot setup, the robot's default behavior might be to attempt to rotate and exit the aisle by moving forward. In such a tight space, this could lead to the robot getting stuck or having difficulties navigating. However, with RL, the robot can learn that in such scenarios, it might be more efficient or safer to reverse out of the aisle instead.

Reinforcement Learning (RL) offers a layer of adaptability absent in traditional methods. Some distinctive advantages of RL in navigation include:

1. **Dynamic Reactions**: Unlike traditional methods that might be challenged in tight spots, an RL agent can determine a sequence of actions that could be non-obvious but effective.

2. **Learning from Mistakes**: If an RL-trained robot finds itself in an unfavorable situation, it can adapt its strategy over subsequent iterations.

3. **Environmental Interactions**: Beyond just navigating, RL agents can learn strategies that involve engaging with their surroundings, such as pushing movable obstacles to clear a path.

4. **Human-aware Navigation**: RL enables robots to discern between static and dynamic obstacles. 

**Example 2**: In a setting where the robot encounters a group of people blocking its path, a traditional deterministic algorithm might immediately seek an alternate route. With RL, the robot can learn that sometimes it's more efficient to wait for a short while for the path to clear, especially if the obstruction is temporary, like a group of people grabbing their coffee or having a brief conversation.

5. **Adaptive Speed & Behavior**: In crowded settings, an RL agent can adjust its speed to ensure safety. Moreover, in dynamic environments like a bustling warehouse, the robot might learn context-aware behaviors like following a human worker or selecting specific routes during busy hours.

6. **Fallback Behaviors**: RL provides a safety net, enabling the robot to devise backup plans when faced with unexpected scenarios.

7. **Optimal Path Selection**: RL allows robots to weigh various factors to choose the most optimal path.

8. **Interactive Behaviors**: RL can empower robots to interact with their environment in innovative ways, for instance, moving a lightweight obstacle aside to proceed.

Incorporating RL with traditional algorithms like nav2 results in a holistic approach to robotic navigation. Nav2 guarantees basic navigational standards, while RL facilitates more intricate, adaptive behaviors for effective navigation in challenging environments.

### Robot Overview

#### Wheel Choice

Mobile robots can be designed with various wheel configurations, each catering to specific needs:

- **Differential Drive**: Uses two separately driven wheels for motion.
  - **Advantage**: Offers simplicity in design and control.
  - **Disadvantage**: Lacks the ability to strafe sideways.

- **Omni-wheels**: Allows omnidirectional movement without needing to rotate the entire robot.
  - **Advantage**: Can move in any direction, offering enhanced maneuverability.
  - **Disadvantage**: Often requires a more complex wheel arrangement and can be mechanically intricate.

- **Ackermann Steering**: Uses steerable wheels, typically seen in cars and trucks.
  - **Advantage**: Provides efficient turning without skidding, especially at high speeds.
  - **Disadvantage**: Limited to forward, backward, and rotational movements without the ability to strafe.

Considering our robot's requirements of efficient navigation in tight, dynamic spaces, we've chosen the **Mecanum Wheel** design. Here's why:

- **Omnidirectional Movement**: Mecanum wheels allow the robot to move in any direction, including strafing side-to-side and moving forward or backward, while also being able to spin in place to change its orientation without the need for a wide turning radius.

- **Agility in Tight Spaces**: Perfect for environments like warehouses and production lines where the robot can exploit its agility to traverse narrow corridors, make sharp turns, and position itself with precision.

- **Simplified Mechanical Design**: Unlike other complex steering systems, Mecanum wheels offer omnidirectional capabilities without additional steering actuators by merely varying the speed and direction of each wheel.

- **Predictable Motion**: When expertly controlled, a Mecanum-wheeled robot can provide smooth and predictable movements, an essential trait for precision tasks.

Given these advantages and our specific operational needs, the Mecanum wheel system emerges as the superior choice for our robot's mobility.



#### Sensors

For effective navigation and mapping in complex environments, our robot will be equipped with a suite of sensors that provide comprehensive environmental feedback:

- **Wheel Encoders**: While traditionally more common in differential drive setups, wheel encoders can still be used in a Mecanum configuration. They measure the rotation of each wheel, which aids in estimating the robot's position over time.

- **IMU (Inertial Measurement Unit)**: This sensor captures the robot's linear and angular velocity. When combined with wheel encoders, it aids in determining the robot's orientation and position, a process commonly referred to as odometry.

- **2D LiDAR**: This laser-based sensor sweeps its environment to measure distances to obstacles in a two-dimensional plane. With high accuracy and a fast rate of data collection, 2D LiDARs are essential for real-time obstacle detection and basic mapping.

- **RGBD Camera**: An RGBD camera captures both color (RGB) and depth (D) data for its field of view. This allows the robot not only to see its surroundings in color but also to gauge the distance to various objects. Such sensors are beneficial for tasks that require understanding the appearance and depth of a scene simultaneously.

One advanced technique to enhance mapping capabilities involves the fusion of data from the 2D LiDAR and the RGBD camera. By combining the high-resolution depth data from the RGBD camera with the broad coverage of the LiDAR, it's possible to construct detailed 3D maps of the environment. This fusion not only increases the richness of the map data but also aids in identifying more complex structures and obstacles in the robot's path.


#### Localization

Localization refers to the robot's ability to determine its position and orientation within a pre-known map of the environment. It's essential for the robot to pinpoint its location to effectively plan and execute navigation tasks.

In our setup:

- **Odometry**: Using wheel encoders and the IMU, the robot estimates its relative position over time. While this method is quick, errors might accumulate over extended distances due to factors like wheel slippage.

- **2D LiDAR & RGBD Data**: These sensors can be used to perform scan matching against a known map, refining the robot's position estimate. By contrasting the current sensor readings with the map, the robot can correct any odometry drift.

- **ROS2 Package**: We'll employ the `nav2_amcl` package, which is the ROS2 adaptation of AMCL (Adaptive Monte Carlo Localization). This uses a particle filter to estimate the robot's pose against a known map with the help of LiDAR data.

#### Mapping

Mapping is the process of creating a representation of the robot's environment. Depending on the application, this can be done in real-time (Simultaneous Localization and Mapping - SLAM) or beforehand.

For our robot:

- **2D LiDAR**: This sensor's range data can help construct a 2D occupancy grid map of the environment, distinguishing between open spaces, obstacles, and undetermined regions.

- **RGBD Camera Fusion**: By merging the depth data from the RGBD camera with 2D LiDAR readings, we can develop 3D maps that encapsulate more environment details. This fusion gives a comprehensive view, especially beneficial in environments with different elevations and intricate structures.

- **ROS2 Packages**: 
  - For SLAM, we'll utilize `cartographer_ros` adapted for ROS2. Cartographer is a versatile SLAM solution that can work with various sensor setups, including the fusion of LiDAR and RGBD camera data.

Both localization and mapping are foundational for ensuring safe and effective robot navigation, enabling the robot to understand its position and its surroundings.


#### Wheels Motor Control

To achieve precise and effective motion with our Mecanum-wheeled robot, controlling the speed and direction of each wheel is crucial. Due to the unique design of Mecanum wheels, the robot's omnidirectional movement is a result of the combined velocities of all four wheels. 

For our setup:

- **Individual Control**: Each of the four wheels will be powered by its own motor, allowing for independent speed and direction control.

- **PID Controllers**: To maintain accurate wheel speeds and compensate for discrepancies caused by factors like terrain irregularities, load changes, or motor differences, we'll implement a PID (Proportional-Integral-Derivative) controller for each wheel. These controllers work by constantly comparing the desired speed with the actual speed and adjusting the motor power accordingly to minimize errors.

- **ROS2 Integration**: Within the ROS2 framework, the PID controllers can be integrated as nodes, communicating with both the high-level navigation modules and the low-level motor drivers. This ensures that the robot's motion is synchronized with the planned paths and adapts in real-time to any environmental changes.

With this setup, the robot is expected to move smoothly, responding effectively to navigation commands, and adjusting swiftly to changes in its environment or mission objectives.


### Nav2 & RL Combination: A New Frontier in Robotic Navigation

Navigating through complex environments is one of the primary challenges in robotics. Traditional navigation stacks, like Nav2 in the ROS ecosystem, have done a commendable job in handling a variety of scenarios. However, there are specific situations where traditional algorithms might not provide the optimal solution, and this is where Reinforcement Learning (RL) comes into play.

#### The Challenge
Let's consider a scenario to understand the potential conflict between traditional navigation and RL. Suppose a robot enters a tight aisle to reach a waypoint and deliver an item. Upon completion, it's time to exit. With Nav2 on a differential-wheeled robot, the default behavior might be to turn the robot around and proceed forwards. However, an RL agent, having learned from numerous similar encounters, might deduce that reversing out of the tight spot is a more optimal solution. Here, a clear conflict arises between the standard behavior of Nav2 and the learned behavior of the RL agent.

#### Possible Approaches
Understanding this potential conflict, there are several methods to integrate Nav2 and RL:

1. **Hierarchical Decision Making**:
    - **How it works**: The high-level decisions, like general path planning, are tackled using traditional navigation algorithms such as Nav2. However, for low-level motion decisions, particularly in challenging scenarios, the RL agent takes over.
    - **Example**: In a warehouse, Nav2 decides the overall path from point A to B. But upon encountering a tight space, the RL agent might execute a specific maneuver, like reversing out, based on its learned behavior.
    
2. **Switching Mechanism**:
    - **How it works**: A mechanism that switches control between Nav2 and the RL agent based on certain criteria, like the complexity of the scenario.
    - **Example**: In open spaces, Nav2 handles navigation. But when the robot detects it's in a confined space, control is handed over to the RL agent.
    
3. **RL as a Feedback Mechanism**:
    - **How it works**: Instead of replacing Nav2's commands, the RL agent provides corrections if the robot is about to make a sub-optimal decision or get stuck.
    - **Example**: Nav2 might command the robot to move forward, but if an obstacle is detected that could cause a deadlock, the RL agent intervenes with a corrective action.
    
4. **Pure RL Approach**:
    - **How it works**: Nav2 is discarded entirely in favor of a full RL-based navigation system.
    - **Challenges**: RL requires extensive data for training, the robot might make suboptimal decisions during exploration, and ensuring the RL agent generalizes well to unforeseen scenarios can be tough.

#### Current Trends
While all these approaches have their merits, hierarchical decision-making is gaining traction in the robotics community. It offers a balanced blend of traditional algorithms' reliability and the adaptability of RL. By having Nav2 decide on the overall navigation strategy and letting RL handle challenging micro-decisions, robots can navigate efficiently even in complex environments.

The choice of approach largely depends on the specific application, the environment, and the robot's objectives. But with the rapid advancements in both Nav2 and RL, the fusion of these methods heralds an exciting era for robotic navigation.

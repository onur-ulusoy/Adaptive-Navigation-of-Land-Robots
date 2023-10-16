## Adaptive Navigation of Land Robots in Tight Spaces and Dynamic Environments using RL Weekly Report 16.10.2023 / Project Planning

### Project Summary:
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



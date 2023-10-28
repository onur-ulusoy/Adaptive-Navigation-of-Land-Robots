## RouteWiseAI Weekly Report 23.10.2023 / Development and Integration
- [RouteWiseAI Weekly Report 23.10.2023 / Development and Integration](#routewiseai-weekly-report-23102023--development-and-integration)
  - [Summary](#summary)
  - [Details](#details)
  - [Next Steps](#next-steps)

### Summary

This week encapsulated the initial stages of environment creation and URDF model development for the project. A parametric approach was adopted for core geometry development, laying the groundwork for the URDF model. Initial URDF.xacro file was created and iteratively refined with inertial information and geometry checks. The simulation environment setup marked a significant stride with the creation of bringup, simulation, and description packages. These packages are fundamental in establishing the simulation environment in Gazebo, with an initial map crafted for this purpose. A Gazebo launch file was also developed to facilitate the simulation bring-up.

### Details

- **Core Geometry Development**: The inception of the project saw the development of the core geometry of RouteWise in a parametric manner, which is pivotal for the URDF model creation.

- **URDF Development**: The initial URDF.xacro file was crafted along with a bash file for geometry checking. Further refinement was done by developing a node to check the geometry of URDF.xacro seamlessly.

- **Simulation Environment Setup**: 
  - **Bringup Package(rw_bringup)**: Created to start the global launch files and configurations required to bring up the robot simulation.
  - **Simulation Package(rw_simulation)**: Developed to encapsulate the simulation-related files and functionalities including simulation launch.
  - **Description Package(rw_description)**: Contains the XML macros (Xacro), URDF model, meshes, and other descriptive files essential for the robot representation in the simulation.
  
- **Inertia Addition & Gazebo Launch File**: Inertial information was appended to the URDF model for a more accurate simulation. A Gazebo launch file was developed to initiate the simulation environment effortlessly.

- **Geometry Check Improvement**: The geometry check node was updated to operate independently, removing the necessity for the geometry check bash script, streamlining the process further.

- **Initial Map Creation**: An initial map was structured on Gazebo to aid in the simulation setup, marking the first creation of environments for the project.

### Next Steps

- **Mecanum Wheels Integration**: The upcoming phase will see the integration of mecanum wheels into the geometry to facilitate omnidirectional movement capabilities.

- **Gazebo Plugin Development**: A Gazebo plugin will be developed to control the mecanum wheels by listening to nodes, enhancing the control and interaction with the robot within the simulation environment.

- **Node Development**: Nodes will be created to control and manipulate the robot's movement, further enriching the simulation setup and testing capabilities.

The foundational work done in this phase is instrumental for the simulation setup and URDF model enhancement, setting the stage for the integration and control of mecanum wheels in the ensuing week.

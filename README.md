# Model Predictive Interaction Controller for Path-Following (MPIC-PF)

This package contains the controller implementations for model predictive interaction control based on a path-following formulation (MPIC-PF) and the Task-Skill-MP Framework.

Please cite the paper https://doi.org/10.1109/ICMA54519.2022.9856004  when you are using results obtained with the MPIC-PF.

---

## 1. Prerequisites
- **Operating System:** Ubuntu 20.04 with real-time kernel
- **ROS Version:** ROS Noetic including Gazebo
- **Dependencies:**
    - `grampc` library
    - `yaml-cpp` library
    - `Pinocchio` library
    - `libfranka` and `franka_ros` for Franka Emika robots
- **Catkin Workspace Structure**:
    `~/catkin_ws/src/ /libs/ /grampc/ /yaml-cpp/ /... /lrt_mpipfc/ /...`


## 2. Installation Guide
#### 2.1 Grampc
- Clone the `grampc` repository into `~/catkin_ws/src/libs`:
    ```bash
    cd ~/catkin_ws/src/libs
    git clone https://github.com/grampc/grampc.git
    ```
- Follow the installation instructions in the `/grampc/doc` folder.
- Ensure the `cpp` folder is compiled.

#### 2.2 Yaml-cpp
- Clone the `yaml-cpp` repository into `~/catkin_ws/src/libs`:
    ```bash
    cd ~/catkin_ws/src/libs
    git clone https://github.com/jbeder/yaml-cpp.git
    ```
- Follow the installation instructions in the `yaml-cpp/install.txt` file.

#### 2.3 Pinocchio
-- Install the Pinocchio library following the instructions at [Pinocchio Installation Guide](https://stack-of-tasks.github.io/pinocchio/download.html).
-- Add the required environment variables to your `~/.bashrc` file.

#### 2.4 franka_ros and libfranka
-- Install `libfranka` and `franka_ros`following the inst5ructions at [libfranka and franka_ros Installation Guide](https://frankaemika.github.io/docs/installation_linux.html).

## 3. Settin Up the ROS Workspace
-- Navigate to the Catkin workspace and build:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
- Source the workspace to make the package available:
    ```bash
    source ~/catkin_ws/devel/setup.bash
    ```

## 4. Running a First Example
### 4.1 Simulated Controller
To run the MPIC-PF controller in a simulation environment:
- **Launch the simulation:**  
    ```bash
    roslaunch lrt_mpipfc sim_mpipfc.launch
    ```

### 4.2 Real Robot Controller
To run the MPIC-PF controller on a real Franka Emika robot:
- **Launch the real robot controller:**  
    ```bash
    roslaunch lrt_mpipfc real_mpipfc.launch robot:=panda robot_ip:=172.16.0.3 use_fts:=false use_pbd_controller:=false
    ```
  - Replace `<robot_ip>` with the IP address of your robot.  
  - Set `use_fts` to true if using an external force-torque sensor.  
  - Set `use_pbd_controller` to true to enable the PBD controller for hand guidance.

### 4.3 Package Structure
The package is organized as follows:

- **`config/`**: Contains configuration files for the controller.
  - `gazebo_pid_gains.yaml`: PID gains for the Gazebo simulation.
  - `lrt_mpipfc.yaml`: General configuration for the MPIC-PF controller.
  - `sim_mpicpf.yaml`: Configuration of the MPIC-PF controller for the simulated robot, including the general MPC parameterization.
  - `real_mpicpf.yaml`: Configuration of the MPIC-PF controller for the real robot, including the general MPC parameterization.
  - `Tasks/`: Yaml-files for task configuration.

- **`exp_data/`**: Stores experimental data and scripts for data analysis.
  - `data.csv`: Data file.
  - `plot_MPIC_PF.m`: MATLAB script for plotting results.

- **`include/`**: Header files for the project.
  - `mp_library/`: Headers for the individual MPs.
  - Headers for the MPIC-PF controller, task control and problem description.

- **`launch/`**: ROS launch files for running the controller.
  - `rviz`: Configuration file for rviz visualization.
  - `real_mpipfc.launch`: Launch file for the real robot controller.
  - `sim_mpipfc.launch`: Launch file for the simulation environment.

- **`src/`**: Source code for the project.
  - `gripper.cpp`: Code for gripper control.
  - `mpic_controller.cpp`: Core implementation of the MPIC-PF controller.
  - `real_mpic_controller.cpp`: Implementation for the real robot.
  - `sim_mpic_controller.cpp`: Implementation for the simulation environment.
  - `task_framework.cpp`, `skill_framework.cpp`, `mp_framework.cpp` and `mp_library/`: Task-Skill-MP framework for task execution.

- **`urdf/`**: Contains URDF files for robot description.
  - `meshes/`: Mesh files for the robot.
  - `robots/`: Robot-specific URDF files.
  - `worlds/`: Gazebo world files for simulation.

### 4.4 Task Programming
- **Specify the path to task description** in `.yaml` files under the `lrt_mpipfc/config/Tasks/`Tasks directory.
- **Adjust the path to the task `task.yaml` file** in line 2-7 of the corresponding launch files (`real_mpipfc.launch` or `sim_mpipfc.launch`) to ensure the correct task configuration is loaded.
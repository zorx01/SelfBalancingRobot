# LQR_SBR_Gazebo

## Requirements
- Ubuntu 22.04
- ROS2 Humble
- Gazebo 11
- Python (matplotlib, control, numpy)

## Setup Instructions

1. Create the `colcon_ws` directory in the Home folder. Inside `colcon_ws`, create a folder named `src`:
    ```bash
    mkdir -p ~/colcon_ws/src
    ```
    Place the `teeterbot` folder, which contains all the launch files and scripts, inside the `src` folder.

2. Navigate to `colcon_ws`, then open a terminal and run:
    ```bash
    colcon build --symlink-install
    ```
    **Note:** Run this command only once to build the project.

3. In `colcon_ws`, run the command:
    ```bash
    source install/setup.bash
    ```

4. Then, in `colcon_ws`, run the command:
    ```bash
    ros2 launch teeterbot_gazebo teeterbot_empty_world.launch.py
    ```
    This will launch the SBR in Gazebo.

5. In another terminal, navigate to `src/teeterbot/LQR` and run:
    ```bash
    python3 LQR_SBR_Gazebo.py
    ```

**Note:** If Gazebo does not spawn, run the following command in the same terminal:
    ```bash
    source /usr/share/gazebo-11/setup.sh
    ```


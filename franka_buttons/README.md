# Read franka buttons in ROS

This is a ROS repo to read the state of the buttons of franka robot.

**TODO: Move this to the top-level fodler in the repository and update the install instructions to match the correct filepaths**

## Installation

**TODO: list extra dependencies (requests and specific websocket version)**

### Using virtual environment (WIP)

**Warning: these instructions do not work properly yet! More testing is required to get installation in a virtual environment working.**

If you do not want to pollute the global installation of Python (which is recommended), the following steps allow you to install the additional dependencies to a virtual environment and run ROS from that environment.

1. Install dependencies:

   ```bash
   sudo apt install python3-venv
   ```

2. In your workspace folder, source ROS to obtain links to the ROS system packages and create a virtual environment with these packages. Also add a `COLCON_IGNORE` file to the virtual environment folder to prevent `colcon` from trying to build the `.venv` contents.

   ```bash
   cd ~/your_workspace
   source /opt/ros/humble/setup.bash
   python3 -m venv .venv --system-site-packages --symlinks
   touch .venv/COLCON_IGNORE
   ```

3. Add the following snippet to `setup.cfg` of all packages that will use dependencies installed in the virtual environment (this is already done for this package):

   ```text
   [build_scripts]
   executable = /usr/bin/env python3
   ```

4. Activate the virtual environment and install python packages (you can repeat this step whenever you want to install additional packages):

   ```bash
   source .venv/bin/activate

   # OR from a requirements.txt file
   # TODO: Perhaps there is a way to get all requirements.txt files from files inside the src/ folder?
   python3 -m pip install -r src/franka_buttons_ros2/franka_buttons/requirements.txt

   # OR directly
   python3 -m pip install package1 package2
   ```

5. Build the workspace **with the virtual environment sourced** and **using `python -m colcon ...`**. This ensures that the workspace is built using the packages in the virtual environment (source: <https://github.com/ros2/ros2/issues/1094#issuecomment-1503725544>). (you can repeat this step whenever you want to build the workspace):

   ```bash
   source .venv/bin/activate
   python -m colcon build --symlink-install --packages-up-to franka_buttons
   ```

6. Source the built files and run the nodes. **Order matters! Source the built workspace first, and then the virtual environment.** (you can repeat this step whenever you want to run or launch):

   ```bash
   # Order matters!
   source install/setup.bash
   source .venv/bin/activate
   ros2 run package_name node_name
   ```

These steps are based on the following resources:

- <https://medium.com/ros2-tips-and-tricks/running-ros2-nodes-in-a-python-virtual-environment-b31c1b863cdb>
- <https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html>
- <https://www.theconstruct.ai/ros2-how-to-install-third-party-python-packages-using-ros2-5/>

## Configuring Franka Desk credentials

To connect to the pilot buttons, we need to connect to the Franka Desk using a username and password. The following steps explain how to configure them:

1. Copy the `.env` template to the `.ros` working directory:

   ```bash
   mkdir -p ~/.ros/franka_buttons/credentials
   cp -i ~/<your_workspace>/src/franka_buttons_ros2/franka_buttons/.env.template ~/.ros/franka_buttons/credentials/.env

2. Add your credentials to the copied file:

   ```bash
   nano ~/.ros/franka_buttons/credentials/.env
   ```

The client credentials for Franka Desk need to be stored in a specific location

## Building

To install the package you need to clone the repo in your workspace and compile it with catkin build.

To read the state of the buttons of the robot, you need to run the following command:

```
roslaunch franka_buttons read_buttons.launch robot_ip:=<robot_ip> username:=<username> password:=<password>
```

where `<robot_ip>` is the ip of the robot. `<username>` and `<password>` are the username and password of the robot that you have in the Desk.

The node will publish th
e state of the buttons in the topic `/franka_buttons/x` and `/franka_buttons/y` as a `Float32` message. This topic returns 0 if not pressed 1 if pressed in postive direction and -1 if pressed in negative direction. I follow the base frame conventions.

You can also read the state of the circle, the check and the cross buttons. The topics are `/franka_buttons/circle`, `/franka_buttons/check` and `/franka_buttons/cross`. This messages are of type `Bool`.

![Figure](buttons.jpg)

Have fun, and if you have any question, please let me know.

**TODO: Properly acknowledge and cite the authors of `panda-py` and the original `franka_buttons`.**

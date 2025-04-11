# IGVC 2024 Simulation
The following material includes the relevant steps and procedures to get the 2024 IGVC simulation up and running on your local system.

### Relevant Software:
Simulation Software: Gazebo <br />
ROS Distribution: Humble <br/>
Build Tool: Colcon <br />
Environment: WSL (Windows Subsystem for Linux)

### Dependencies:
sudo apt install ros-humble-xacro <br />
sudo apt install ros-humble-joint-state-publisher-gui <br />
sudo apt install ros-humble-ros2-control <br />
sudo apt install ros-humble-ros2-controllers <br />
sudo apt install ros-humble-gazebo-ros-pkgs <br />
sudo apt install ros-humble-gazebo-ros2-control <br />

### Using the Custom World Models:
Within the custom_models directory you will find a series of folders consisting of varying road types and obstacles (ignore the baseframe.STL file). These are the models that the custom Gazebo IGVC course world pulls from. Given that you have already installed Gazebo locate the following directory "/home/user/.gazebo/models". Copy and paste each of the model folders (except baseframe.STL) into the Gazebo models directory listed previously.

### Building and Running the Simulation Package:
1. Make sure you have setup a ROS2 workspace already in your operating system's environment. <br />
&ensp; If not refer to the following video (For Windows Users): https://www.youtube.com/watch?v=KLvUMtYI_Ag&list=PLSK7NtBWwmpTS_YVfjeN3ZzIxItI1P_Sr&index=14 <br />

2. Clone the simulation repository under your ROS2 Workspace src directory, for example "/home/user/ros2_ws/src".

3. Traverse back to your ROS2 Workspace directory, for example "/home/user/ros2_ws".

4. Run the following command to build the simulation package: <br />
    "colcon build --packages-select autonav_sim"

5. Run the following command to source the directory: <br />
    ". install/setup.bash"

6. Run the following command to launch the simulation with the robot spawned in the IGVC course world: <br />
    "ros2 launch autonav_sim launch_sim.launch.py world:=./src/autonav_sim/worlds/autonav_igvc_course.world"

### Troubleshooting

- if the node does not launch due to "x package not found", but all the Dependencies in this file are installed, try sourcing only the local setup for this package instead of all packages in the repo. This can be done as following:
```bash
# if you were doing this:
source install/setup.bash
# or something similar,
# try this:
source install/autonav_sim/share/local_setup.bash
# there is probably a cleaner solution... if one is discovered feel free to remove this point
```
- if the simulation does not render any world info at all, please ensure you are in the correct directory when running the launch file. If your launch file begins with "./src", you need to be in the "isaac_ros-dev" directory when launching.
- if the simulation fails to load some models in the world, ensure that all the required custom models are present in your .gazebo file. On a unix system (including WSL), this can be done by running the following from the "isaac_ros-dev" directory:
```bash
cd src/autonav_sim/custom_models \
cp * ~/.gazebo/models/ -r 
```
the models should now properly load.


### Helpful Command and Tools:
- "ros2 run teleop_twist_keyboard teleop_twist_keyboard" <br />
    While you have the Gazebo simulation running open another terminal and move into your ROS2 workspace directory. Once this is done if you run this command you can move the robot in the simulation by using the controls provided by the teleop_twist_keyboard pluggin.
- "ros2 run autonav_sim opencv_node" <br />
    While the gazebo simulation is running if you open another terminal and move to the ROS2 workspace then source the directory. Running the following command will start the built-in node to show you the current camera feed on the robot.
- "ros2 launch autonav_sim launch_sim.launch.py" <br />
    Spawning the robot in an empty Gazebo Simulation.


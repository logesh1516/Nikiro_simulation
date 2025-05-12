# Nikiro_simulation
Nikiro_simulation is a ROS 2-based simulation framework for the Nikiro Autonomous Mobile Robot (AMR) platform. It is designed for indoor navigation using a differential drive system and provides seamless integration with Gazebo, RViz, and Nav2 for:

SLAM (Simultaneous Localization and Mapping)

Localization

Path Planning

This repository primarily focuses on the simulation aspects of both the MyCobot robotic arm and the AMR, offering a modular and extensible setup for research, development, and testing.

# 🚀 Features

  **Differential Drive Simulation:** Accurately simulates AMR movement with a differential drive system.
  
  **Modular Architecture:** Easily extend or customize components like sensors, controllers, and robot arms.
  
  **SLAM Integration:** Supports SLAM using tools like slam_toolbox for real-time map building.
  
  **Localization:** Enables localization using AMCL and Nav2's lifecycle nodes.
  
  **Path Planning & Navigation:** Utilizes Nav2 stack for robust path planning, obstacle avoidance, and goal reaching.
  
  **Gazebo Simulation:** Full integration with Gazebo Classic for 3D physics-based simulation.
  
  **RViz Visualization:** Real-time visualization of the robot, sensors, maps, and paths in RViz2.
  
  **Multi-Robot Ready:** Easily scalable to support multiple robots in simulation.
  
  **MyCobot Arm Simulation:** Integrates the MyCobot 6-DOF robotic arm for manipulation tasks.
  Object Detection Support.

# INSTALLATION
```
mkdir -p nikiro_simulation/src
cd nikiro_simulation/src
git clone https://github.com/logesh1516/Nikiro_simulation.git
cd ..
colcon build
```
# SIMULATION 

```
cd nikiro_simulation
source install/setup.bash
```
**AMR SIMULATION**
```
ros2 launch amr_description gazebo.launch.py
```
to map the environment use this command

```
ros2 launch amr_description mapping.launch.py
```
to perform nav2 navigation use this command
```
ros2 launch amr_description nav2.launch.py
```
**CONVEYOR SIMULATION**

**Basic simulation**

```sh
cd ~/colcon_ws
source install/setup.bash
ros2 launch conveyor_description conveyor_model.launch.py 
```
Spawning Objects over the Conveyor

a) Single object

```sh
cd ~/colcon_ws
source install/setup.bash
ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyor_description" --urdf "redcube.urdf" --name "redcube" --x -0.2 --y 0 --z 1
(or)
ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyor_description" --urdf "bluecube.urdf" --name "bluecube" --x -0.2 --y 0 --z 1
(or)
ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyor_description" --urdf "greencube.urdf" --name "greencube" --x -0.2 --y 0 --z 1
```
through this command only red,green and bluecubes can be spawned.

b) Multiple Object

I have created a python script to spawn multiple objects over the conveyor.

Each object can be spawned only once.

```sh
cd colcon_ws
source install/setup.bash
cd ~/src/Conveyor_simulation/ros2_conveyorbelt/python
python3 summoner.py
```
**Simulation of the conveyor**

a) Manual power input

Activate the ConveyorBelt with the desired speed -> Value = (0,100]:

```sh
cd ~/colcon_ws
source install/setup.bash
ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: --}"

```
b) Conveyor Automation using Ir sensors

Two IR sensors are placed in the start and end position of the conveyor for the automation depending on the presence of the object.

```sh
cd ~/colcon_ws
source install/setup.bash
ros2 run conveyor_automation conveyor_automation_node
```
**Mycobot_simulation**

```
ros2 launch mycobot_gazebo mycobotjn_control.launch.py
```
to move the mycobot-arm 
```
ros2 run mycobot_gazebo pick_and_place
```
# SIMULATION IMAGES
![Screenshot from 2025-05-11 22-44-44](https://github.com/user-attachments/assets/aef01611-3f4d-450a-b539-3f6b1ba52c6d)
![Screenshot from 2025-02-09 19-53-31](https://github.com/user-attachments/assets/dd4af00a-3e8d-4e7c-a134-7c5d580e141b)
![Screenshot from 2025-02-09 20-01-57](https://github.com/user-attachments/assets/253ebebc-489d-4ee7-89aa-68d6d321e1ea)
![Screenshot from 2025-02-09 20-36-26](https://github.com/user-attachments/assets/b163a8eb-8576-47a0-bdef-fbba2a5f7504)
![Screenshot from 2024-11-06 23-19-00](https://github.com/user-attachments/assets/f7070e00-396c-4c26-96d4-45eea8033aa4)
![Screenshot from 2024-11-06 22-52-22](https://github.com/user-attachments/assets/78f59595-f534-498f-b012-c187b09fc3e8)
![Screenshot from 2025-03-29 23-35-47](https://github.com/user-attachments/assets/a16476db-2154-4a29-91ff-84b55276e70e)
![Screenshot from 2025-03-27 10-50-12](https://github.com/user-attachments/assets/df5a2f87-f379-4033-b55d-4262d6de225d)
![Screenshot from 2025-01-07 20-27-48](https://github.com/user-attachments/assets/56a8ea1b-e843-4ad6-9a2e-66fd26833575)






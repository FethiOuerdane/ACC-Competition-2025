## 🚀 Software Used


| Docker | QLabs | ROS 2 | Isaac ROS | Python |
|--------|-------|------|-----------|--------|
| <img src="https://upload.wikimedia.org/wikipedia/commons/4/4e/Docker_%28container_engine%29_logo.svg" alt="Docker" width="100"/> | <img src="https://play-lh.googleusercontent.com/AsRg-f1bCP2T_jgAAwAJvcH8i82wnq5Ojgp1wSaB64SnBytM-iehCNzp4xuuBtcxY1Y" alt="QLabs" width="100"/> | <img src="https://roboticsbackend.com/wp-content/uploads/2022/04/ros_logo.png" alt="ROS 2" width="100"/> | <img src="https://avatars.githubusercontent.com/u/91228115?s=280&v=4" alt="Isaac ROS" width="100"/> | <img src="https://upload.wikimedia.org/wikipedia/commons/c/c3/Python-logo-notext.svg" alt="Python" width="100"/> |



---


# 🚗 **Tutorial: Running Virtual QCar with Isaac ROS**  

This tutorial will guide you through setting up and running the **Virtual QCar** inside a **Docker container**, integrating it with **Isaac ROS**

## **00. Running Quanser Interactive Labs (Qlabs) on Ubuntu 24**  

<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/184d441dbfa3440a15a9bb846aea13b66a0dc858/Software_Setup/qlab_1.png" alt="ROS 2"/>


<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/184d441dbfa3440a15a9bb846aea13b66a0dc858/Software_Setup/qlab_2.png" alt="ROS 2"/>

<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/184d441dbfa3440a15a9bb846aea13b66a0dc858/Software_Setup/qlab_3.png" alt="ROS 2"/>



## 📌 **1. Running the Virtual QCar Container**  

First, navigate to the Virtual QCar directory:  

```bash
cd /home/$USER/Documents/ACC_Development/docker/virtual_qcar2
```

Now, run the Virtual QCar Docker container:  

```bash
sudo docker run --rm -it --network host --name virtual-qcar2 quanser/acc2025-virtual-qcar2 bash
```

Once inside the container, navigate to the QCar2 scripts directory:  

```bash
cd /home/qcar2_scripts/python/
```

Set up the competition map:  

```bash
python3 Base_Scenarios_Python/Setup_Competition_Map.py
```
<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/47a83c18e0968cd919eb1b9f3692f628a17ef7ec/Software_Setup/tut_comp_map_terminal.png" alt="ROS 2"/>
---

Hit Enter, and wait for Qlabs to connect with the Program:  
**Notes :** `Setup_Competition_Map.py` is one of many other scenarios you can test with.

<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/47a83c18e0968cd919eb1b9f3692f628a17ef7ec/Software_Setup/map_before_ready.png" alt="ROS 2"/>

After successful connection with Qlabs, you will the view below:

<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/47a83c18e0968cd919eb1b9f3692f628a17ef7ec/Software_Setup/map_ready_terminal.png" alt="ROS 2"/>

**You can adjust the view from the menu on the top**

<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/47a83c18e0968cd919eb1b9f3692f628a17ef7ec/Software_Setup/qcar2_cam_perspectives.png" alt="ROS 2"/>


## 📌 **2. Setting Up Isaac ROS for QCar2**  

### 🔹 **2.1. Running the Isaac ROS Development Environment**  

Open a new terminal and navigate to the **Isaac ROS** directory:  

```bash
cd /home/$USER/Documents/ACC_Development/isaac_ros_common/
```

Run the development environment script:  

```bash
./scripts/run_dev.sh /home/$USER/Documents/ACC_Development/ros2
```

Source ROS 2 Humble:  

```bash
source /opt/ros/humble/setup.bash
```

### 🔹 **2.2. Compile and Launch QCar2 ROS Nodes**  

Compile the QCar2 ROS nodes:  

```bash
colcon build
```

Source the compiled packages:  

```bash
. install/setup.bash
```

Launch the QCar2 nodes configured for the **virtual** QCar:  

```bash
ros2 launch qcar2_nodes qcar2_launch_virtual.py
```


<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/52fefb4579557d3335610cda8396723c562af4a2/Software_Setup/container_2_qcar_isaac_running_m.png" alt="ROS 2"/>


---

## 📌 **3. Visualizing in RViz**  

To visualize the QCar2 data, run:  

```bash
ros2 run rviz2 rviz2
```

---

## 📌 **4. Controlling QCar2 via ros2 Pub command or with Keyboard Python Script**  

Navigate to the QCar2 scripts directory:  

You can send control commands using **ROS 2 topics**.  
For example, to set motor speed and steering angle, use:  

```bash
ros2 topic pub /qcar2_motor_speed_cmd qcar2_interfaces/msg/MotorCommands \
"{motor_names: ['steering_angle', 'motor_throttle'], values: [0.5, 0.7]}"
```

You can use a python script that allows you to Publish keyboard to the `qcar2_motor_speed_cmd` topic and control the Qcar, it also subscribe to the video Stream coming from the Intel Realsense camera (Color Stream). 
```bash
cd src/qcar2_nodes/scripts
```

you can view the script here on Github : [Check the Python Script](keyboard_teleop.py)

<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/52fefb4579557d3335610cda8396723c562af4a2/Software_Setup/teleop_sh.png" alt="ROS 2"/>
---

## 📌 **5. ML Models Included **  

### 🔹 **5.1. YOLO (Traffic Light Detection)**  
Use a **YOLO-based model** to detect traffic lights. provided by Quanser

### 🔹 **5.2. LaneNet (Lane Detection)**  
LaneNet can be integrated to detect road lanes, enabling autonomous lane-following. pt Files Provided by quanser

Model Related Files can be found at this location : 

<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/61e3fa03cca05819c1da2424c7f73b70d75b6749/Software_Setup/ml.png" alt="ROS 2"/>
---


## 📌 **6. Useful ROS 2 Topics**  

To list available ROS 2 topics, run:  

```bash
ros2 topic list
```

Some key topics include:  

- **Camera Topics**  
  ```bash
  /camera/color_image
  /camera/depth_image
  /camera/csi_image
  ```
- **Sensor Topics**  
  ```bash
  /qcar2_imu
  /qcar2_battery
  /scan
  ```
- **Control Topics**  
  ```bash
  /qcar2_motor_speed_cmd
  /qcar2_led_cmd
  ```

---

## 📌 **7. Stopping and Restarting the Docker Container**  

To **stop the container**, find the container name or ID:  

```bash
docker ps
```

Then kill it:  

```bash
docker kill <container_name_or_id>
```

---

# 🚀 Useful Docker Commands for Virtual QCar & Container Management  

This document provides essential **Docker commands** to manage the **Virtual QCar** container and other Docker applications efficiently.  

---

## 📌 1. Basic Container Management  
### 🔹 List Running Containers  
```bash
docker ps
```
### 🔹 List All Containers (Including Stopped Ones)  
```bash
docker ps -a
```
### 🔹 Stop a Running Container  
```bash
docker stop <container_name_or_id>
```
### 🔹 Remove a Container  
```bash
docker rm <container_name_or_id>
```
### 🔹 Force Kill a Container  
```bash
docker kill <container_name_or_id>
```
### 🔹 Restart a Container  
```bash
docker restart <container_name_or_id>
```

---

## 📌 2. Working with Images  
### 🔹 List Downloaded Docker Images  
```bash
docker images
```
### 🔹 Remove an Image  
```bash
docker rmi <image_name_or_id>
```
### 🔹 Pull an Image from Docker Hub  
```bash
docker pull <image_name>
```
### 🔹 Build an Image from a Dockerfile  
```bash
docker build -t <image_name> .
```

---

## 📌 3. Running Containers  
### 🔹 Run a Container with an Interactive Terminal  
```bash
docker run -it <image_name> bash
```
### 🔹 Run a Container in Detached Mode (Background)  
```bash
docker run -d --name <container_name> <image_name>
```
### 🔹 Run a Container and Remove It After Exit  
```bash
docker run --rm -it <image_name> bash
```
### 🔹 Run a Container with a Mounted Volume (Persistent Storage)  
```bash
docker run -v /local/path:/container/path -it <image_name> bash
```

---

## 📌 4. Networking  
### 🔹 List Docker Networks  
```bash
docker network ls
```
### 🔹 Inspect a Specific Network  
```bash
docker network inspect <network_name>
```
### 🔹 Remove a Network  
```bash
docker network rm <network_name>
```
### 🔹 Run a Container on a Specific Network  
```bash
docker run --network=<network_name> -it <image_name> bash
```

---

## 📌 5. Viewing Logs & Debugging  
### 🔹 View Container Logs  
```bash
docker logs <container_name_or_id>
```
### 🔹 Follow Live Logs (Real-time Output)  
```bash
docker logs -f <container_name_or_id>
```
### 🔹 Check Resource Usage of Running Containers  
```bash
docker stats
```
### 🔹 Inspect Container Details  
```bash
docker inspect <container_name_or_id>
```

---

## 📌 6. Executing Commands Inside a Running Container  
### 🔹 Attach to a Running Container  
```bash
docker attach <container_name_or_id>
```
### 🔹 Open a New Terminal Inside a Running Container  
```bash
docker exec -it <container_name_or_id> bash
```
### 🔹 Run a Command Inside a Running Container  
```bash
docker exec -it <container_name_or_id> <command>
```
Example:  
```bash
docker exec -it virtual-qcar2 ls /home/qcar2_scripts/
```

---

## 📌 7. Saving and Loading Containers  
### 🔹 Save a Docker Image as a Tar File  
```bash
docker save -o my_image.tar <image_name>
```
### 🔹 Load a Docker Image from a Tar File  
```bash
docker load -i my_image.tar
```

---

## 📌 8. Docker Compose (For Managing Multi-Container Applications)  
### 🔹 Start a Multi-Container Application  
```bash
docker-compose up
```
### 🔹 Start in Detached Mode  
```bash
docker-compose up -d
```
### 🔹 Stop and Remove All Containers in a Compose Project  
```bash
docker-compose down
```

---

# 🚗 **Must Know things about the Competition (Based on Video)** 

[![Click to watch video](https://github.com/FethiOuerdane/ACC-Competition-2025/blob/d82a34c83a668629b0b9ebb54fc3a0ed19e8a2a3/Software_Setup/acc_comp.png)](https://youtu.be/NtgBwlfGbMc)

[Watch the video on YouTube](https://youtu.be/NtgBwlfGbMc)



<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/b79dc849e864227f203a7d283b9515cfaca5a7a3/Software_Setup/comp_2025_important_locations.png" alt="ROS 2"/>


<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/43da51bace6a37e11045d572dbfa63cf8ff6d515/Software_Setup/com_map_test.png" alt="ROS 2"/>

# Core Principles of Self-Driving for Autonomous Driving Competition

When preparing the team for a competition focused on autonomous driving, it’s crucial to ensure that they understand and prioritize the **Core Principles of Self-Driving** to create a well-rounded and effective approach. Here's how the team should focus on these core principles:
<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/43da51bace6a37e11045d572dbfa63cf8ff6d515/Software_Setup/4_competition_parts.png" alt="ROS 2"/>

## 1. Data Collection
**Focus:**  
The team should develop algorithms that efficiently collect data from a range of sensors (e.g., cameras, LiDAR, radar, ultrasonic). These sensors will provide raw information about the vehicle’s surroundings and its own state.

**Action Steps:**
- Ensure that the vehicle can gather data from both interoceptive sensors (internal systems, like battery status) and exteroceptive sensors (external systems, like obstacles and traffic signals).
- The team should develop data filtering mechanisms to eliminate irrelevant data and ensure accurate readings for decision-making. This will be vital for the vehicle to process the information in real-time.

## 2. Interpretation
**Focus:**  
The next critical step is the interpretation of the collected data. The team needs to design the system to correlate the gathered sensor data with real-world events.

**Action Steps:**
- Use machine learning models and sensor fusion techniques to identify external factors such as traffic lights, pedestrians, road signs, and other vehicles.
- Implement robust systems to track internal data like battery levels, tire pressure, or vehicle health and integrate this information with environmental data. This will ensure the vehicle’s state is continuously monitored and understood.
- Fine-tune algorithms to react to various scenarios, ensuring accurate decision-making in complex or changing environments.

**<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/19d4e34b12670b25a04e9f161871ea502d8f3aac/Software_Setup/traffic_1.png" alt="ROS 2"/>**

**<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/19d4e34b12670b25a04e9f161871ea502d8f3aac/Software_Setup/traffic_2.png" alt="ROS 2"/>**


**<img src="https://github.com/SulimanSalih/KFUPMAUTOWHEELS--Stage1/blob/6d8bdee8ad745c8130f53bd3cfe80dac896c60e4/result.jpg" alt="ROS 2"/>**


## 3. Control Systems
**Focus:**  
Once the vehicle interprets the surrounding environment, it needs to make the right decisions and execute them smoothly and safely. The focus here is on ensuring accurate control over the vehicle's movements.

**Action Steps:**
- Develop the control systems responsible for maintaining lane discipline, speed control, turning, obstacle avoidance, and stopping.
- Implement algorithms for path correction (for example, if the vehicle is off track or veering out of lane), collision avoidance, and emergency stopping.
- Ensure real-time decision-making capabilities that adjust vehicle behavior based on real-time data.

## 4. Localization and Path Planning
**Focus:**  
The vehicle needs to understand where it is in the world and how to get to its destination safely. This is a critical aspect of the competition as it involves determining the best route and adjusting based on the environment.

**Action Steps:**
- Use localization algorithms provided by Quanser
- Detect and Avoid Obstacles


The Folder can be found in Documents Folder on the Lab's PC, and is also in this Github and the Quanser's Github repo of this Competition, the Location to this directory is shown in the screenshot
**<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/19d4e34b12670b25a04e9f161871ea502d8f3aac/Software_Setup/folder_lib.png" alt="ROS 2"/>**

You can check all the necessary python files and learn how use each for the Control, image processing, path planning Tasks

**<img src="https://github.com/FethiOuerdane/ACC-Competition-2025/blob/19d4e34b12670b25a04e9f161871ea502d8f3aac/Software_Setup/important_files.png" alt="ROS 2"/>**
## Additional Tips for the Team
- **Testing and Debugging:** Continuous testing of all sensors, algorithms, and control systems in varied environments will help the team refine their system.
- **Collaboration:** Ensure that the team works closely. Autonomous systems require a multidisciplinary approach to integrate data processing, control, and hardware systems.
- **Optimization:** The team should continuously optimize the algorithms to run efficiently on the vehicle’s processing hardware. This will minimize latency in decision-making and ensure smooth operation.
  
By focusing on these core principles, the team can develop a well-balanced, efficient, and competitive self-driving system for the competition.
---


## 🚀 Software Used


| Docker | QLabs | ROS 2 | Isaac ROS |
|--------|-------|------|-----------|
| <img src="https://upload.wikimedia.org/wikipedia/commons/4/4e/Docker_%28container_engine%29_logo.svg" alt="Docker" width="100"/> | <img src="https://play-lh.googleusercontent.com/AsRg-f1bCP2T_jgAAwAJvcH8i82wnq5Ojgp1wSaB64SnBytM-iehCNzp4xuuBtcxY1Y" alt="QLabs" width="100"/> | <img src="https://roboticsbackend.com/wp-content/uploads/2022/04/ros_logo.png" alt="ROS 2" width="100"/> | <img src="https://avatars.githubusercontent.com/u/91228115?s=280&v=4" alt="Isaac ROS" width="100"/> |



---


# 🚗 **Tutorial: Running Virtual QCar with Isaac ROS**  

This tutorial will guide you through setting up and running the **Virtual QCar** inside a **Docker container**, integrating it with **Isaac ROS**



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
<img src="Software_Setup/cont_2_qcar_isaac_running.png" alt="ROS 2" width="100"/>
---

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

---

## 📌 **3. Visualizing in RViz**  

To visualize the QCar2 data, run:  

```bash
ros2 run rviz2 rviz2
```

---

## 📌 **4. Controlling QCar2 via Keyboard or Python Script**  

Navigate to the QCar2 scripts directory:  

```bash
cd src/qcar2_nodes/scripts
```

You can send control commands using **ROS 2 topics**.  
For example, to set motor speed and steering angle, use:  

```bash
ros2 topic pub /qcar2_motor_speed_cmd qcar2_interfaces/msg/MotorCommands \
"{motor_names: ['steering_angle', 'motor_throttle'], values: [0.5, 0.7]}"
```

Alternatively, use a Python script to process images and display them:  

```python
import cv2

# Load an image or a frame
image = cv2.imread('/path/to/image.jpg')

# Show the image in a window
cv2.imshow('Window Name', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

---

## 📌 **5. Running AI Modules**  

### 🔹 **5.1. YOLO (Traffic Light Detection)**  
Use a **YOLO-based model** to detect traffic lights. You need a **pre-trained YOLO model** running on a Jetson device.  

### 🔹 **5.2. LaneNet (Lane Detection)**  
LaneNet can be integrated to detect road lanes, enabling autonomous lane-following.  

### 🔹 **5.3. Path Planning**  
Once object detection and lane detection are set up, you can implement **path planning algorithms** to navigate around obstacles.  

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

## 🎯 **Conclusion**  

This tutorial provides a **step-by-step guide to running Virtual QCar, integrating Isaac ROS, and using AI models**. You can expand it further by incorporating **SLAM, deep learning-based perception, and advanced motion planning**.  

Would you like a specific section explained in more detail? 🚀  
```

---

### 📌 **How to Use This File?**
1. Copy the content above.  
2. Save it as `virtual_qcar_tutorial.md`.  
3. Upload it to your **GitHub repository**.  

✅ **Your GitHub Markdown file is now ready!** 🚀
---

```md
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


---


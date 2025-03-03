
---

### 📄 **`Helper Notes when Testing ROS2 and Qcar2`**
```md
# How to Open Another Terminal in a Running Docker Container (ROS + Quanser QCar)

When running a ROS launch file inside a Docker container, you may need to open another terminal to check ROS topics, logs, or execute additional commands **without stopping the running nodes**.  

This guide explains two methods to open a new terminal inside a running Docker container.

---

## 🔹 Method 1: Using `docker exec` (Recommended)
This method allows you to open a **new interactive shell** inside the running container from your host machine.

### ✅ **Steps:**
1. **Find the running container name**  
   Run this command on your **host machine** to list running containers:
   ```bash
   docker ps
   ```
   Example output:
   ```
   CONTAINER ID   IMAGE           COMMAND       CREATED        STATUS        NAMES
   a1b2c3d4e5f6   qcar_docker     "/ros_entry"  10 minutes ago Up 10 mins    qcar_container
   ```
   Here, the container name is **`qcar_container`**.

2. **Open a new terminal inside the container**  
   Run:
   ```bash
   docker exec -it qcar_container bash
   ```
   This starts a new shell inside the running container.

3. **Run ROS commands**  
   Now, you can inspect topics inside the new shell:
   ```bash
   rostopic list
   ```
   Or check topic messages:
   ```bash
   rostopic echo /qcar/sensor_data
   ```

### 🚀 **Why Use This Method?**
✅ Doesn’t interfere with the running launch file  
✅ You can open multiple terminals as needed  
✅ Full shell access inside the container  

---

## 🔹 Method 2: Opening a New Bash Session Inside the Container
If you're **already inside the container** and need another shell session, use this method.

### ✅ **Steps:**
1. **If you're inside the container**, start a new Bash session by running:
   ```bash
   bash
   ```
   or
   ```bash
   /bin/bash
   ```
   This opens a **new shell session** inside the container.

2. **Run ROS commands**  
   Example:
   ```bash
   rostopic list
   ```
   or
   ```bash
   rostopic echo /qcar/sensor_data
   ```

### 🚀 **Why Use This Method?**
✅ Simple if you're already inside the container  
✅ No need to leave your current session  

---

## 🔹 Which Method Should You Use?
| Scenario                                      | Recommended Method |
|-----------------------------------------------|--------------------|
| Need a new terminal from **host machine**    | `docker exec` (Method 1) |
| Already inside the container, need another shell | `bash` (Method 2) |

Both methods allow you to **run ROS commands without stopping the main launch file**. 🎯


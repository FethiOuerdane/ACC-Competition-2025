Here’s a **ready-to-copy** Markdown (`.md`) file for **GitHub** documentation:  

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

## 🎯 Conclusion  
These **Docker commands** will help you efficiently manage your Virtual QCar container and other Docker-based applications.  

Feel free to **contribute** or **suggest new commands**! 🚀  
```

---

### 📌 **How to Use This File?**
1. Copy the content above.
2. Save it as `docker_commands.md`.
3. Upload it to your **GitHub repository**.

✅ **Your GitHub Markdown file is now ready!** 🎉

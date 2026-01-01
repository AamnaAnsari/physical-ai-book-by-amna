---
sidebar_position: 3
title: "Chapter 3: The Digital Twin"
sidebar_label: "3. Simulation (Gazebo/Unity)"
---

# Module 03: The Digital Twin

**Weeks 6-7**

### 🎯 Goal
Create a **physics-compliant virtual environment** where you can safely test and iterate on robot designs.  
> Before breaking a real robot, we break the simulated one!

### 📚 Key Topics (Detailed)

#### 1️⃣ Gazebo: Physics Simulation
- **Purpose:** Simulate real-world physics including gravity, friction, and collisions.  
- **Key Features:**
  - Rigid body dynamics for robot links  
  - Joint limits and torque constraints  
  - Realistic contact modeling for walking, grasping, or collisions  
- **Example:**  
  - A bipedal robot walking across uneven terrain  
  - Observe balance control algorithms in simulation  
- **Exercise:**  
  - Spawn a URDF robot in Gazebo  
  - Apply gravity and simulate simple motions  
  - Monitor joint angles and sensor feedback  

#### 2️⃣ Unity: High-Fidelity Rendering
- **Purpose:** Provide **realistic visualization** for interactions with robots.  
- **Key Features:**
  - Photorealistic lighting and shadows  
  - Texture mapping for objects and robot surfaces  
  - Integration with ROS 2 via ROS# or ROS-TCP Connector  
- **Exercise:**  
  - Import a robot model into Unity  
  - Add interactive objects (boxes, tables)  
  - Observe robot motion synced with physics simulation  

#### 3️⃣ Simulated Sensors
- **Purpose:** Test robot perception **before hardware deployment**  
- **Key Sensors:**
  - **LIDAR:** Point clouds for mapping and obstacle detection  
  - **Depth Cameras:** RGB-D for environment awareness  
  - **IMU:** Orientation and balance in virtual environment  
- **Integration:**  
  - Sensor data published on ROS 2 topics  
  - Virtual perception pipelines mimic real robot behavior  
- **Exercise:**  
  - Simulate a LIDAR scan in Gazebo  
  - Verify the robot can detect and avoid virtual obstacles  
  - Stream camera feed from Unity to ROS 2 node  

#### 4️⃣ Best Practices
- Always **test in simulation** before real hardware  
- Use **modular URDF/XACRO files** to simplify model changes  
- Log and visualize sensor data to debug algorithms  
- Compare results between Gazebo physics and Unity rendering to ensure consistency  

### 💡 Exercises
1. Spawn a robot in Gazebo and simulate a small walk cycle  
2. Test collision detection with static obstacles  
3. Stream camera feed to ROS 2 and process in a Python node  
4. Visualize simulated sensors in RViz2 and debug perception  

---


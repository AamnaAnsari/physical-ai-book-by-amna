---
sidebar_position: 7
title: "Chapter 7: Capstone Project"
sidebar_label: "7. 🎓 Capstone Project"
---

# Capstone: The Autonomous Humanoid

**Final Integration Project**

## 🎯 Goal
Congratulations! You have reached the final milestone. Your task is to build a complete **End-to-End Autonomous System** in simulation.

## 🛠️ Project Scope: "The Butler Bot"

Your simulated humanoid robot must perform the following sequence without human intervention:

1.  **Wait for Command:** The robot stands idle until it hears a voice command (e.g., "Bring me the red soda can").
2.  **Perceive:** Use the RGB-D camera to scan the room and identify the "red soda can" using computer vision (YOLO or Isaac perception).
3.  **Plan:** Use **Nav2** to generate a path to the table while avoiding the sofa and chairs.
4.  **Act:** Use **MoveIt 2** to plan the arm trajectory, grasp the can, and lift it.
5.  **Return:** Navigate back to the starting position.

## 📋 Submission Requirements
1.  **Source Code:** A GitHub repository containing your custom ROS 2 packages.
2.  **Video Demo:** A 2-minute screen recording of the simulation executing the task.
3.  **Documentation:** A `README.md` explaining your node graph.

:::warning Challenge
The environment will have **random obstacles**. Your Navigation stack must be dynamic enough to re-plan paths in real-time!
:::

## 🏆 Graduation
Upon completion, you will have successfully bridged the gap between Digital Brains and Physical Bodies. Welcome to the world of Physical AI!
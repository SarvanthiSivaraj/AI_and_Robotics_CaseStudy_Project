# AI_and_Robotics_CaseStudy_Project

## Overview

This project aims to develop and evolve a **bipedal robot** using **evolutionary algorithms** within the **Webots simulation environment**.
The robot learns to walk and balance through an iterative process inspired by natural evolution — involving simulation, selection, mutation, and improvement across generations.

Instead of hardcoding motion, the robot’s gait is discovered and optimized automatically through evolutionary learning.

---

## Project Phases

### **Phase 1 – Setup and Foundation**

* Defined the core **ideology** and project structure.
* Created a **base bipedal robot prototype** and an **evolutionary supervisor** framework.
* Established **fitness criteria** such as distance traveled and balance.
* Organized the controller and world files for modular development.

---

### **Phase 2 – World Creation**

* Selected a **base world file** from Cyberbotics and customized it to suit project needs.
* Modified terrain, removed unnecessary elements, and adjusted environmental parameters.
* Ensured the world supports **multiple robots and evolutionary simulations**.
* Resulted in a functional and adaptable simulation environment.

---

### **Phase 3 – World Re-Design and Implementation**

* Performed several iterations of **world re-design** for better testing conditions.
* Encountered coordinate alignment issues and created a **simpler rectangular arena** for accurate debugging.
* Started building the **bipedal robot from scratch**, using components like HingeJoint, Solid, and Physics nodes.
* Completed approximately **70% of the robot’s physical model** and began developing the **controller logic**.
* The redesigned world now supports efficient testing and robot placement.

---
Perfect — here’s your **Phase 4** and **Phase 5** written in the same clean, concise style as your earlier phases 👇

---

### **Phase 4 – Transition to DARwIn-OP3 and Controller Adaptation**

* Faced multiple mechanical and coordinate issues while building the robot from scratch, leading to unstable walking behavior.
* Switched to the **DARwIn-OP3** humanoid robot available in Webots for a more reliable and stable framework.
* Implemented custom Python controllers enabling the robot to **walk forward, backward, and turn** smoothly.
* Focus shifted from robot construction to refining **controller logic and walking behavior**.
* This transition allowed continued progress on **evolutionary learning** without being blocked by mechanical instabilities.

---

### **Phase 5 – Evolutionary Algorithm Integration**

* Implemented a **genotype-based evolutionary algorithm** to evolve the robot’s walking gait.
* Each genotype encodes **motor angles and timings** for one complete stride.
* The evolution process starts when **‘E’** is pressed, generating a population of genomes and evaluating them based on **distance, balance, and stability**.
* Performed **selection, mutation, and crossover** to evolve improved gait patterns over generations.
* Encountered challenges with fitness stability, mutation tuning, and inconsistent gait convergence.
* Current progress shows gradual improvement in **walking performance and stability**, forming the foundation for full self-evolving behavior.

---

## Tools and Technologies

* **Webots** – for 3D robotic simulation
* **Python** – for supervisor and controller programming
* **Evolutionary Algorithms** – for adaptive learning and optimization

---

## Expected Outcome

To create a **self-evolving bipedal robot** capable of learning efficient walking and balancing behaviors autonomously within a simulated environment.
<img width="414" height="375" alt="image" src="https://github.com/user-attachments/assets/890f3e02-ac30-4360-8884-1c1536f64294" />


#  Future Engineers - Robotics Project Documentation

<!-- NOTE: MUST PRINT HARD COPY AS WELL -->


Welcome to the official repository of **Team KMIDS-GFM**. We are participants in the Future Engineers category of the WRO competition.

## Team Members
- **Chayanon Ninyawee (Garfield)** 
- **Supakorn Sunthonthammarat (Pleum)** 
- **Thammatouch Chantasarn (Kao)** 

We are a team of dedicated students with a passion for robotics and innovation. This repository documents our full engineering process, including design, components used, development, testing, and coding of our robot.

<!-- NOTE: Replace all placeholder sections below with your team's actual content and details. -->

##  Table of Contents
- [1. About the Project](#1-about-the-project)
- [2. Mobility Management](#2-mobility-management)
- [3. Power and Sense Management](#3-power-and-sense-management)
- [4. Obstacle Management](#4-obstacle-management)
- [5. Images](#5-images)
- [6. Performance Videos](#6-performance-videos)
- [7. Source Code](#7-source-code)
- [8. Build Instructions](#8-build-instructions)
- [9. GitHub Usage and Commit History](#9-github-usage-and-commit-history)

---

## 1. About the Project


This project focuses on designing, building, and programming an autonomous robot capable of completing a series of complex obstacle challenges as part of the WRO Future Engineers competition. 

Team KMIDS-GFM was inspired by the challenge of applying engineering principles and problem-solving skills to creatively and efficiently solve complex problems. Pushing forward with our passion for innovation and hands-on learning.

Our goal is to design and build a reliable and efficient system that demonstrates our technical and collaborative skills while serving as a learning experience. We followed a systematic process, including brainstorming, researching, prototyping, testing, and iterating. We maintained detailed documentation for ease of knowledge sharing and a smoother workflow throughout the project.

The robot is engineered with a modular chassis, powered by an N20 motor with an encoder, and controlled using a Raspberry Pi 5 and a Raspberry Pi Pico 2. It utilises a combination of an RPLidar C1 LIDAR sensor and a fish-eye lens camera to provide an advanced system for obstacle detection and navigation.

Our objective was to create an intelligent robot that is capable of navigating through obstacles with pinpoint precision and speed. 


<!-- TODO: You should add a few paragraphs here about your inspiration, high-level goals, team structure, and overall approach. -->

---
## 2. Mobility Management

### 2.1 Motor Selection

**Motor Used:** N20 DC Motor with Encoder  


<!-- **Specifications:**
- Voltage: 
- RPM:
- Torque: -->

**Reason for Selection:**
- **Compact and lightweight**, ideal for small self-driving vehicles.
- **Integrated encoder** enables precise speed/distance tracking, essential for navigation.
- **Low power consumption**, suitable for embedded systems.
- **Moderate torque** for flat, indoor arenas.
- **Readily available** and widely supported.

**Mounting:**
- Installed using **3D-printed motor brackets** or **motor clamps**.
- Encoder wires routed neatly and connected to **Raspberry Pi Pico 2**.
- Rubber wheels are friction-fitted or screwed onto the motor shaft.

**Servo Used:** S0004
<!-- **Specifications:** -->

**Reason for Selection:**
- **Standard size and PWM interface** makes it easy to control via Raspberry Pi Pico 2.
- **Sufficient torque** to steer the front wheels responsively.
- **Balanced speed and stability** during turns and lane changes.
- **Widely used in hobby robotics**, with available documentation and mounting kits.

**Mounting:**
- Installed at the **front center of the chassis** using servo mounts or brackets.
- A servo horn is attached to the **pivoting front wheel or steering linkage**.
- The servo is screwed into a platform plate or chassis slot to prevent shifting during operation.

---

### 2.2 Chassis Design

**Design Overview:**
- Chassis is optimized for **symmetry** and **low center of gravity**.
- **Material:** Acrylic sheet or 3D-printed PLA/ABS.
- **Dimensions:** Approximately 20 cm × 15 cm (adjustable to rules).

**Component Mounting:**
- **SBC (Raspberry Pi 5)** mounted on **shock-absorbing standoffs**.
- **RPi Pico 2** and power regulators are mounted on the base with Velcro or screws.
- Servo and steering mechanism positioned at the front with rigid brackets.
- All cables are routed to avoid entanglement or interference with motion.

---

### 2.3 Mobility Principles

- **Drive System:** 2-wheel differential drive (rear wheels).
- **Steering:** Front-wheel steering using S0004 servo.
<!-- - **Gear Ratio:**  
- **Speed/Torque Tradeoff:**  -->
- **Feedback Control:** Uses encoder data for PID control to maintain trajectory.

---

### 2.4 Mounting and Structure

- All components secured with **custom 3D-printed mounts** or **screw brackets**.
- 3D designs available in **STL and STEP formats**.
- Top-down mounting with modular access for battery and components.
- Wire management maintained using **cable sleeves** and **clips**.

---

## 3. Power and Sense Management

### 3.1 Power Source

**Battery:**  
**Reason for Selection:**


---

### 3.2 Sensors Used

| Sensor                 | Purpose                          | Reason for Selection                              |
|------------------------|-----------------------------------|---------------------------------------------------|
| **RPLidar C1**         | 2D Lidar for obstacle detection   | Fast 360° scan, accurate, SLAM compatible         |
| **5MP Fish Eye Camera**| Lane and object detection         | Wide-angle view, IR-compatible, fast image output |
| **Motor Encoders**     | Speed and distance tracking       | Enables precise closed-loop control               |

**Integration:**
- Camera and Lidar connected to **RPi 5**.
- Encoders connected to **RPi Pico 2**.
<!-- - All sensors powered through regulated 5V output from UBECs. -->

---

### 3.3 Wiring and Bill of Materials (BOM)

**Wiring Summary:**
>

**Wiring Diagram:**
> *[Include labeled wiring diagram image here]*

---

**Bill of Materials:**

| Component               | Specs                          | Qty | Source/Supplier        |
|------------------------|---------------------------------|-----|------------------------|
| N20 DC Motor + Encoder |                                 | 2   |    |
| S0004 Servo            |                                 | 1   |    |
| Raspberry Pi Pico 2    | Microcontroller                 | 1   |    |
| Raspberry Pi 5         | SBC                             | 1   | |
| RPLidar C1             | 360° Lidar                      | 1   | SLAMTEC         |
| 5MP Fish Eye Camera    | Wide-angle, IR-capable          | 1   | Cytron     |
| Chassis Material       | PLA or Acrylic Sheet            | 1   | Local / 3D Printed     |
| 3D Printed Mounts      | STL files included              | N/A | User-designed          |
| Wires                  | Assorted lengths/connectors     | —   | Any electronics vendor |

---

### 📁 CAD & Design Files


- ` ` — Base chassis
- ` ` — Motor bracket
- ` ` — Fisheye camera mount
- ` ` — Front steering servo mount

All files will be uploaded under ` ` and ` ` folders. 


---

## 4. Obstacle Management

<!-- TODO: Obstacle management discussion should include the strategy for the vehicle to
negotiate the obstacle course for all the challenges. This could include flow
diagrams, pseudo code and source code with detailed comments. -->

### 4.1 Strategy Overview
<!-- TODO: Explain the logic and steps your robot takes to detect and navigate obstacles. -->

### 4.2 Flowcharts and Pseudo Code
<!-- TODO: Include diagrams or lists that represent logical flow for obstacle avoidance. -->

### 4.3 Source Code Summary
<!-- TODO: Describe key code modules related to obstacle management and explain them briefly. -->

---

## 5. Images

<!-- TODO: Pictures of the team and robot must be provided. The pictures of the robot must
cover all sides of the robot, must be clear, in focus and show aspects of the
mobility, power and sense, and obstacle management. Reference in the
discussion sections 1, 2 and 3 can be made to these pictures. Team photo is
necessary for judges to relate and identify the team during the local and
international competitions. -->

### 5.1 Robot Images
<!-- TODO: Include clear, high-quality photos from top, bottom, front, back, left, and right. -->

### 5.2 Internal View
<!-- TODO: Show images of internal layout (wiring, board placement, sensors, motors). -->

### 5.3 Team Photo
<!-- TODO: Add a team photo with names and roles of each team member. -->

---

## 6. Performance Videos

<!-- TODO: The performance videos must demonstrate the performance of the vehicle from
start to finish for each challenge. The videos could include an overlay of
commentary, titles or animations. The video could also include aspects of
section 1, 2 or 3. -->

### 6.1 Challenge 1 - Open Challenge
[Watch on YouTube](#) <!-- TODO: Replace with actual video link -->

### 6.2 Challenge 2 - Obstacle Challenge
[Watch on YouTube](#) <!-- TODO: Replace with actual video link -->

<!-- NOTE: Videos must be at least 30 seconds of continuous autonomous run. You may add overlays and labels. -->

---

## 7. Source Code

### 7.1 Code Structure
<!-- TODO: Explain how your codebase is organized (e.g., folders for different challenges or components). -->

<!-- ### 7.2 Main Code Modules
- Motor Control
- Sensor Input & Processing
- Obstacle Navigation
- Serial Communication (e.g., with camera or sensors)
- Setup and Initialization

### 7.3 Compilation/Upload Instructions
NOTE: List any software/IDE needed (e.g., Arduino IDE), libraries required, and steps to upload to microcontroller. -->

---

## 8. Build Instructions

### 8.1 3D Printed Parts
<!-- TODO: Mention parts that are 3D printed. Include .STL files in the repo. -->

### 8.2 Tools and Assembly
<!-- TODO: Provide brief assembly instructions or a link to a full manual. -->

---

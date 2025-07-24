# Hello! We are **Team KMIDS-GFM**.


<p align="center">
  pic
</p>



# WRO - Future Engineers - Robotics Project Documentation


## Team Members
- **Chayanon Ninyawee (Garfield)** 
- **Supakorn Sunthonthammarat (Pleum)** 
- **Thammatouch Chantasarn (Kao)** 

We are a team of dedicated students with a passion for robotics and innovation. This repository documents our full engineering process, including design, components used, development, testing, and coding of our robot.

<!-- NOTE: Replace all placeholder sections below with your team's actual content and details. -->

##  Table of Contents
- [1. About the Project](#1-about-the-project)
- [2. Mobility Management](#2-mobility-management)
  - [Powertrain](#21-powertrain)
    - [Motor](#motor)
  - [Steering](#22-steering)
    - [Servo](#servo)
  - [Chassis Design](#23-chassis-design)
  - [Mounting and Structure](#24-mounting-and-structure)
- [3. Power and Sense Management](#3-power-and-sense-management)
  - [Power Source](#31-power-source)
  - [Sensor and Camera](#32-sensor-and-camera)
    - [LIDAR Sensor](#lidar-sensor)
    - [Fish Eye Lens Camera](#fish-eye-lens-camera)
  - [Processing-Unit](#33-processing-units)
    - [Microcontroller](#microcontroller)
    - [Single-Board Computer](#single-board-computer)
  - [Circuit Diagram](#34-circuit-diagram)
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

Mobility Principles

- **Drive System:** 2-wheel differential drive (rear wheels).
- **Steering:** Front-wheel steering using S0004 servo.
<!-- - **Gear Ratio:**  
- **Speed/Torque Tradeoff:**  -->
- **Feedback Control:** Uses encoder data for PID control to maintain trajectory.

### 2.1 Powertrain


### Motor

Motor: N20 Motor w/ Encoder
<!-- link here -->
<table>
  <tr>
    <td align="center" width="300" >
      <img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/motor.webp" alt="INSERT MOTOR" >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Voltage: </li>
        <li>No-load Speed: </li>
        <li>No-load Speed (RPM)</li>
        <li>Stall Torque 0.3kg-cm</li>
        <li>Current Draw (no-load & stall)</li>
        <li>Gear Ratio: 30:1</li>
      </ul>
    </td>
  </tr>
</table>

**Reason for Selection:**
- **Compact and lightweight**, ideal for small self-driving vehicles.
- **Integrated encoder** enables precise speed/distance tracking, essential for navigation.
- **Low power consumption**, suitable for embedded systems.
- **Moderate torque** for flat, indoor arenas.
- **Readily available** and widely supported.

**Mounting:**
- Installed using **3D-printed motor clamps** screwed to a detatchable motor plate.
- Encoder wires routed neatly and connected to **Raspberry Pi Pico 2**.
- Rubber wheels are friction-fitted or screwed onto the motor shaft.

### 2.2 Steering

### S004 Servo 
<!-- link here -->
<table>
  <tr>
    <td align="center" width="300" >
      <img src="" alt="INSERT SERVO" >
    </td>
    <td> 
      <h3>Specifications:</h3>
      <ul>
          <li>Torque Rating: </li>
          <li>Speed: sec/60°)</li>
          <li>Voltage: </li>
          <li>Control Signal Type: (e.g. PWM)</li>
          <li>Rotation Angle: </li>
      </ul>
    </td>
  </tr>
</table>

### Servo
**Servo Used:** S004
**Specifications:**

**Reason for Selection:**
- **Standard size and PWM interface** makes it easy to control via Raspberry Pi Pico 2.
- **Sufficient torque** to steer the front wheels responsively.
- **Balanced speed and stability** during turns and lane changes.
- **Widely used in hobby robotics**, with available documentation and mounting kits.

**Mounting:**
- Screwed directly into the **front of the chassis** .
- A servo horn is attached to the **pivoting front wheel or steering linkage**.
- The servo is screwed into a platform plate or chassis slot to prevent shifting during operation.
- Uses Ackermann steering geometry similar to the mechanism found in real-world cars. Compared to differential drive robots (which steer by varying the speed of wheels), Ackermann steering turns the front wheels using the servo, while the rear wheel provides the drive force.

This approach allows the robot to:

Turn smoothly with minimal skidding.

Follow curved paths more realistically.

Navigate tight corners with better control at high speeds

---

### 2.3 Chassis Design

### Design Overview:

**Principle**


**Design**

Our robot chassis was completely custom-designed and 3D printed using [placeholder] material, that gives us a detailed and precise model we can tailor to our needs such as allowing our components to fit together seamlessly, while the plastic allows the robot to be both lightweight and durable. The chassis is also designed with modularity in mind for additional future components and fixes.

Material: 

Dimensions:

Design Philosophy: Compact, modular, precision

CAD Software Used: 

dedicated mounting points for motors, servo, camera, LIDAR, and battery compartment. making sure to have reduced overhangs

Similar to the geometry found in cars, the chassis layout follows a rear-drive layout using differential motors (N20 w/ encoders) and front Ackermann steering using an S004 servo. This allows the robot to make curved turns with reduced slippage and increased stability, especially important for tight navigation tasks.




### 2.4 Mounting and Structure

- Uses a differential geartrain
- The servo is installed at the front center of the chassis using **custom printed servo mounts**, typically included with robotics chassis kits.
- A **custom printed servo horn** is attached to a pivoting front wheel or steering linkage to control direction.
- The servo is securely screwed into a platform plate.

---

## 3. Power and Sense Management

### 3.1 Power Source

### [18650 Lithium Ion Battery](https://th.cytron.io/p-3.7v-2000mah-li-ion-battery))

<table>
  <tr>
    <td align="center" width="300" >
      <img src="" alt="INSERT Battery" width = 100% >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
      </ul>
    </td>
  </tr>
</table>

### 3.2 Sensor and Camera

### LIDAR Sensor

### [RPLIDAR C1](https://www.slamtec.com/en/C1)

<table>
  <tr>
    <td align="center" width="300" >
      <img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/LIDAR.jpg" alt="INSERT LIDAR" width = 100% >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Distance Range:
          <ul>
            <li>White: 0.05~12m (70% Reflectivity)</li>
            <li>Black: 0.05~6m (10% Reflectivity)</li>
          </ul>
        </li>
        <li>Angle Resolution: 0.72°</li>
        <li>Sample Frequency: 8~12Hz (10Hz Typical)</li>
        <li>Range Accuracy: 15mm</li>
        <li>Discharge Rate: 30C</li>
        <br/><br/>
      </ul>
    </td>
  </tr>
</table>

**Reason for Selection:**
- **Precise distance measurement** of obstacles within a wide radius
- **360° scanning** makes it ideal for SLAM (Simultaneous Localization and Mapping).
- **Compact size and lightweightness** allows easy fitting on our robot.
- **Fast sampling rate** allows real-time mapping and obstacle avoidance.

**Additional info**
- custom made sensor driver 

### Fish Eye Lens Camera

### [Fish Eye Lens Raspberry Pi 5MP IR Camera](https://th.cytron.io/p-fish-eye-lense-raspberry-pi-5mp-ir-camera?r=1&language=en-gb&gad_campaignid=18809653822)

<table >
  <tr>
    <td align="center" width="300" >
      <img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/RaspberryPI-FE-5MPIRCAM.jpg" alt="INSERT Camera" width = 100% >
    </td>
    <td >
      <h3>Specifications:</h3>
      <ul>
        <li>Voltage: 6V</li>
        <li>Resolution: 5MP </li>
        <li>Focal Length: Adjustable</li>
        <li>Image Sensor: OV5647</li>
        <li>Aperture: F2.35</li>
      </ul>
    </td>
  </tr>
</table>

**Reason for Selection:**
- **130° wide field of view** captures a large area for tracking.
- **Infrared compatibility** enables low-light vision.
- **High-resolution (5MP)** provides clear image for the robot.
- **Compact size** fits well on our robot.


**Integration:**
- Camera and Lidar connected to **Raspberry Pi 5**.


### 3.3 Processing Units

### Microcontroller

### [Raspberry Pi 5](https://gammaco.com/gammaco/Raspberry_Pi_GB_89RD014.html)

<table>
  <tr>
    <td align="center" width="300" >
      <img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/RP5.webp" alt="INSERT RP5" >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Quad-core Arm Cortex-A76 CPU @ 2.4GHz</li>
        <li>8GB LPDDR4X RAM</li>
        <li>Dual 4K HDMI output (60fps)</li>
        <li>PCIe 2.0 support via FPC connector</li>
        <li>2x USB 3.0 ports, 2x USB 2.0 ports</li>
      </ul>
    </td>
  </tr>
</table>

### Single-Board Computer
### [Raspberry Pi Pico 2](https://th.cytron.io/p-raspberry-pi-pico2-board)

<table>
  <tr>
    <td align="center" width="300" >
      <img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/RPP2.webp" alt="INSERT RPP2" >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Dual-core Arm Cortex-M0+</li>
        <li>2MB flash memory</li>
        <li>264KB SRAM</li>
        <li>Built-in Wi-Fi (802.11n)</li>
        <li>26 multi-function GPIO pins</li>
      </ul>
    </td>
  </tr>
</table>



### 3.4 Circuit Diagram

**Wiring Summary:**
>

**Wiring Diagram:**
> *[Include labeled wiring diagram image here]*

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



### 8.1 3D Printed Parts
<!-- TODO: Mention parts that are 3D printed. Include .STL files in the repo. -->

### 8.2 Tools and Assembly
<!-- TODO: Provide brief assembly instructions or a link to a full manual. -->

---

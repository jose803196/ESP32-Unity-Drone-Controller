<div align="center">
  <h1>Project: DroneLink Simulator</h1>
  <p>
    <strong>A real-time 3D drone simulator controlled by a custom-built physical interface based on the ESP32 microcontroller.</strong>
  </p>
  <br>
  <p>
    <a href="#-about-the-project">About</a> ·
    <a href="#-features">Features</a> ·
    <a href="#-tech-stack">Tech Stack</a> ·
    <a href="#-getting-started">Getting Started</a> ·
    <a href="#-repository-structure">Structure</a> ·
    <a href="#-author">Author</a>
  </p>
</div>

---

## 📋 About The Project

**DroneLink Simulator** is a comprehensive hardware-in-the-loop project that bridges the gap between physical electronics and virtual simulation. The core idea is to provide an immersive and realistic control experience by flying a 3D drone in a Unity environment using a dedicated hardware controller.

This repository contains everything from the ground up:
*   **Hardware Design:** Custom schematics and PCB layouts for the controller.
*   **Embedded Firmware:** Robust C code for the ESP32 to read sensors and communicate data.
*   **3D Simulation:** A fully functional Unity project that receives and interprets the hardware data to manipulate the drone.
*   **Complete Documentation:** In-depth reports explaining the design and implementation process.

<br>

## ✨ Features

*   **Real-time Control:** Low-latency communication between the ESP32 and Unity via UART.
*   **Intuitive Physical Interface:** Control using a joystick for movement, buttons for altitude and rotation.
*   **Realistic Physics:** The drone in Unity is powered by a `Rigidbody` component for believable movement.
*   **Modular Codebase:** Clean and well-documented code in both C (ESP32) and C# (Unity).
*   **Integrated 3D Workflow:** Includes `.blend` files directly within the Unity project for seamless model editing.
*   **Open Source:** Fully documented and ready to be forked, modified, and improved.

<br>

## 🛠️ Tech Stack

This project is built with a combination of hardware and software technologies:

<table>
  <tr>
    <td align="center"><strong>Hardware</strong></td>
    <td align="center"><strong>Firmware</strong></td>
    <td align="center"><strong>Simulation</strong></td>
    <td align="center"><strong>Documentation</strong></td>
  </tr>
  <tr>
    <td>
      <ul>
        <li>ESP32 DevKitC</li>
        <li>MPU-6050 Gyro/Accel</li>
        <li>Analog Joystick</li>
        <li>Push Buttons</li>
      </ul>
    </td>
    <td>
      <ul>
        <li>C Language</li>
        <li>ESP-IDF Framework</li>
        <li>FreeRTOS</li>
        <li>UART Communication</li>
      </ul>
    </td>
    <td>
      <ul>
        <li>Unity Engine</li>
        <li>C# Language</li>
        <li>Blender (for 3D models)</li>
      </ul>
    </td>
    <td>
      <ul>
        <li>LaTeX</li>
        <li>Markdown</li>
        <li>KiCad (for Schematics/PCB)</li>
      </ul>
    </td>
  </tr>
</table>

<br>

## 🚀 Getting Started

Follow these steps to get the project up and running on your local machine.

### Prerequisites

*   **Hardware:** All components listed in `1_Hardware/BillOfMaterials.md`.
*   **Software:**
    *   Unity Hub & Unity Editor (version `202X.X.Xf1` recommended).
    *   ESP-IDF (version `vX.X` recommended).
    *   A LaTeX distribution like MiKTeX (Windows) or MacTeX (macOS) to compile the documentation.

### Installation & Setup

1.  **Clone the Repository**
    ```sh
    git clone https://github.com/jose803196/ESP32-Unity-Drone-Controller
    ```

2.  **Flash the Firmware**
    *   Navigate to the `2_Firmware_ESP32/` directory.
    *   Connect your assembled controller to your computer.
    *   Run `idf.py flash monitor` to build, upload, and view the serial output.

3.  **Run the Unity Simulation**
    *   Open Unity Hub and select "Add project from disk".
    *   Choose the `3_Unity_Project/` folder from this repository.
    *   Once the project is open, navigate to the main scene in `Assets/_Project_DroneSim/Scenes/`.
    *   In the `DroneController` script component, set the `Port Name` to match the one used by your ESP32.
    *   Press **Play**!

<br>

## 📂 Repository Structure

The repository is organized into four main sections for clarity and modularity:


<br>

## 👤 Author

**Jose G. Lopez B.**

*   <a href="https://github.com/jose803196">
      <img src="https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=github&logoColor=white" alt="GitHub"/>
    </a>
*   <a href="https://www.linkedin.com/in/jose803196/">
      <img src="https://img.shields.io/badge/LinkedIn-0A66C2?style=for-the-badge&logo=linkedin&logoColor=white" alt="LinkedIn"/>
    </a>

---
<div align="center">
  <small>This project was created as part of Project Laboratory.</small>
</div>
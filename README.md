<div align="center">
<h1 align="center">Room Mapping System</h1>

  <p align="center">
    A microcontroller-based system that maps a room using a time-of-flight sensor and a stepper motor.
    <br />
  </p>
</div>

<p align="center">
  <img src="https://github.com/user-attachments/assets/2d531bdb-83e0-4dec-8bb3-2016026bb219" width="700"/>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/d2b1510b-a544-4a6e-9241-1ee22cf27fc2" width="700"/>
</p>




## Table of Contents

<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#key-features">Key Features</a></li>
      </ul>
    </li>
    <li><a href="#built-with">Built With</a></li>
    <li><a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

## About The Project

The Room Mapping System is an embedded project designed to scan and map the surroundings of a room. It uses a VL53L1X time-of-flight sensor interfaced with a TM4C1294NCPDT microcontroller to measure distances. A stepper motor rotates the sensor, allowing it to capture distance measurements at various angles, effectively creating a 2D "radar" scan of the room. The collected data is then transmitted via UART to a computer for processing and visualization using Python.

### Key Features

*   **Time-of-Flight Distance Measurement:** Utilizes the VL53L1X sensor for accurate distance readings.
*   **Automated Scanning:** Employs a stepper motor to rotate the sensor and perform a 360-degree scan.
*   **Data Acquisition:** Collects distance measurements at defined angular increments.
*   **UART Communication:** Transmits the collected data to a computer via UART.
*   **Data Processing and Visualization:** Python scripts process the data and generate a 3D point cloud visualization using the Open3D library.
*   **Button Controlled Operation:** Onboard buttons allow for easy start/stop of motor rotation and data acquisition.
*   **LED Indicators:** Onboard LEDs provide visual feedback on the system's status, including UART transmission, measurement status, and troubleshooting.

## Built With

*   [C](https://en.wikipedia.org/wiki/C_(programming_language))
*   [Python](https://www.python.org/)
*   [Keil uVision IDE](https://www.keil.com/product/uvision/)
*   [Texas Instruments TM4C1294NCPDT Microcontroller](https://www.ti.com/product/MSP432E401Y)
*   [VL53L1X Time-of-Flight Sensor](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html)
*   [Open3D](http://www.open3d.org/)

## Getting Started

To get a local copy up and running, follow these steps.

### Prerequisites

*   **Keil uVision IDE:**  Required for building and flashing the microcontroller firmware.
    *   Download: [https://www.keil.com/](https://www.keil.com/)
*   **Python 3.6-3.9:** Required for running the data processing and visualization scripts, with Open#D only functional on the listed versions.
    *   Download: [https://www.python.org/downloads/](https://www.python.org/downloads/)
*   **Numpy:** Required for converting point data into array formats compatible with Open3D.
    ```sh
    pip install numpy
    ```
*   **Open3D:** Required for 3D point cloud visualization in Python.
    ```sh
    pip install open3d
    ```
*   **PySerial:** Required for serial communication with the microcontroller.
    ```sh
    pip install pyserial
    ```
### Keil Setup
1.   **Manage Run-Time Environment:**
      *   Make sure that the device setup and CMSIS CORE are both checked off
   <p align="left">
    <img src="https://github.com/user-attachments/assets/1e2f4f46-ec31-4d3f-ada5-3d021144afdd" width="450"/>
  </p>
  
2.   **Options for Target - Target:**
      *   Make sure ARM compiler is default
   <p align="left">
    <img src="https://github.com/user-attachments/assets/bc5e6b1c-c0a9-4326-a473-dcae57d4bf0b" width="450"/>
  </p>

3.   **Options for Target - C/C++:**
      *   Make sure optimization is set to -O1
   <p align="left">
    <img src="https://github.com/user-attachments/assets/f85507fc-3d5f-4f6b-b8f6-6c620bf4acc9" width="450"/>
  </p>
  
4. **Options for Target - Debug:**
      *   Make sure to use CMSIS-DAP Debugger
   <p align="left">
    <img src="https://github.com/user-attachments/assets/65555c6e-8c4f-4ae9-b183-e258e6ebe67d" width="450"/>
  </p>

5. **Options for Target - Debug: Use Settings**
      *   Select 'XDS110 with CMSIS-DAP'
   <p align="left">
    <img src="https://github.com/user-attachments/assets/5ce6438e-8a0a-4484-8eb6-122b477643ee" width="450"/>
  </p>

      


### Installation

1.  **Clone the repository:**
    ```sh
    git clone https://github.com/huzai4a/room-mapping-system.git
    cd room-mapping-system
    ```
2.  **Translate/Build the firmware:**
    *   Open the `2dx_studio_8c.uvprojx` project in Keil uVision.
    *   Translate/Build the project to generate the executable.
3.  **Flash the microcontroller:**
    *   Connect the TM4C1294NCPDT microcontroller to your computer using a JTAG debugger.
    *   Flash the generated executable to the microcontroller using Keil uVision.
    <p align="left">
            <img src="https://github.com/user-attachments/assets/1bf9b42f-b2d2-43fe-8339-37d802d539ac" width="450"/>
    </p>
4.  **Install Python dependencies:**
    ```sh
    cd py
    pip install numpy open3d pyserial
    ```
5.  **Run the data acquisition and visualization script:**
    *   Ensure the correct COM port is set in `Project_Deliverable.py`.
    ```python
    with serial.Serial('COM5', 115200, timeout=2) as s: # Replace COM5 with your port
    ```
    *   Execute the Python script:
    ```sh
    python Project_Deliverable.py
    ```
6.  **Scanning can now be completed!**
    *  Refer to the datasheet for any further clarification


## Acknowledgments

*   Base code provided by Valvano, with modifications by course instructors/assistants Tom Doyle and Hafez Mousavi Garmaroudi for Time of Flight sensor application

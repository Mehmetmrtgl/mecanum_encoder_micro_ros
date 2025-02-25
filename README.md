# Mecanum Encoder Micro-ROS with ROS 2 on Jetson Orin

This project is designed to read data from four encoders connected to an ESP32 microcontroller using micro-ROS and publish the data to a ROS 2 system running on a Jetson Orin.

## 📦 Project Structure
- **ESP32:** Reads encoder data and publishes it to ROS 2.
- **Jetson Orin:** Runs ROS 2 and subscribes to encoder topics.
- **PlatformIO:** Used to develop and upload firmware to the ESP32.

## 🛠 Required Software

### For Jetson Orin (ROS 2)
1. Update and upgrade your system:
    ```bash
    sudo apt update
    sudo apt upgrade -y
    ```
2. Install ROS 2 (Humble or latest LTS version):
    ```bash
    sudo apt install ros-humble-desktop
    ```
3. Source ROS 2 in your shell:
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

### For ESP32 (Micro-ROS)
1. Install PlatformIO:
    ```bash
    sudo apt install git python3 python3-pip
    pip3 install platformio
    ```
2. Clone the project repository:
    ```bash
    git clone https://github.com/YourUsername/mecanum_encoder_micro_ros.git
    cd mecanum_encoder_micro_ros
    ```

### Install Micro-ROS Agent on Jetson Orin
1. Install dependencies:
    ```bash
    sudo apt install libasio-dev libtinyxml2-dev
    ```
2. Clone and build the micro-ROS agent:
    ```bash
    git clone -b humble https://github.com/micro-ROS/micro_ros_agent.git
    cd micro_ros_agent
    colcon build
    source install/setup.bash
    ```

## ⚙️ Hardware Setup

| Encoder | A Channel Pin | B Channel Pin |
|---------|--------------|--------------|
| 1       | 13           | 12           |
| 2       | 2            | 15           |
| 3       | 26           | 27           |
| 4       | 18           | 19           |

## 🚀 Running the System

### 1. Flash the ESP32
```bash
pio run --target upload
```

### 2. Start the Micro-ROS Agent on Jetson Orin
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```

### 3. Monitor Encoder Data in ROS 2
```bash
ros2 topic echo /encoder_ticks
```

## ❗️ Important Notes
- Ensure the ESP32 is properly connected to Jetson Orin via a USB cable.
- Set the baud rate to `921600` for serial communication.
- Make sure to use the correct `/dev/ttyUSBX` device.

## 🧩 Troubleshooting
- If no data is received, verify that the Micro-ROS Agent is running.
- Ensure the encoder wiring and pins are correct.

## 💡 Contribution
Feel free to fork this repository, create a new branch, and submit a pull request with your improvements.


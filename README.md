# micro-ROS Encoder & RPM Publisher for ESP32

This project is a micro-ROS application running on ESP32. It reads data from four encoders, calculates both raw encoder counts and motor RPM (revolutions per minute), and publishes them to ROS 2.

## Features
- **Supports Four Encoders**: The ESP32 reads output from four different encoders and sends the data to ROS 2.
- **RPM Calculation**: Encoder data is processed to calculate RPM values and published to ROS 2.
- **micro-ROS Integration**: Developed using PlatformIO and micro-ROS.

## Hardware Requirements
- ESP32 Development Board
- 4 Encoders
- A computer with ROS 2 installed (Jetson Orin or another Linux machine)

## Installation
1. **Set Up micro-ROS with PlatformIO**
   - Ensure PlatformIO and micro-ROS are installed.
   - Configure your `platformio.ini` file accordingly.

2. **Upload the Code to ESP32**
   ```sh
   pio run --target upload
   ```

3. **Start Serial Communication with micro-ROS**
   - Connect ESP32 to your computer and run the following command to start the micro-ROS serial agent:
   ```sh
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
   ```

4. **Monitor ROS 2 Messages**
   - To view encoder data:
   ```sh
   ros2 topic echo /encoder_ticks
   ```
   - To view RPM data:
   ```sh
   ros2 topic echo /rpm_data
   ```

## Code Explanation
- **Reading Encoders**: The A-phase pulses from encoders are counted using interrupts, and the B-phase determines the direction.
- **RPM Calculation**: Encoder data is processed at intervals to compute RPM values.
- **Publishing to micro-ROS**: ESP32 sends `encoder_ticks` and `rpm_data` messages to the ROS 2 network.

## ROS 2 Message Types Used
- `std_msgs/msg/Int32MultiArray`: Used for encoder counts.
- `std_msgs/msg/Float32MultiArray`: Used for RPM data.




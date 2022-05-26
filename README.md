## Repository for Upcoming Course

- **robot_driving_src** : Contain all source Codes to run on RPI
- **slamBot** : Package Develop for ROS2 Rpi

#### Running Code
- **Interfacing Code for RPI**
    - RpLidar ROS Package
    ```
    - sudo apt-get install ros-foxy-rplidar-ros
    ``` 
    -  Install required PIGPIO Library
    ```
    pip install pigpio
    sudo apt-get install libpigpiod-if2-1
    ```
    - Build From Source ( a must )
    ```
    git clone https://github.com/joan2937/pigpio.git
    cd pigpio
    make
    sudo make install
    ```
    - Example of compiling a CPP file
    ```
    g++ -Wall -pthread -o test hello_blink.cpp -lpigpiod_if2
    ```

- **ROS Package**
    - Creating a Workspace
    ```
    mkdir -p ~/ros2_ws/src
    ```
    - Cloning Repository into ~/ros2_ws/src
    ```
    cd ~/ && git clone  https://github.com/noshluk2/ROS2-SlamBot-using-RaspberryPI
    ```
    - Sourcing Workspace ( Only ONCE )
    ```
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    ```


## Requirments

- ### Software and Dependencies
  - Ubuntu Server 20.04
  - Ros2 Foxy-base installation
  - Turtlesim Package
    ```
    sudo apt-get install ros-foxy-turtlesim
    ```
- ### Hardware
  - Raspberry pi 4
  - Power Bank
  - 16gb Sdcard
  - L298D motor Driver
  - 12V DC magnetic Encoder Motors
  - 12V lipo battery
  - Srews ,pcb offsets and male to female/Male jupmer wires

## Using This repository
- Connect your RPI with Wifi -> [Video link](https://www.youtube.com/watch?v=s4ZDlV3tIuM&t=507s&ab_channel=RaspberryTips)

### Running a SSH control through terminal ( shell only )
  - Open terminal on your laptop
    - IP of raspberry pi -> get from any wifi conencted device info app
    - passwork is 'ubuntu'
  ```
  ssh ubuntu@pi_IP
  ```
  - Lidar Scan generation -> fake transform
  ```
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 "world" "laser"
  ```
  - Stopping Rplidar Scanning
  ```
  
  ```

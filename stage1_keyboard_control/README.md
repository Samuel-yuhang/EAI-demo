# Stage 1: Keyboard Control

This stage details the complete process for setting up the Raspberry Pi from scratch, installing ROS 2, and running the nodes required for basic keyboard-based remote control of the car.

## Objective

-   Install Ubuntu Server and ROS 2 Humble on the Raspberry Pi.
-   Establish a wireless SSH connection to the Raspberry Pi for remote development.
-   Create a ROS 2 node on the Pi to listen for velocity commands and control the motors.
-   Run a standard ROS 2 node on a host PC to capture keyboard input and send control commands.

---

### 1. Flash OS and Initial Setup

The first step is to prepare the operating system on a TF card.

1.  **Download Imager:** Go to the official Raspberry Pi website at [https://www.raspberrypi.com/software/](https://www.raspberrypi.com/software/) and download the Raspberry Pi Imager for your computer's OS.
2.  **Choose OS:** Open the imager and select the following operating system: `Other general-purpose OS` -> `Ubuntu` -> `Ubuntu Server 22.04.4 LTS (64-bit)`.
3.  **Configure:** Before writing, click the settings (gear icon) button to pre-configure the OS:
    -   **Set Hostname:** You can set a custom hostname, or leave it as `raspberrypi`.
    -   **Enable SSH:** In the "Services" tab, check the box to "Enable SSH" and use password authentication.
    -   **Set Username/Password:** Create a username and a secure password.
    -   **Configure Wireless LAN:** Input the SSID (name) and password for your mobile phone's hotspot. **It is recommended to use a simple name and password with no special characters to avoid connection issues.**
4.  **Write to TF Card:** Select your TF card and click the "WRITE" button.

---

### 2. First Boot and Remote Connection

1.  **Power On:** Once the writing process is complete, eject the TF card, insert it into the Raspberry Pi, and connect the power. Make sure your mobile hotspot is turned on.
2.  **Connect PC:** Connect your personal computer to the same mobile hotspot.
3.  **Find IP Address (Optional):** The Pi should automatically connect to the hotspot. You can find its IP address using a network scanning tool on your phone (like Fing) or by checking the list of connected devices in your phone's hotspot settings.
4.  **Connect via SSH:** Open a terminal (or use the integrated terminal in VS Code) and connect to the Raspberry Pi using one of the following commands:
    -   `ssh your_username@raspberrypi.local` (If your network supports mDNS)
    -   `ssh your_username@<ip_address_of_pi>` (Using the IP address you found)
    -   You will be prompted for the password you set during the imaging process.

---

### 3. Install ROS 2 Humble on Raspberry Pi

Once connected to the Raspberry Pi via SSH, execute the following series of commands to install ROS 2 Humble. **This process is time-consuming and requires a stable internet connection.**

```bash
# Update package sources and install required tools
sudo apt update
sudo apt install software-properties-common
# You will be prompted to press Enter after this command
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

# The following command may remove the "needrestart" package to prevent interruptions during installation
sudo apt-get purge needrestart

# (Optional) Use a host mapping to speed up GitHub access
sudo sh -c 'sed -i "/# GitHub520 Host Start/Q" /etc/hosts && curl [https://raw.hellogithub.com/hosts](https://raw.hellogithub.com/hosts) >> /etc/hosts'

# Add the ROS 2 repository key
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list and install development tools
# You will be asked to continue, press 'Y' then Enter
sudo apt update && sudo apt install ros-dev-tools

# Update and upgrade all packages
sudo apt update
sudo apt upgrade

# Finally, install ROS 2 Humble Desktop
# WARNING: This command can take around 40 minutes and download ~1.5 GB of data.
sudo apt install ros-humble-desktop
```

---

### 4. Create the Motor Control Node

Now, create a ROS 2 workspace and the package that will contain our motor control code.

1.  **Create Workspace:**
    ```bash
    mkdir -p ~/FIRSTPROJECT/src
    cd ~/FIRSTPROJECT/src
    ```

2.  **Create Package:**
    ```bash
    ros2 pkg create wheels --build-type ament_python --dependencies rclpy
    ```

3.  **Create Python Script:**
    -   Navigate into the `wheels` package: `cd wheels/wheels`
    -   Create the motor control script: `touch motor_control.py`
    -   Open this new file with a text editor (like `nano` or `vim`) and paste the code from this repository's `/stage1_keyboard_control/src/motor_control.py` file into it. Save and exit.

4.  **Install Dependencies:**
    ```bash
    sudo apt install python3-gpiozero python3-pigpio
    ```

5.  **Configure `setup.py`:**
    -   Open the `setup.py` file located in the `~/FIRSTPROJECT/src/wheels` directory.
    -   Find the `entry_points` dictionary and add the following `console_scripts` entry to register your node as an executable:
        ```python
        entry_points={
            'console_scripts': [
                "motor_control = wheels.motor_control:main"
            ],
        },
        ```

---

### 5. Build and Run the Project

With the code and configuration in place, you can now build and run the nodes.

1.  **Build the Workspace:**
    -   Navigate back to the root of your workspace: `cd ~/FIRSTPROJECT`
    -   Run the build command:
        ```bash
        colcon build --symlink-install
        ```

2.  **Run the Nodes:**
    You will need **two separate terminals** connected to your network.

    -   **IN TERMINAL 1 (SSH'd into Raspberry Pi):**
        ```bash
        
        # Navigate to your workspace
        cd ~/FIRSTPROJECT
        
        # Source your workspace's environment
        source install/setup.bash
        
        # Run your motor control node
        ros2 run wheels motor_control
        ```
        You should see a message indicating the node has started and is waiting for commands.

    -   **IN TERMINAL 2 (On your Host PC):**
        ```bash
        # Run the keyboard teleop node
        ros2 run teleop_twist_keyboard teleop_twist_keyboard
        ```
        Now, with this terminal window active, you can use the keyboard keys (`i`, `j`, `l`, etc.) to send commands and control your car.

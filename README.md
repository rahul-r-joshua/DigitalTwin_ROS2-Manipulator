# 🤖 DigitalTwin ROS2 + Manipulator

A complete ROS 2 framework for controlling a robotic manipulator with Arduino hardware integration, featuring FK/IK control modes and MoveIt planning capabilities.

## 🦾 Project Overview
This project integrates a robotic manipulator with ROS 2, using an Arduino for hardware communication. It provides both Forward Kinematics (FK) and Inverse Kinematics (IK) control, with MoveIt for planning and visualization. You can control the manipulator via a joystick or a GUI, and visualize its state in RViz.

**Key Highlights:**
- 🔀 Seamless switching between control modes (FK Joystick, FK GUI, IK MoveIt)
- 🔧 Hardware agnostic - works with various Arduino-compatible boards
- 📡 Robust ROS 2 architecture with proper topic management
- 🎯 Real-time hardware feedback and control

## ✨ Features
- 🤖 Manipulator control via ROS 2 & Arduino
- 🧠 MoveIt for IK planning & RViz visualization
- 🎮 Joystick and 🖱️ GUI FK control
- 🔄 Automatic management of `joint_state_broadcaster` to avoid conflicts
- 🔀 Easy switching between control modes

## 📋 Prerequisites
- 🐧 **Ubuntu 22.04** (recommended)
- 🌐 **ROS 2 Humble** (or compatible)
- 🦾 **MoveIt 2**
- 🐍 **Python 3.10+**
- 🪫 Arduino board (compatible with your manipulator)
- 🎮 Xbox 360 or similar joystick (for joystick mode)
- 🛠️ All required ROS 2 packages built in your workspace

---

## ⚙️ Installation & Setup

1️⃣ **Install ROS 2 Humble:**
👉 [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)

2️⃣ **Install MoveIt 2:**
👉 [MoveIt 2 Installation Guide](https://moveit.ros.org/install/)

**Install MoveIt Setup Assistant and OMPL (Open Motion Planning Library):**
```bash
sudo apt install ros-humble-moveit-setup-assistant
sudo apt install ros-humble-planners-ompl ros-humble-moveit-planners
```

3️⃣ **Clone this repository:**
```bash
mkdir digitaltwin_ws
```

```bash
cd digitaltwin_ws
```
```bash
git clone https://github.com/rahul-r-joshua/DigitalTwin_ROS2-Manipulator.git .
```
```bash
chmod +x *.sh
```
> ⚠️ **Important:** The `.` at the end clones the repo directly into your current directory. The `chmod` command makes all shell scripts executable.


4️⃣ **Install additional dependencies:**
```bash
sudo apt update && sudo apt install -y ros-humble-joy ros-humble-joint-state-publisher-gui ros-humble-rviz2 python3-serial libserial-dev
```
5️⃣ **Build the workspace:**5️
```bash
colcon build --symlink-install
source install/setup.bash
```



---

## ⚙️ Arduino Firmware Upload (Required Once)

Before running any control mode, upload the firmware to your Arduino:

1. Navigate to the firmware folder:
   ```bash
   cd ~/digitaltwin_ws/src/arduinobot_firmware/firmware
   ```

2. Open `live_moveit_control.ino` in Arduino IDE

3. **Verify pin assignments in the code:**
   - 🔧 Pin 8 → Base (J1)
   - 🔧 Pin 9 → Shoulder (J2)
   - 🔧 Pin 10 → Elbow (J3)
   - 🔧 Pin 11 → Gripper (J4)

4. Upload to Arduino

5. **Close Arduino IDE** before proceeding

---

## 🚀 How to Run

### 📦 Terminal 1: Launch Hardware Bringup (Required First)
```bash
source install/setup.bash

ros2 launch arduinobot_bringup arduinobot_bringup_hardware.py
```
This initializes the controller_manager, hardware interface, and prepares the system for control.

---

### 🎮 Option A: Forward Kinematics via Joystick

#### 🕹️ Check Joystick Connection
```bash
lsusb # Look for your joystick (e.g., Xbox 360 Controller)
```

#### ⚠️ Important
Make sure Arduino IDE is closed before proceeding!

#### 🎮 Terminal 2: Launch Joystick Control
```bash
# Check your Arduino port first: ls /dev/ttyACM* or ls /dev/ttyUSB*

./start_joystick.sh /dev/ttyACM0  # Replace with your actual port
```
- Deactivates `joint_state_broadcaster`, launches joystick control, and reactivates the broadcaster after shutdown.

#### 🎮 Control Mapping
- **Deadman Switch: LB (Left Bumper)** - Hold to enable control
- **J1 (Base):** Left Stick Left/Right
- **J2 (Shoulder):** Left Stick Up/Down
- **J3 (Elbow):** Right Stick Up/Down
- **J4 (Gripper):** Right Stick Left/Right

> 📌 **Note:** RViz will show the robot moving even if the port is wrong! This is simulation-only. Check the terminal logs for:
> - ✅ `Arduino connected successfully` = Hardware will move
> - ❌ `ERROR: Port /dev/ttyACM0 ... does not appear to be an Arduino` = Only RViz visualization, **hardware won't move**

---

### 🖱️ Option B: Forward Kinematics via GUI

#### ⚠️ Important
Make sure Arduino IDE is closed before proceeding!

#### 🖱️ Terminal 2: Launch GUI Control
```bash
# Check your Arduino port first: ls /dev/ttyACM* or ls /dev/ttyUSB*

./start_gui.sh /dev/ttyACM0  # Replace with your actual port
```
- Deactivates `joint_state_broadcaster`, launches GUI control, and reactivates the broadcaster after shutdown.

> 📌 **Note:** RViz will show the robot moving even if the port is wrong! This is simulation-only. Check the terminal logs for:
> - ✅ `Arduino connected successfully` = Hardware will move
> - ❌ `ERROR: Port /dev/ttyACM0 ... does not appear to be an Arduino` = Only RViz visualization, **hardware won't move**

---

### 🧠 Option C: Inverse Kinematics via MoveIt

#### ⚠️ Important
Make sure Arduino IDE is closed before proceeding!

#### 🧠 Terminal 2: Launch MoveIt + RViz
```bash
# Check and replcae your Arduino port first: ls /dev/ttyACM* or ls /dev/ttyUSB*

./start_moveit_digitaltwin.sh /dev/ttyACM0  
```


### Or, run the launch file directly
``` bash
# Replace with your actual port

ros2 launch arduinobot_firmware moveit_digitaltwin.launch.py port:=/dev/ttyACM0 
```
- This launches both MoveIt and RViz for IK planning and visualization.

> 📌 **Note:** IK/FK works through MoveIt. Joystick or GUI control in Gazebo will only visualize motion in RViz and **will not move the robot**.

---

## 🧠 How to Use MoveIt for Robot Control

### 1️⃣ Launch MoveIt + RViz
- MoveIt and RViz are now running from **Option C** above

### 2️⃣ Configure Motion Planning
- **Planning Group:** Select from the dropdown:
  - 🦾 `arm` - Controls the main manipulator joints
  - 🔧 `gripper` - Controls the gripper

### 3️⃣ Set Your Goal Position with Interactive Markers
1. **Drag the Interactive Marker** in RViz to your desired end-effector position
2. The marker will show you where the robot can reach
3. This sets your goal pose for IK solving

### 4️⃣ Configure Motion Planning Settings
1. In the **Motion Planning** panel, go to the **Context** tab
2. Under **Planning Library:** Select `OMPL` (if not already selected)
3. Switch to the **Planning** tab
4. Ensure **Approximate IK Solutions** is turned **ON** (check the box)

> ⚠️ **Important:** Every time you change the **Planning Group** (arm ↔ gripper), you must re-verify that **OMPL** is selected in the **Context** tab. MoveIt may reset the planning library when switching groups!

### 5️⃣ Plan and Execute the Trajectory

**For ARM Control:**
1. In Planning panel → **Planning Group:** Select `arm`
2. Move the **Interactive Markers** in RViz to your desired end-effector position
3. Click **Plan** to generate a collision-free trajectory
4. Visualize the planned path in RViz (it will show in a different color)
5. Click **Execute** to move the robot to the goal position

**For GRIPPER Control:**
1. In Planning panel → **Planning Group:** Select `gripper`
2. Set **Start State:** `home`
3. Set **Goal State:** Click `Random Valid` to generate random valid gripper positions
4. Click **Plan** and **Execute** to move the gripper

### 6️⃣ Joint State Control (Alternative)
- **Random Valid:** Click to move to a random valid configuration
- **Goal State:** Customize joint angles as desired
- Both `arm` and `gripper` groups support random state generation

### 💡 Tips
- Make sure the hardware bringup is running before launching MoveIt
- Always visualize the planned trajectory in RViz **before executing** on real hardware
- If the robot doesn't move after clicking Execute, check:
  - ✅ Hardware is powered on
  - ✅ Correct controllers are active
  - ✅ Arduino connection is stable
- You can switch between `arm` and `gripper` planning groups to control different parts
- Use **Approximate IK** for faster planning if exact IK solutions are not needed

> ⚠️ **Hardware Reachability Note:** Due to physical motor limits, the hardware robot cannot reach some positions that MoveIt plans in RViz. MoveIt's IK solver may generate valid mathematical solutions that exceed the actual motor range or mechanical constraints of your specific hardware. If the robot doesn't move to a planned position, try:
> - Planning to a closer position
> - Using smaller movements
> - Checking that the target is within the physical workspace
> - Verifying joint limits in `joint_limits.yaml` match your hardware capabilities

---

## 🛠️ Troubleshooting

### General Hardware Issues
- 🔌 **Arduino not detected:**
  - Check USB connection and port (e.g., `/dev/ttyACM0`).
  - Ensure user is in the `dialout` group:
    ```bash
    sudo usermod -a -G dialout $USER
    ```
  - Verify Arduino IDE is closed (it holds the port).
  - Check Arduino connection: `ls -la /dev/ttyACM*`

- 🎮 **RViz shows movement but hardware doesn't respond:**
  - RViz visualization works independently of hardware connection!
  - Check terminal logs for Arduino connection errors:
    ```bash
    # Look for: "ERROR: Port /dev/ttyACM0 ... does not appear to be an Arduino"
    ```
  - Verify correct port: `ls /dev/ttyACM* /dev/ttyUSB*`
  - Look for successful connection message: `Arduino connected successfully`
  - Restart with the correct port if you see connection errors

### ROS 2 & Controller Issues
- ⚠️ **joint_state_broadcaster conflict:**
  - Only one publisher should be active on `/joint_states`.
  - Scripts automatically deactivate the broadcaster for FK (joystick/gui) and reactivate it after shutdown.
  - **If RViz glitches or hardware jerks:** the broadcaster likely wasn't deactivated. Use this reset sequence, then relaunch `start_joystick.sh` or `start_gui.sh`:
    ```bash
    ros2 control list_controllers
    ros2 control switch_controllers --deactivate joint_state_broadcaster --strict
    ```
    - Kill the current joystick/GUI process and relaunch it.
  - Verify controller state:
    ```bash
    ros2 control list_controllers
    ```
  - Check active topics:
    ```bash
    ros2 topic echo /joint_states
    ```

- 🧠 **MoveIt not controlling hardware:**
  - IK planning works in RViz, but hardware may not move if controllers are not properly configured.
  - **Check the Arduino port:** Verify you're using the correct port (`ls /dev/ttyACM*`)
  - **Verify in RViz Context panel:** Ensure `OMPL` is loaded and active
  - Check controller_manager logs: `ros2 param list /controller_manager`
  - Ensure correct controller activation via `ros2 control list_controllers`
  - Verify MoveIt config is correct for your hardware setup
  - Restart the hardware bringup if the port was incorrect

### RViz & Visualization Issues
- 🏳️ **RViz shows blank/white screen:**
  - Wait for hardware controller to start publishing.
  - Robot will appear once topics are active.
  - Check available topics: `ros2 topic list`
  - Verify joint_state_broadcaster is active for MoveIt visualization

---

## 🏆 Major Challenge
The main challenge is managing the `joint_state_broadcaster`. It publishes to `/joint_states` and is required by MoveIt, but conflicts with hardware control (joystick/gui). To avoid this:
- Scripts deactivate `joint_state_broadcaster` for FK control.
- After shutdown, it is automatically reactivated so MoveIt can function.

---

## 🔧 Hardware Build Guide

### 📦 Required Components

#### 🖨️ 3D Printed Parts
All STL files are included in the repository for 3D printing:
- Basement
- Base Plate
- Claw Support
- Drive Gear
- Driven Gear
- Forward Drive Arm
- Horizontal Arm
- Left Finger
- **Link Plate** × 3
- Right Finger
- Round Plate
- **Servo Plate** × 2
- Triangular Link
- Vertical Drive Arm

#### ⚙️ Electronics
- **4× SG90 Servo Motors** (or compatible 9g servos)
- **1× Arduino Uno**
- **External Power Supply** (5V, capable of powering 4 servos)
- **Breadboard**
- **Jumper Wires** (sufficient quantity)
- **Arduino USB Cable**

#### 🔩 Hardware (All Allen Head Screws)
- **M3 Screws 12mm** × 7
- **M3 Screws 14mm** × 7 (or use 15mm if not available)
- **M3 Screws 16mm** × 4 (or use 15mm if not available)
- **M3 Screws 10mm** × 1
- **M3 Screws 30mm** × 1
- **M2 Screws 20mm** × 2
- **M3 Nuts** × 22

> 💡 **Tip:** If 14mm or 16mm screws are not available in your local market, you can use 15mm screws instead.

#### 🔫 Assembly Tools
- Hot glue gun with glue sticks
- Screwdriver set (Allen key/hex set for M2 and M3)

---


### 🛠️ Assembly Instructions

> 📌 **Hardware Note:** Before mounting the motors, set each motor to **90° (center)**. Calibration code is available at 
```bash 
cd ~/digitaltwin_ws/src/arduinobot_firmware/firmware/motor_calibration
```

#### J1 (Base)
1. **Secure the base servo (J1)** inside the basement using a hot glue gun. Ensure the servo is firmly fixed and the output shaft is centered.
2. **Attach the servo double arm** to the servo horn. This will provide a strong anchor for the base plate.
3. **Mount the base plate** onto the servo double arm and secure it with screws if possible.
> 💡 **Tip:** Wait for the glue to cool and harden before proceeding to the next step.

#### J2 (Shoulder)
1. **Attach a servo plate** to the shoulder servo (J2) for easier mounting.
2. **Fix the shoulder servo** to the base plate using screws through the servo plate holes.
3. **Attach the servo single arm** to the forward drive arm using a hot glue gun. Make sure the arm is aligned straight.
4. **Connect the forward drive arm** (with single arm attached) to the servo horn of the shoulder servo.
> 💡 **Tip:** Double-check alignment before gluing for smooth movement.

#### J3 (Elbow)
1. **Connect the base plate** to the vertical drive arm using screws or the provided mounting holes.
2. **Attach a servo single arm** to the vertical drive arm with a hot glue gun, ensuring a solid bond.
3. **Mount the elbow servo (J3)** to the assembly using a servo plate and screws.
4. **Attach the servo horn** to the single arm for actuation.

#### J4 (Gripper)
1. **Attach the claw support** to the end of the vertical drive arm. Make sure it is firmly aligned and secured with screws or bolts.
2. **Mount the gripper servo (J4)** onto the claw support:
  - Apply a hot glue gun for initial placement, ensuring the servo output shaft is facing outward and properly aligned.
  - Once the glue sets, reinforce with screws through the servo mounting tabs for maximum stability.
  - ⚠️ **Important:** Double-check the servo orientation before gluing and screwing!
3. **Assemble the gripper mechanism:**
  - Attach the fingers and linkage parts to the servo horn as per the STL design.
  - Ensure all moving parts are free and can rotate smoothly.
  - Use the correct screws and check that the linkage pivots easily.
4. **Test the gripper movement** by gently rotating the servo horn by hand. Confirm the gripper opens and closes fully without binding.
  - If needed, adjust alignment or add a drop of lubricant to the pivot points for smoother operation.
5. **Final check:** Tighten all fasteners and make sure servo wires are routed safely, away from moving parts.
> 💡 **Tip:** Temporarily power the servo and run a test open/close cycle to confirm correct orientation and range before final assembly.

#### Wiring & Power
1. **Connect each servo signal wire to the Arduino:**
  - **Pin 8** → J1 (Base)
  - **Pin 9** → J2 (Shoulder)
  - **Pin 10** → J3 (Elbow)
  - **Pin 11** → J4 (Gripper)
2. **Connect all servo power (VCC) and ground (GND) wires** to the breadboard, powered by an external 5V supply.
3. **Ensure the Arduino GND is connected** to the external power supply GND (common ground).
4. **Double-check all connections before powering on.**

> ⚠️ **Important:** Never power the servos directly from the Arduino 5V pin! Always use an external 5V supply capable of handling all 4 servos at once.
  - **Pin 8** → J1 (Base)
  - **Pin 9** → J2 (Shoulder)
  - **Pin 10** → J3 (Elbow)
  - **Pin 11** → J4 (Gripper)
2. Connect all servo power (VCC) and ground (GND) wires to the breadboard, powered by an external 5V supply.
3. Ensure the Arduino GND is connected to the external power supply GND (common ground).
4. Double-check all connections before powering on.

> ⚠️ **Important:** Never power the servos directly from the Arduino 5V pin! Always use an external 5V supply capable of handling all 4 servos at once.

---

## 💡 Additional Notes
- Switch between joystick and GUI control as needed.
- Launch files are designed to be robust and avoid controller conflicts.
- For custom manipulator hardware, update the Arduino firmware and ROS 2 parameters as required.

---

## 🙋‍♂️ Questions?
For questions or issues, open an issue on GitHub or contact the maintainer.

---

## 🤝 Acknowledgments
This project is built on the excellent manipulator design from [AntoBrandi's Robotics and ROS 2 Learn by Doing - Manipulators](https://github.com/AntoBrandi/Robotics-and-ROS-2-Learn-by-Doing-Manipulators). We extend our gratitude for the foundational work that made this digital twin implementation possible.

---

**Happy Hacking!** 🤖🦾🎮🖱️
# DigitalTwin_ROS2-Manipulator

# Catch_Nearest_Turtle_ROS2


markdown
Copy
Edit
# 🐢 Catch_Nearest_Turtle_ROS2

This ROS 2 package simulates a turtle in the `turtlesim` environment that automatically catches the **nearest turtle**. It uses ROS 2 nodes to detect the closest turtle and move toward it until it's caught.

---

## 🚀 How to Run

### 1. Clone the repository

```bash
cd ~/catch_turtle_ws/src
git clone <your-repo-url> my_turtle_bringup
2. Build the workspace
bash
Copy
Edit
cd ~/catch_turtle_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
3. Source the workspace
bash
Copy
Edit
source install/setup.bash
4. Launch the simulation
bash
Copy
Edit
ros2 launch my_turtle_bringup turtlesim_catch_them_all.launch.xml
This command will:

Start the turtlesim simulation

Spawn multiple turtles

Launch the controller node that makes the main turtle chase and catch the nearest one

⚙️ Configuration
Configuration files are located at:

swift
Copy
Edit
/home/atharva/catch_turtle_ws/src/my_turtle_bringup/config/
These YAML files define various parameters such as:

Turtle speed

Catching radius

Number of turtles to spawn

Update rate for position tracking

You can modify these to fine-tune the simulation behavior.

📁 Project Structure
css
Copy
Edit
my_turtle_bringup/
├── config/
│   └── [*.yaml]           # Configuration files
├── launch/
│   └── turtlesim_catch_them_all.launch.xml
├── src/
│   └── [*.cpp]            # ROS 2 nodes
├── CMakeLists.txt
└── package.xml
✅ Dependencies
Make sure the following ROS 2 packages are installed:

turtlesim

geometry_msgs

rclcpp

tf2_ros

launch_ros

Install missing dependencies with:

bash
Copy
Edit
sudo apt install ros-<your-distro>-turtlesim
Replace <your-distro> with your ROS 2 version (e.g., humble, iron, foxy).

📜 License
This project is licensed under the MIT License. Feel free to use, modify, and share.

👨‍💻 Author
Atharva


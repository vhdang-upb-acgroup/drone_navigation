# 🤖 ROS-Gazebo Implementation: Nonlinear Control Design

This sub-repository provides an initial implementation for students participating in "Nonlinear Control of Autonomous and Robotic Systems."

## 🗂️ Repository Structure

```bash
drone_navigation/
├── launch/ 
├── models/
├── package.xml
├── CMakeList.txt
└── README.md
```
## Getting started
1. Install Ubuntu 22.04 or 24.04
2. Install ROS2: Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)
3. Install Gazebo
## We are now here to use this implementation
1. Create a custom ROS2 package:
   ```bash
   ## Create your working directory:
   mkdir -p /<your_preference_name>/src
   ## Navigate to your working directory:
   cd ..
   ## Now you are in /<your_preference_name> folder and you can build the package
   ros2 pkg create --build-type ament_python <your_package_name>
   ```
2. Clone or download this to <your_working_directory>/src
   ```bash
   # Clone the repo to your machine
   git clone
   # You can download it directly
   ```
3. Modify CMakeList.txt and package.xml accordingly
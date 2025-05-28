# 🤖 ROS-Gazebo Implementation: Nonlinear Control Design

This sub-repository provides an initial implementation for students participating in "Nonlinear Control of Autonomous and Robotic Systems."

## 🗂️ Repository Structure

```bash
drone_navigation/
├── launch/ 
├── models/
├── docs/
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
2. Download launch and models to <your_working_directory>/src/<your_package_name>
   ```bash
   # You can download it directly
   ```
3. Modify CMakeList.txt and package.xml accordingly

## An alternative, you can clone the repo 
   ```bash
   # Clone the repo to your working directory /src
   git clone git@github.com:vhdang-upb-acgroup/drone_navigation.git
   ```
## Build and run the launch file
   1. navigate to /<your working directory> such as ros2_ws, etc
   2. Run colcon build
   ```bash
      # Option 1: Build all packages which are located in /src
      colcon build
      # Option 2: Build only specific package
      colcon build --packages-select <package_name>
   ```
# Remember to source after building the package
   ```bash
      # source to get update by
      source install/setup.bash
   ```
# Now try to run the launch file to see
   ```bash
      # Run launch file with ROS2
      # ros2 launch <name_of_your_package> <name of your launch file>
      # For example:
      ros2 launch drone_navigation drone_nav.launch.py
   ```
# If everything works, you can observe
![Gazebo Interface](docs/gazebo.png)

## Exercise 1: Check control inputs and feedback pose
In order to design a feedback control system, it is important to understand how the drone is controlled and which information is available.

# Control signals:
   For this drone system, we can control it via RPMs of four motors. 

# Feedback signals:
   We can access to the drone's pose which contains positions: (x, y, z) and orientation: (x, y, z, w) in quaternion.

## Exercise 2: Design a simple P or PI controller for hovering task
To do so, you should create another ROS2 package and you can name it "drone_controller"
1. To create a ROS2 package you can do as follows
   ```bash
      # Create an empty ROS2 package with python
      # Navigate to <your working directory>/src folder and run
      ros2 pkg create --build-type ament_python drone_controller
      # Build this package: You need to navigate to your working directory
      cd ..
      # Option 1: Build all packages inside /src with colcon build
      colcon build
      # Option2: Build only drone controller package with
      colcon build --packages-select drone_controller
   ```
2. Create a python file to implement your sensor_node that reads the pose of the drone
   ```bash
      ## You need to navigate to src/drone_controller/drone_controller
      # Option 1: using VSCode to create a python file called: sensor.py
      # Option 2: using touch sensor.py
      touch sensor.py
   ```
3. Start writing your implementation of reading the pose of the drone
   ```bash
      import rclpy
      from rclpy.node import Node
      from geometry_msgs.msg import PoseArray

      class PoseListener(Node):
         def __init__(self):
            super().__init__('pose_listener')
            self.subscription = self.create_subscription(
                  PoseArray,
                  '/world/quadcopter/pose/info',
                  self.pose_callback,
                  10)

         def pose_callback(self, msg: PoseArray):
            if len(msg.poses) > 1:
                  pose = msg.poses[1]  # Second pose
                  self.get_logger().info(
                     f"Second Pose: Position({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}) "
                     f"Orientation({pose.orientation.x:.2f}, {pose.orientation.y:.2f}, "
                     f"{pose.orientation.z:.2f}, {pose.orientation.w:.2f})"
                  )
            else:
                  self.get_logger().warn("Received PoseArray with fewer than 2 poses")

      def main(args=None):
         rclpy.init(args=args)
         node = PoseListener()
         rclpy.spin(node)
         node.destroy_node()
         rclpy.shutdown()

      if __name__ == '__main__':
         main()
   ```
4. Update dependencies and console
   4.1 Update dependencies: Update the package.xml
   ```bash
      <!-- Build dependencies -->
      <build_depend>ros_gz_bridge</build_depend>
      <build_depend>rclcpp</build_depend>
      <build_depend>rclpy</build_depend>
      <build_depend>std_msgs</build_depend>
      <build_depend>geometry_msgs</build_depend>

      <!-- Runtime dependencies -->
      <exec_depend>ros_gz_bridge</exec_depend>
      <exec_depend>rclcpp</exec_depend>
      <exec_depend>rclpy</exec_depend>
      <exec_depend>std_msgs</exec_depend>
      <exec_depend>geometry_msgs</exec_depend>
   ```
   4.2 Update console: In setup.py you should add
   ```bash
      entry_points={
         'console_scripts': [
               "sensor_node = drone_controller.sensor:main",
         ],
      },
   ```
5. Build and run the node
   ```bash
      # Build the package
      colcon build --packages-select drone_controller
      # Make sure you source before you run the node
      source install/setup.bash
      # Run it
      ros2 run drone_controller sensor_node
   ```
## Exercise 3: Let's write control_command.py to send RPMs to control gz drone
All you need to do is that
1. You create a control_commands.py within src/drone_controller/drone_controller and the implementation looks like codes below
   ```bash
      import rclpy
      from rclpy.node import Node
      from actuator_msgs.msg import Actuators  # Make sure this matches your message package
      from std_msgs.msg import Header
      import time

      class MotorCommandPublisher(Node):
         def __init__(self):
            super().__init__('motor_command_publisher')
            self.publisher_ = self.create_publisher(Actuators, '/X3/gazebo/command/motor_speed', 10)
            self.timer = self.create_timer(0.1, self.timer_callback)

         def timer_callback(self):
            msg = Actuators()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()

            # Set motor velocities (rad/s), adjust as needed
            msg.velocity = [500.0, 500.0, 500.0, 500.0]

            # Optional: clear other fields
            msg.position = []
            msg.normalized = []

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published motor velocities: {msg.velocity}')

      def main(args=None):
         rclpy.init(args=args)
         node = MotorCommandPublisher()
         rclpy.spin(node)
         node.destroy_node()
         rclpy.shutdown()

      if __name__ == '__main__':
         main()
   ```
2. Update dependencies and console
   In setup.py add
   ```bash
      "control_node = drone_controller.control_commands:main",
   ```
3. Rebuild, source and run it again





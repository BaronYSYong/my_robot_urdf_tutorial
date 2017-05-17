# Step 3: Using the robot state publisher on your own robot

When you are working with a robot that has many relevant frames, it becomes quite a task to publish them all to tf. The robot state publisher is a tool that will do this job for you.

The robot state publisher helps you to broadcast the state of your robot to the tf transform library. The robot state publisher internally has a kinematic model of the robot; so given the joint positions of the robot, the robot state publisher can compute and broadcast the 3D pose of each link in the robot.

You can use the robot state publisher as a standalone ROS node or as a library:

## 1. Running as a ROS node
### 1.1 robot_state_publisher
The easiest way to run the robot state publisher is as a node. For normal users, this is the recommended usage. You need two things to run the robot state publisher:

* A urdf xml robot description loaded on the Parameter Server.
* A source that publishes the joint positions as a sensor_msgs/JointState.
* http://wiki.ros.org/robot_state_publisher

#### 1.1.1 Subscribed topics
* joint_states (sensor_msgs/JointState)
    * joint position information
    * http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
    
    


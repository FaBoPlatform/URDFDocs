# Robot Car

## hello_urdf.urdf.xacro

hello_urdf.urdf.xacro

``` xml
<?xml version="1.0" encoding="UTF-8"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="hello_urdf">
   <xacro:include filename="hello_urdf.gazebo" />
   <link name="robot_footprint" />
   <joint name="robot_footprint_joint" type="fixed">
      <origin xyz="0 0 1" rpy="0 0 0" />
      <parent link="robot_footprint" />
      <child link="chassis" />
   </joint>
   <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
         <mass value="20.0" />
         <origin xyz="0.0 0 .25" rpy=" 0 0 0" />
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
      </inertial>
      <collision name="collision">
         <origin xyz="0 0 .25" rpy=" 0 0 0" />
         <geometry>
            <box size=".95 .6 .05" />
         </geometry>
      </collision>
      <visual name="chassis_visual">
         <origin xyz="0 0 0.25" rpy=" 0 0 0" />
         <geometry>
            <box size=".95 .6 .05" />
         </geometry>
      </visual>
   </link>
   <link name="front_right_wheel">
      <visual>
         <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707" />
         <geometry>
            <cylinder length="0.05" radius="0.1" />
         </geometry>
      </visual>
      <collision>
         <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707" />
         <geometry>
            <cylinder length="0.05" radius="0.1" />
         </geometry>
      </collision>
      <inertial>
         <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707" />
         <mass value="5.5" />
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
      </inertial>
   </link>
   <link name="back_right_wheel">
      <visual>
         <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707" />
         <geometry>
            <cylinder length="0.05" radius="0.2" />
         </geometry>
      </visual>
      <collision>
         <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707" />
         <geometry>
            <cylinder length="0.05" radius="0.2" />
         </geometry>
      </collision>
      <inertial>
         <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707" />
         <mass value="0.5" />
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
      </inertial>
   </link>
   <joint name="front_right_wheel_joint" type="continuous">
      <axis xyz="0 1 0" rpy="0 0 0" />
      <dynamics damping="1.0" friction="1.0" />
      <parent link="chassis" />
      <child link="front_right_wheel" />
      <origin xyz="0.3 -0.3 -0.1" rpy="0 0 0" />
   </joint>
   <joint name="back_right_wheel_joint" type="continuous">
      <axis xyz="0 1 0" rpy="0 0 0" />
      <dynamics damping="1.0" friction="1.0" />
      <parent link="chassis" />
      <child link="back_right_wheel" />
      <origin xyz="-0.2 -0.3 0" rpy="0 0 0" />
   </joint>
   <link name="front_left_wheel">
      <visual>
         <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707" />
         <geometry>
            <cylinder length="0.05" radius="0.1" />
         </geometry>
      </visual>
      <collision>
         <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707" />
         <geometry>
            <cylinder length="0.05" radius="0.1" />
         </geometry>
      </collision>
      <inertial>
         <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707" />
         <mass value="5.5" />
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
      </inertial>
   </link>
   <link name="back_left_wheel">
      <visual>
         <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707" />
         <geometry>
            <cylinder length="0.05" radius="0.2" />
         </geometry>
      </visual>
      <collision>
         <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707" />
         <geometry>
            <cylinder length="0.05" radius="0.2" />
         </geometry>
      </collision>
      <inertial>
         <origin xyz="0.0 0 0" rpy=" 0 1.5707 1.5707" />
         <mass value="0.5" />
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
      </inertial>
   </link>
   <joint name="front_left_wheel_joint" type="continuous">
      <axis xyz="0 1 0" rpy="0 0 0" />
      <dynamics damping="1.0" friction="1.0" />
      <parent link="chassis" />
      <child link="front_left_wheel" />
      <origin xyz="0.3 0.3 -0.1" rpy="0 0 0" />
   </joint>
   <joint name="back_left_wheel_joint" type="continuous">
      <axis xyz="0 1 0" rpy="0 0 0" />
      <dynamics damping="1.0" friction="1.0" />
      <parent link="chassis" />
      <child link="back_left_wheel" />
      <origin xyz="-0.2 0.3 0" rpy="0 0 0" />
   </joint>
   <gazebo reference="chassis">
      <material>Gazebo/Gray</material>
   </gazebo>
   <gazebo reference="front_right_wheel">
      <material>Gazebo/Black</material>
   </gazebo>
   <gazebo reference="front_left_wheel">
      <material>Gazebo/Black</material>
   </gazebo>
   <gazebo reference="back_right_wheel">
      <material>Gazebo/Black</material>
   </gazebo>
   <gazebo reference="back_left_wheel">
      <material>Gazebo/Black</material>
   </gazebo>
</robot>
```

## hello_urdf.gazebo

``` xml
<?xml version="1.0"?>
<robot>

  <!--skid_steer_drive_controller-->
  <gazebo>
    <plugin name="skid_steer_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/hello_urdf</namespace>
        <argument>/cmd_vel:=mr_cme_vel</argument>
        <argument>/odom:=odom</argument>
      </ros>
      <update_rate>100.0</update_rate>
      <odometry_frame>odom</odometry_frame>
            <num_wheel_pairs>2</num_wheel_pairs>

      <robot_base_frame>robot_footprint</robot_base_frame>
      <max_wheel_torque>20</max_wheel_torque>
      <left_joint>front_left_wheel_joint</left_joint>
      <left_joint>back_left_wheel_joint</left_joint>
      <right_joint>front_right_wheel_joint</right_joint>
      <right_joint>back_right_wheel_joint</right_joint>
      <wheel_separation>0.108</wheel_separation>
      <wheel_diameter>0.024</wheel_diameter>
      <publish_wheel_joint_state>true</publish_wheel_joint_state>
      <legacy_mode>false</legacy_mode>

      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>

      <odometry_source>world</odometry_source>
    </plugin>
  </gazebo>
</robot>
```

## TEST

``` console
ros2 topic pub /hello_urdf/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}' -1
```
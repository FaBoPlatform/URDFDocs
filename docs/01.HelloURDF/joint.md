# URDF Docs

## Projectの作成

``` console
cd ~/ros_ws/src/
ros2 pkg create --build-type ament_cmake hello_urdf
```

``` console
cd ~/ros_ws/src/hello_urdf/
mkdir urdf
cd urdf
```

`urdf/hello_urdf.urdf.xacro`

``` xml
<robot name="hello_urdf" 
  xmlns:xacro="http://www.ros.org/wiki/xacro">
 
  <xacro:include filename="hello_urdf.gazebo" />
 
  <link name="robot_footprint"></link>
 
  <joint name="robot_footprint_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="robot_footprint"/>
    <child link="chassis" />
  </joint>
 
  <link name='chassis'>
    <pose>0 0 0.1 0 0 0</pose>
 
    <inertial>
      <mass value="15.0"/>
      <origin xyz="0.0 0 0" rpy=" 0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1" />
    </inertial>
 
    <collision name='collision'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/>
      <geometry>
        <box size=".5 .5 .1"/>
      </geometry>
    </collision>
 
    <visual name='chassis_visual'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/>
      <geometry>
        <box size=".5 .5 .1"/>
      </geometry>
    </visual>
    </link>

  	<gazebo reference="chassis">
    	<material>Gazebo/Gray</material>
  	</gazebo>

</robot>

```

`urdf/hello_urdf.gazebo` 

```
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

      <robot_base_frame>robot_footprint</robot_base_frame>
      <max_wheel_torque>20</max_wheel_torque>
 
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
 
      <odometry_source>world</odometry_source>
    </plugin>
  </gazebo>
</robot>
```

``` console
cd ~/ros_ws/src/
mkdir launch
```

`launch/world.launch.py`

``` python
import os
  
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# this is the function launch  system will look for

def generate_launch_description():

    robot_name = 'hello_urdf'
    # full  path to urdf and world file
    urdf = os.path.join(get_package_share_directory(robot_name), 'urdf', 'hello_urdf.urdf')

    # read urdf contents because to spawn an entity in 
    # gazebo we need to provide entire urdf as string on  command line

    xml = open(urdf, 'r').read()

    # double quotes need to be with escape sequence
    xml = xml.replace('"', '\\"')

    # this is argument format for spwan_entity service 
    spwan_args = '{name: \"hello_urdf\", xml: \"'  +  xml + '\" }'

    # create and return launch description object
    return LaunchDescription([

        # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
        # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
```

## Command

``` console
ros2 topic pub  -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```

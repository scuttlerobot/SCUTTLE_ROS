-------------------------
This is a guide for setting up a fresh catkin workspace (If you already have one this is for creating a new one, just dont overwrite your old one accidentally if you want to keep it)
You can change the name but im gonna use catkin_ws for my workspace. In addition the code used will remove any existing folder with the name catkin_ws in your home directory so be careful!!!

Prereq:
Installed ROS Melodic or Kinetic and Appropriate Ubuntu

remove old catkin_ws (OPTIONAL)
```
cd ~/
sudo rm -r -f catkin_ws
```

create a new catkin workspace
```
cd ~/
mkdir --parents catkin_ws/src
cd catkin_ws
catkin init
catkin_make
```

This should produce a new folder called catkin_ws in your home directory to store all of our custom scripts. Inside the catkin_ws folder your should see another folder called src this is where cloned packages will go!

-------------------------

This is a guide for setting up the base scuttle packages
- scuttle
- scuttle_description
- -rplidar_ros

1: scuttle base package contains the startup launch file. This is used on boot to "bring up" the rplidar, motor control code and some other bits and bobs.
```
cd ~/catkin_ws/src
catkin_create_pkg scuttle rospy roscpp scuttle_description
cd scuttle
mkdir launch
cd launch
touch scuttle_bringup.launch
chmod +x scuttle_bringup.launch
```
At this point you need to edit the created launch file with the code below. Ill explain after:
```
<launch>
	<param name ="/use_sim_time" value="false"/>		
	<arg name="gui" default="False" />
	<param name="use_gui" value="$(arg gui)"/>
	<param name="robot_description" command="cat $(find scuttle_description)/urdf/scuttle.urdf" />
	
	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
	
	<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

	<node name="cmd_vel_sub" pkg="two_wheel_driver" type="cmd_vel_sub.py" output="screen"/>

	<include file="$(find rplidar_ros)/launch/rplidar.launch" />
</launch>
```

The launch file will run nodes and other packages in a top down order. So first it checks if we are using the simulator time. NO... Then are we using a GUI. NO at least for me. Next is the robot description urdf file. This technically doesn't exist yet in the process so bear with me! This just loads the file for us and makes it accessible within ros for handling the relation in space of our sensors.

next the joint state publisher and the robot state publisher publish data from our urdf and sensors to the TF. Dont worry about this for now.

then we see a node that launches our driving script. This is loaded from the two_wheel_driver package and run along with all of the other programs.

finally we launch the rplidar node and we are all set for now.

2: Create the scuttle_description package
```
cd ~/catkin_ws/src
catkin_create_pkg scuttle_description robot_state_publisher urdf
cd scuttle_description
mkdir urdf
cd urdf
touch scuttle.urdf
```
edit the scuttle.urdf file with the below code: This is my bad urdf but maybe itll be changed *fingers crossed*
```
<?xml version="1.0"?>
	<robot name="scuttle">
		<link name="base_link">
		<visual>
		    <origin xyz="0 0 0.0" rpy="0 0 0"/>
			<geometry>
				<box size="0.32 0.26 0.085"/>
			</geometry>
			<material name="Cyan1">
	       		<color rgba="0 0.9 0.9 1.0"/>
	     	</material>
		</visual>	
		</link>		
		<link name="laser">
      <visual>
		    <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.03" length="0.03"/>
        </geometry>
        <material name="Black1">
              <color rgba="0.2 0.2 0.2 1.0"/>
          </material>
		  </visual>	
		</link>

		<joint name="base_link_to_laser" type="fixed">
			<parent link="base_link"/>
			<child link="laser"/>
			<origin xyz="-0.143 0 0.1" rpy="0.0 0.0 0.0"/>
			<axis xyz="0 0 0"/>
		</joint>	
		
	</robot>

```

3: Clone the rplidar_ros package from github to the right location
```
cd ~/catkin_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
```

4: build the catkin_ws with catkin_make ONLY IF YOU CREATED ALL PACKAGES AND FILES!
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

5: test the new launch file directly
```
roslaunch scuttle scuttle_bringup.launch
```

if all works you can reopen the teleop twist keyboard and drive the robot around. Also you should be able to see topics in RVIZ if you know how.

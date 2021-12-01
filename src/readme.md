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

This repository contains software which may be used to teloperate the PR2 robot
by Willow Garage, using the PS3 controller and various immersive perspectives.

One perspective utilizes the Oculus rift to map the persons head movements to
the PR2 and displays the stereo cameras to their view.

The second utilizes an RViz perspective where the user controls head movement
by pointing and clicking the image on a screen.

The third we used was simply driving the PR2 as an RC car and reuqires no
special tools for perspective.

The pr2_apps portion of the code, used to operate the PR2 with the PS3
controller, was based on code found here.

https://github.com/bosch-ros-pkg

The surrogate code, which runs the oculus was based on code from.

https://github.com/ros-interactive-manipulation


Here are some basic launch directions.

##################################
####### Navigation Task ##########
##################################

# Run Oculus or 3rd person start-upa

### On Robot
roslaunch oculus_teleop robot_drive_oculus.launch

### On Desktop
roslaunch oculus_teleop desktop_Oculus.launch


# Run Rviz or 3rd person start-up

### On Robot
roslaunch oculus_teleop robot_drive_rviz.launch

### On Desktop
roslaunch oculus_teleop desktop_Rviz.launch


##################################
###### Manipulation Task #########
##################################

# Run Oculus or 3rd person start-up

### On Robot
roslaunch oculus_teleop robot_manipulate_oculus.launch

### On Desktop
roslaunch oculus_teleop robot_manipulate_rviz.launch

# Run Rviz or 3rd person start-up

### On Robot
roslaunch oculus_teleop desktop_Oculus.launch

### On Desktop
roslaunch oculus_teleop desktop_Rviz.launch


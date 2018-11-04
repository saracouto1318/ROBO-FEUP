# ROBO-FEUP

 Pre-Requisites:
 * Install ROS
 * Install STDR
 * Copy all the contents of 'stdr_resources/maps' to the respective folder inside the stdr_resources package (use 'roscd stdr_resources/maps')
 * Copy all the contents of 'stdr_resources/resources/robots' to the respective folder in the stdr_resources package
         (use 'roscd stdr_resources/resources/robots')

 Compilation / Installation:
 * Execute 'catkin_make' in the root directory

 Use:
 * Initialize the STDR GUI + Map + Robot
   - Execute:
   
 * 'roslaunch stdr_launchers reactive-robot.launch' for robot in V-squared wall
 * 'roslaunch stdr_launchers reactive-robot2.launch' for robot in V-pointed wall
 * 'roslaunch stdr_launchers reactive-robot-W.launch' for robot in W-squared wall
 * 'roslaunch stdr_launchers reactive-robot-W.launch' for robot in W-pointed wall
 
 * Initialize code
   - Execute:
   
 * 'rosrun reactive_robot robot robot0 laser_0'
 
 
 Report: https://www.overleaf.com/6654294114vfjcwbgcdtgk

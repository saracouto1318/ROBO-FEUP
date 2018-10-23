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
   
 * 'roslaunch stdr_launchers reactive-robot.launch' for robot in V wall
 * 'roslaunch stdr_launchers reactive-robot-W.launch' for robot in W wall

##Subacz - RBE550 - Baxter Dual Arm Manipulator


## Baxter API: http://api.rethinkrobotics.com/baxter_interface/html/index.html

##Steps for success
##1.0 - Starting the Baxter Sim:
##     1.1 - Open a terminal and enter './baxter.sh sim'
##     1.2 - Next enter "roslaunch baxter_gazebo baxter_world.launch", it might take a few minutes be patient!!!
##     1.3 - (Optional) Test to make sure the sim is installed and running correcty, Open a new terminal
##     1.3.1 - "rosnode list",  should show similiar results as http://sdk.rethinkrobotics.com/wiki/Simulator_Architecture#Ros_Nodes
##     1.3.2 - "rostopic list", should show similiar results as http://sdk.rethinkrobotics.com/wiki/Simulator_Architecture#Ros_Topics
##     1.3.3 - "rostopic echo /robot/state",The print message should show:
##          enabled: False
##          stopped: False
##          error: False
##          estop_button: 0
##          estop_source: 0
##


import 
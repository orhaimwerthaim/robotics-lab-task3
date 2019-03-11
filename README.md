# robotics-lab-task3
turtlebot3 navigation and pushing the blue box

for running the project:
1. put the src folder in your ROS catkin workspace
2. make sure the your bot and pc ROS_MASTER_URI and ROS_IP parameters are set to the correct IP.
3. make sure that the time in both your PC and turtlebot3 and in sync 
    (use '$ date +%s' to see the machine epoche time, and '$ date -s '@<epcheTime>' to set the epochetime).
4. run 'roscore' from your PC. 
5. run 'roslaunch turtlebot3-camera turtlebot3_camera.launch' from the turtelbot3.
6. run 'rosrun task3 pub_angles_from_colored_obj.py', 'rosrun task3 bot_move_control.py' and 'rosrun task3 task3.py' from your PC.
7. set the final navigation destination via RVIs GUI.
  
  

explanation on the different nodes:
1. pub_angles_from_colored_obj ROS node is getting the bot camera data via the '/usb_cam/image_raw' topic.
   it publishes the distance of the center of the blue object from the center of the frame via the '/angles_from_colored_object'    topic.
   it publishes a Float64 msg when 0.0 is for centered and 900000 is for 'object not in frame'.
2. bot_move_control ROS node is used for controling the turtelbot3 movement not via the navigation stack.
   the node receives a MoveCommand message type on the '/move_bot_command' topic.
   MoveCommand has 'direction' string field then can get values of 'counter-clock-wise-rotate' or 'forward', a 'distance' float field that states the distance in degrees/meters the bot should move, and a 'speed' float field in m/s (speed can be positive or nagative).
   the node translate these commands to the turtelbot3 'cmd_vel' topic commands.
3. task3 ROS node is used to make sure we move the blue box.
   it does it by checking if the blue box is in the frame or by doing a full 360 degrees check every 20 seconds for cases the bot did not move towards the box directly.
   when the box is found the bot stopps the navigation, pushes the box and then continue to his original destination.
   
   
   techinacal problems i incountered:
   1. the navigation stack was not able to move the bote. the reason was because the clocks of the PC and bot were not synchronaized. i tried setting the PC to be the NTP server of the bot, that did not work so i manually changed the time of the bot to be like the PC (using '$ date +%s' and '$ date -s '@<epche time>').
    for it to work it is importent to set the 'transform_tolerance' parameter to '15.0' so it want be strict (in local_costmap_params.yaml and global_costmap_params.yaml.
    2. from some reason when i used the navigation stack the bot was not moving floauntly, and it looked in the RVis GUI like an inflation problem.
    setting the inflation to 0.0 and restarting the navigation did node did not change this state. only when i checked it in a different day it worked so may be this problem required full restart of the turtlebot/roscore etc.. (even doe it does not make sense).
    3. when activating the navigation node using a map file the bot initially finds it location but is not fully adjusted with the map walls.
    after consulting with people that worked with ROS i found that causing the turtelbot to make 1-3 rounds in his place fixes the problem.
    
 


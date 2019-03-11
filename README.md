# robotics-lab-task3
turtlebot3 navigation and pushing the blue box

for running the project:
1. put the src folder in your ROS catkin workspace
2. make sure that your turtlebot3 and PC ROS_MASTER_URI and ROS_IP ENV. parameters are set to the correct IP.
3. make sure that the time in both your PC and turtlebot3 is synchronized 
    (use '$ date +%s' to see your machine epoch time, and '$ date -s '@<epochTime>' to set the epoch time on your turtlebot3).
4. run 'roscore' from your PC. 
5. run 'roslaunch turtlebot3-camera turtlebot3_camera.launch' from the turtelbot3.
6. run 'rosrun task3 pub_angles_from_colored_obj.py', 'rosrun task3 bot_move_control.py' and 'rosrun task3 task3.py' from your PC.
7. set the final navigation destination via RVIz GUI.
  
  

explanation on the different nodes:
1. pub_angles_from_colored_obj ROS node: is getting the turtlebot3 camera data via the '/usb_cam/image_raw' topic.
   it publishes the distance of the center of the blue object from the center of the (camera image) frame via the '/angles_from_colored_object' topic.
   it publishes a Float64 message, '0.0' is for centered and '900000' is for 'object not in frame'.
2. bot_move_control ROS node: is used for controlling the turtelbot3 movement not via the navigation stack.
   the node receives a MoveCommand message type on the '/move_bot_command' topic.
   MoveCommand has: 
    A.'direction' (string) field: can get values of 'counter-clock-wise-rotate' or 'forward'.
    B. 'distance' (float) field: that states the distance in degrees/meters the bot should move. 
    C.'speed' (float) field: in m/s (speed can be positive or nagative).
   the node translates these commands to the turtelbot3 'cmd_vel' topic commands.
3. task3 ROS node: is used to make sure we move the blue box.
   it does it by checking if the blue box is in the frame or by doing a full 360 degrees rotation every 20 seconds, for cases the turtelbot3 did not move towards the box directly.
   when the box is found the turtlebot3 stopps the navigation, pushes the box and then continue to his original destination.
   
   
   
   technical problems i encountered:
   1. the navigation stack was not able to move the bot. the reason was because the clocks of the PC and the bot not were not synchronized (and the difference was more than the 'transform_tolerance'). i tried setting the PC to be the NTP server of the bot, that did not work, so i manually changed the time of the bot to be like the PC (using '$ date +%s' and '$ date -s '@<epoch  time>').
    for it to work it is important to set the 'transform_tolerance' parameter to '15.0' so it won't be strict (in local_costmap_params.yaml and global_costmap_params.yaml).
    2. from some reason when i use the navigation stack the bot was not moving flowently, and it looked in the RVIz GUI like an inflation problem.
    setting the inflation to 0.0 and restarting the navigation did node did not change this state. 
    only when i checked it in a different day it worked, so may be this problem required full restart of the turtlebot/roscore etc.. (even though it does not make sense).
    3. when activating the navigation node using a map file the bot initially finds it's location but is not fully adjusted with the map walls (by the laser).
    after consulting with people that worked with ROS i found that causing the turtlebot to make 1-3 rounds in his place fixes the problem.
    
 


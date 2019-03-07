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
3. task3 ROS node


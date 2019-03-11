#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from task2.msg import MoveCommand
from std_msgs.msg       import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import move_base_msgs
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction

class task3:
    def callback_get_current_pose(self,data):
        self.pose = data

    def callback_get_current_goal(self,data):
        if not self.initFinalGoal:
            self.initFinalGoal = True
            self.FinalGoal = data
        self.currentGoal = data

    def __init__(self):
        rospy.init_node('task3', anonymous=False)
        self.move_client_init()
        self.initFinalGoal = False
        self.pose = PoseWithCovarianceStamped()
        self.pub_bot_commands = rospy.Publisher('/move_bot_command', MoveCommand, queue_size=10)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                                                self.callback_get_current_pose)
        rospy.Subscriber('/move_base/goal', move_base_msgs.msg.MoveBaseActionGoal, self.callback_get_current_goal)

    def stopBotNav(self):
        #self.move_client.cancel_all_goals()
        self.move_client.cancel_goals_at_and_before_time(rospy.Time.now() - rospy.Duration(2) )
        #self.sendMoveGoal(self.pose.pose.pose.position.x, self.pose.pose.pose.position.y)

    def navigateToFinalDestination(self):
        self.sendMoveGoal(self.FinalGoal.goal.target_pose.pose.position.x, self.FinalGoal.goal.target_pose.pose.position.y)

    def sendMoveGoal(self, x, y):
        rospy.loginfo('send goal() x:%f, y:%f',x,y)
        seq = 50
        if self.initFinalGoal:
            seq = self.currentGoal.goal.target_pose.header.seq + 1
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.seq = seq
        goal.target_pose.pose.position.x = x # x
        goal.target_pose.pose.position.y = y  # y
        goal.target_pose.pose.orientation.w = 1
        self.move_client.send_goal(goal)
        rospy.loginfo('send goal() goal sent')

    def move_client_init(self):
        self.move_client = actionlib.SimpleActionClient("move_base",
                                              move_base_msgs.msg.MoveBaseAction)
        self.move_client.wait_for_server(rospy.Duration(10))


class user_commands:
    def findObjectPutInCenter(self):
        currentAngle = 0
        rospy.loginfo('doing a circle to find object')
        while self.anglesFromObject > 100000 and currentAngle < 360:  # when object not in frame
            self.rotate(20)
            currentAngle = currentAngle + 20
            rospy.sleep(2.5)
        if self.anglesFromObject < 100000:#object in fram put object in center
            rospy.loginfo('object found')
            return True
        else:
            #self.rotate(-currentAngle)
            #rospy.sleep(3)
            print 'object not found'
            return False
    def pushObj(self):
        if not self.findObjectPutInCenter():
            rospy.loginfo('find obj is False')
            return False
        rospy.loginfo('find obj is True, moving to push it')
        result = False

        cmdForward = MoveCommand()
        cmdForward.direction = 'forward'
        cmdRotate = MoveCommand()
        cmdRotate.direction = 'counter-clock-wise-rotate'
        self.rotate(-15)
        rospy.sleep(2)
        rospy.loginfo('self.anglesFromObject:%f',self.anglesFromObject)
        while (abs(self.anglesFromObject) > 10 and abs(self.anglesFromObject) < 500) or self.forward_dist > 0.5:
            rospy.loginfo('rotate clockWise so object will bein the center, shift:%f', self.anglesFromObject)
            if abs(self.anglesFromObject) > 10:
                cmdRotate.distance = 8
                if self.anglesFromObject > 0:
                    rospy.loginfo('rotate clockWise so object will bein the center, shift:%f', self.anglesFromObject)
                    cmdRotate.speed = -0.2
                else:
                    cmdRotate.speed = 0.2
                    rospy.loginfo('rotate counterClockWise so object will bein the center, shift:%f',
                                  self.anglesFromObject)
            else:
                cmdRotate.distance = 0
                cmdRotate.speed = 0
            self.pub_bot_commands.publish(cmdRotate)
            rospy.sleep(2)
            if self.anglesFromObject < 100000 and self.anglesFromObject < 50 and self.forward_dist > 0.4:
                rospy.loginfo('moving to object, distance: %f', self.forward_dist)
                cmdForward.distance = 0.57
                cmdForward.speed = 0.03
                result = True
            #else:
             #   rospy.loginfo('self.anglesFromObject: %f, self.anglesFromObject:f, self.forward_dist:%f', self.anglesFromObject, self.forward_dist)
              #  cmdForward.distance = 0
               # cmdForward.speed = 0
            self.pub_bot_commands.publish(cmdForward)

            rospy.sleep(0.5)
        if result:
            rospy.sleep(6)
            cmdForward.distance = -0.45
            cmdForward.speed = -0.2
            self.pub_bot_commands.publish(cmdForward)
            rospy.sleep(4)
        return result


    def hitObject(self):
        # put object in frame
        while True:
            interval = 0
            direction = 0
            rospy.loginfo('starting auto move: trying to put object in frame')
            while self.anglesFromObject > 100000:  # when object not in frame
                interval = interval + 1
                if not interval % 2 == 0:
                    fullRound = 0
                    max = 0
                    while self.anglesFromObject > 100000 and (fullRound < 350 or self.forward_dist < 1.5):
                        rospy.loginfo(
                            'trying to put object in frame, self.anglesFromObject:%f, fullRound:%f, self.forward_dist:%f',
                            self.anglesFromObject, fullRound, self.forward_dist)
                        fullRound = fullRound + 30
                        if direction % 3 == 0:
                            self.rotate(30)
                        else:
                            self.rotate(-30)
                        rospy.sleep(2.5)
                        if self.anglesFromObject > max:
                            max = self.anglesFromObject
                        if fullRound > 359 and (max - 0.1) < self.anglesFromObject:
                            break

                else:
                    rospy.loginfo('trying to put object in frame, moving straight, self.forward_dist:%f',
                                  self.forward_dist)
                    direction = direction + 1
                    self.forward(8)
                    rospy.sleep(3)

            rospy.loginfo('auto move: object in frame, trying to go to it')
            cmdRotate = MoveCommand()
            cmdRotate.direction = 'counter-clock-wise-rotate'

            cmdForward = MoveCommand()
            cmdForward.direction = 'forward'
            self.pub_bot_commands.publish(cmdRotate)
            self.pub_bot_commands.publish(cmdForward)

            # go to object
            maxTimeToGoToObject = 20
            startGoingToObject = rospy.Time.now().to_sec()
            while (abs(self.anglesFromObject) > 10 and abs(self.anglesFromObject) < 500) or self.forward_dist > 0.5:
                if abs(self.anglesFromObject) > 10:
                    cmdRotate.distance = 5
                    if self.anglesFromObject > 0:
                        rospy.loginfo('rotate clockWise so object will bein the center, shift:%f',
                                      self.anglesFromObject)
                        cmdRotate.speed = -0.2
                    else:
                        cmdRotate.speed = 0.2
                        rospy.loginfo('rotate counterClockWise so object will bein the center, shift:%f',
                                      self.anglesFromObject)
                else:
                    cmdRotate.distance = 0
                    cmdRotate.speed = 0

                if self.forward_dist > 0.5:
                    rospy.loginfo('moving to object, distance: %f', self.forward_dist)
                    cmdForward.distance = 0.2
                    cmdForward.speed = 0.2
                else:
                    cmdForward.distance = 0
                    cmdForward.speed = 0

                # if bot is stuck (laser problems etc..)
                if (rospy.Time.now().to_sec() - startGoingToObject) > maxTimeToGoToObject:
                    cmdForward.distance = 5
                    cmdForward.speed = 1
                    cmdRotate.distance = 180
                    cmdRotate.speed = 1
                    self.pub_bot_commands.publish(cmdForward)
                    self.pub_bot_commands.publish(cmdRotate)
                    rospy.sleep(5)
                    cmdForward.distance = 0
                    cmdForward.speed = 0
                    cmdRotate.distance = 0
                    cmdRotate.speed = 0
                    break

                self.pub_bot_commands.publish(cmdForward)
                self.pub_bot_commands.publish(cmdRotate)
                rospy.sleep(0.2)
            # stop bot
            cmdRotate.distance = 0
            cmdRotate.speed = 0
            cmdForward.distance = 0
            cmdForward.speed = 0
            self.pub_bot_commands.publish(cmdForward)
            self.pub_bot_commands.publish(cmdRotate)
            if self.forward_dist < 0.6 and abs(self.anglesFromObject) < 400:
                break


    def forward(self, dist):
        cmd2 = MoveCommand()
        cmd2.direction = 'forward'
        cmd2.distance = dist
        cmd2.speed = 0.4
        self.pub_bot_commands.publish(cmd2)

    def forward_50cm(self):
        cmd = MoveCommand()
        cmd.direction = 'forward'
        cmd.distance = 0.5
        cmd.speed = 0.4
        self.pub_bot_commands.publish(cmd)

    def rotate(self, deg):
        rospy.loginfo('sending rotate command for %f deg', self.anglesFromObject)
        cmd = MoveCommand()
        cmd.direction = 'counter-clock-wise-rotate'
        cmd.distance = abs(deg)
        if deg > 0:
            cmd.speed = -0.3
        else:
            cmd.speed = 0.3
        rospy.loginfo('published ged:%f , speed: %f',cmd.distance, cmd.speed)
        self.pub_bot_commands.publish(cmd)

    def getDistFromColoredObj(self):
        rospy.sleep(0.2)
        if self.anglesFromObject > 100000:
            rospy.loginfo('No object with this color on fram')
            return None
        else:
            while abs(self.anglesFromObject) > 10:
                if self.anglesFromObject > 0:
                    rospy.loginfo('rotate clock-wise for object')
                    self.rotate(5)
                else:
                    rospy.loginfo('rotate counter-clock-wise for object')
                    self.rotate(-5)
                rospy.sleep(0.2)
            rospy.loginfo('Distance to object is:%f, shift from center is:%f', self.forward_dist, self.anglesFromObject)
            return self.forward_dist

    def autoMove(self):
#put object in frame
            rospy.loginfo('starting auto move: trying to put object in frame')
            startGoingToObject = rospy.Time.now().to_sec()
            checkEvery = 25
            wasPushed = False
            while not wasPushed:#when object not in frame
                if self.anglesFromObject < 100000:
                    rospy.loginfo('found object while going to destination')
                    rospy.sleep(3)
                    self.nav.stopBotNav()
                    wasPushed = self.pushObj()
                    self.nav.navigateToFinalDestination()
                    startGoingToObject = rospy.Time.now().to_sec()
                else:
                    if rospy.Time.now().to_sec() - startGoingToObject > checkEvery:
                        self.nav.stopBotNav()
                        rospy.loginfo('looking for object to push')
                        wasPushed = self.pushObj()
                        self.nav.navigateToFinalDestination()
                        if wasPushed:
                            rospy.loginfo('object was pushed')
                            break
                        else:
                            startGoingToObject = rospy.Time.now().to_sec()
                rospy.sleep(0.2)
            return wasPushed
    def callback_get_shift_from_colored_obj(self, data):
        self.anglesFromObject = data.data
        #rospy.loginfo('updated self.anglesFromObject:%f',self.anglesFromObject)

    def callback_update_dist(self, msg):
        self.lr = msg.ranges
        self.forward_dist = msg.ranges[0]

        #rospy.loginfo('msg.ranges[0]:%f', msg.ranges[0])

    def callback_get_current_bot_speed(self, data):
        self.current_bot_speed = data

    def __init__(self, navi):
        self.nav = navi
        self.anglesFromObject = 999999
        rospy.Subscriber('/scan', LaserScan, self.callback_update_dist)
        self.pub_bot_commands = rospy.Publisher('/move_bot_command', MoveCommand, queue_size=10)
        rospy.Subscriber('/cmd_vel_current', Twist, self.callback_get_current_bot_speed)
        self.pub_object_color = rospy.Publisher('/object_color', String, queue_size=10)

        self.pub_angles_from_colored_object = rospy.Publisher('/angles_from_colored_object', Float64, queue_size=10)

        self.sub_color_image = rospy.Subscriber('/angles_from_colored_object', Float64, self.callback_get_shift_from_colored_obj)

        self.sub_current_bot_vel = rospy.Subscriber('/cmd_vel_current', Twist, queue_size=10)
        self.sub_color_image = rospy.Subscriber('/angles_from_colored_object', Float64)
if __name__ == '__main__':
    t3 = task3()
    t3.stopBotNav()
    userCom = user_commands(t3)
    res = userCom.autoMove()
    print 'result is:', res
    #while True:
     #   print "click a to send"
      #  input = raw_input()
       # if input == 'stop':
        #    t3.stopBotNav()
        #if input == '0':
        #    t3.sendMoveGoal(0,0)
        #if input == 'go':
         #   t3.navigateToFinalDestination()
            #t3.sendMoveGoal(0,0)
        #else:
         #   rospy.sleep(1)



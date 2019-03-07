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
    '''this is the class the manages comm. with the navigation stack'''

    #will get goals set to the navigation stack, first when running task3 is the final destination goal
    def callback_get_current_goal(self,data):
        if not self.initFinalGoal:
            self.initFinalGoal = True
            self.FinalGoal = data
        self.currentGoal = data

    def __init__(self):
        rospy.init_node('task3', anonymous=False)
        self.move_client_init()
        self.initFinalGoal = False
        self.pub_bot_commands = rospy.Publisher('/move_bot_command', MoveCommand, queue_size=10)
        rospy.Subscriber('/move_base/goal', move_base_msgs.msg.MoveBaseActionGoal, self.callback_get_current_goal)

    def stopBotNav(self):
        '''cancel navigation stack goals, it is used when we want to navigate manually'''
        self.move_client.cancel_goals_at_and_before_time(rospy.Time.now() - rospy.Duration(2) )

    def navigateToFinalDestination(self):
        '''sends the final destination goal to the navigation stack'''
        self.sendMoveGoal(self.FinalGoal.goal.target_pose.pose.position.x, self.FinalGoal.goal.target_pose.pose.position.y)

    def sendMoveGoal(self, x, y):
        '''sends a navigation goal to navigation stack'''
        rospy.loginfo('send goal() x:%f, y:%f',x,y)

        #the navigation stack does not navigate to past goals so we need to increment the sequence counter
        seq = 50
        if self.initFinalGoal:#if no goals received stay we default value of 50 else increment seq
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
        '''init the navigation actions client'''
        self.move_client = actionlib.SimpleActionClient("move_base",
                                              move_base_msgs.msg.MoveBaseAction)
        self.move_client.wait_for_server(rospy.Duration(10))


class user_commands:
    def findObjectPutInCenter(self):
        '''will do a 360 deg turn and will return if blue box object was found'''
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
        '''will push blue box object if it has a clear line to it'''
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
            self.pub_bot_commands.publish(cmdForward)

            rospy.sleep(0.5)
        if result:
            rospy.sleep(6)
            cmdForward.distance = -0.45
            cmdForward.speed = -0.2
            self.pub_bot_commands.publish(cmdForward)
            rospy.sleep(4)
        return result

    def forward(self, dist):
        '''sends a forward command to the node that moves the bot'''
        cmd2 = MoveCommand()
        cmd2.direction = 'forward'
        cmd2.distance = dist
        cmd2.speed = 0.4
        self.pub_bot_commands.publish(cmd2)

    def rotate(self, deg):
        '''sends a rotate command to the node that moves the bot'''
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

    def autoMove(self):
            '''this method manages the movement of the robot:
            the robot is navigating to final destination
            every 25 seconds stop the robot navigation and look for the blue box and push it,
            if the blue box is found while navigating then push it
            if box was pushed navigate to final destination'''
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
        '''getting data on the location of the box in the usb-camera frame'''
        self.anglesFromObject = data.data
        #rospy.loginfo('updated self.anglesFromObject:%f',self.anglesFromObject)

    def callback_update_dist(self, msg):
        '''updating the data comming from the laser'''
        self.lr = msg.ranges
        self.forward_dist = msg.ranges[0]

    def __init__(self, navi):
        self.nav = navi
        self.anglesFromObject = 999999
        rospy.Subscriber('/scan', LaserScan, self.callback_update_dist)
        self.pub_bot_commands = rospy.Publisher('/move_bot_command', MoveCommand, queue_size=10)
        self.pub_object_color = rospy.Publisher('/object_color', String, queue_size=10)

        self.pub_angles_from_colored_object = rospy.Publisher('/angles_from_colored_object', Float64, queue_size=10)

        rospy.Subscriber('/angles_from_colored_object', Float64, self.callback_get_shift_from_colored_obj)
        self.sub_current_bot_vel = rospy.Subscriber('/cmd_vel_current', Twist, queue_size=10)

if __name__ == '__main__':
    #create the communication to navigation stack object
    t3 = task3()

    #stop old navigation goal
    t3.stopBotNav()

    #init driver object with the navigation communication object
    userCom = user_commands(t3)

    #start the auto move control
    res = userCom.autoMove()



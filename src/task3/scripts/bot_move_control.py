#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from task2.msg import MoveCommand
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

distance_to_stop_in_meter = 0.0#because we are pushing the box we allow distance to be 0
class bot_move_control:
    '''this node gets commands on moving the bot and it translate them to actual commands for the turtlebot'''
    def callback_update_dist(self, msg):
        self.forward_dist = msg.ranges[0]

    def callback_move_bot_commands(self, msg):
        rospy.loginfo('move command received')
        if msg.direction == 'forward':
            rospy.loginfo('command received is :forward')
            self.forwardNeededDistance = abs(msg.distance)
            self.forwardSpeed = msg.speed
            self.updateForward = True
            self.doForward = True

        if msg.direction == 'counter-clock-wise-rotate':
            self.rotateNeededAngle = msg.distance * 2 * math.pi / 360
            self.rotateSpeed = msg.speed
            rospy.loginfo('command received is :rotate, deg:%f, speed:%f', self.rotateNeededAngle, self.rotateSpeed)
            self.updateRotate = True
            self.doRotate = True

    def executeMoveCommandsLoop(self):
        while True:
            if self.updateForward:
                rospy.loginfo('update cmd_vel: by forward command')
                self.updateForward = False
                self.forwardStartTime = rospy.Time.now().to_sec()
                self.forwardCurrentDistance = 0
            if self.updateRotate:
                rospy.loginfo('update cmd_vel: by rotate command')
                self.updateRotate = False
                self.roatetStartTime = rospy.Time.now().to_sec()
                self.rotateCurrentDistance = 0
            currentTime = rospy.Time.now().to_sec()
            currentAngle = abs(self.rotateSpeed * (self.roatetStartTime - currentTime))
            currentForwardDistance = abs(self.forwardSpeed * (self.forwardStartTime - currentTime))  # Calculates distancePoseStamped
            if currentForwardDistance < abs(self.forwardNeededDistance) and (self.forwardSpeed < 0 or distance_to_stop_in_meter < self.forward_dist):
                self.vel_msg.linear.x = self.forwardSpeed
            else:
                self.vel_msg.linear.x = 0.0

            if currentAngle < self.rotateNeededAngle:
                rospy.loginfo('current angle:%f,  needed angle:%f',currentAngle, self.rotateNeededAngle)
                self.vel_msg.angular.z = self.rotateSpeed
            else:
                self.vel_msg.angular.z = 0.0

            if self.doForward or self.doRotate:
                self.pub_cmd_vel.publish(self.vel_msg)  # Publish the velocity
            if self.vel_msg.linear.x == 0.0:
                self.doForward = False
            if self.vel_msg.angular.z == 0.0:
                self.doRotate = False
            self.pub_current_cmd_vel.publish(self.vel_msg)
            rospy.sleep(0.1)

    def __init__(self):
        self.doRotate = False
        self.doForward = False
        self.forwardNeededDistance = 0.0
        self.rotateNeededAngle = 0.0
        self.forwardStartTime = rospy.Time.now().to_sec()
        self.roatetStartTime = rospy.Time.now().to_sec()
        self.vel_msg = Twist()
        self.updateRotate = False
        self.updateForward = False
        self.rotateSpeed = 0.0
        self.forwardSpeed = 0.0
        rospy.loginfo('ros move bot node started')
        self.commands = []
        self.forward_dist = 0.0
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_current_cmd_vel = rospy.Publisher('/cmd_vel_current', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.callback_update_dist)
        rospy.Subscriber('/move_bot_command', MoveCommand, self.callback_move_bot_commands)
        self.executeMoveCommandsLoop()

if __name__ == '__main__':
    rospy.init_node('bot_move_control', anonymous=True)
    mover = bot_move_control()
    rospy.spin()

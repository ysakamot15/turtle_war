#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import random
import time

from abstractBot import *
from geometry_msgs.msg import Twist

STATE_ROLL = 1
STATE_LINE = 2

class RandomBot(AbstractBot):
    
    def strategy(self):
        r = rospy.Rate(100)
        
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        surplus = 0

        UPDATE_FREQUENCY = 1
        update_time = 0
        state = STATE_ROLL
        count = 0
        x = 0
        th = 3
        while not rospy.is_shutdown():

            update_time = time.time()
            if self.center_bumper or self.left_bumper or self.right_bumper:
                
                rospy.loginfo('bumper hit!!')
                state = STATE_ROLL
                x = -1
                y = 0
                th = 0
                #control_speed = -1
                #control_turn = 0
            
            else: # time.time() - update_time > UPDATE_FREQUENCY:
                update_time = time.time()
                if state == STATE_ROLL:
                    th = 3
                    x = 0
                    y = 0
                    count += 1
                    if count >= 30:
                        print "ToLine"
                        count = 0
                        state = STATE_LINE
                else:
                    count += 1
                    x = 1
                    #if x == 1:
                    #    x = -1
                    #else:
                    #    x = 1
                    
                    th = 0
                    if  count == 800:
                        print "ToRound"
                        count = 0
                        state = STATE_ROLL
                    
                

            target_speed = x
            target_turn = th
            control_speed = x
            target_turn = th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            #print(twist)
        
            self.vel_pub.publish(twist)

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('random_bot')
    bot = RandomBot('Random')
    bot.strategy()

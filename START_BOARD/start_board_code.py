#!/usr/bin/env python


import sys

import random

import rospy

from std_msgs.msg import String

import time

raspberry = True

pi = None
#
 If passed the -r argument, load the rapsberry libs
if raspberry:
    import pigpio
    pi = pigpio.pi()

rospy.init_node('startup_conf')

pub_run = rospy.Publisher('run', String, queue_size=1)

#
#   GPIO PIN CONFIGURATIONS
#
# This pin will be used to configure thame
pin_team = rospy.get_param("/pins/team_switch")
# This pin will be used to drive a led to indicate that the team has been set correctly in ROS
pin_team_feedback = rospy.get_param("/pins/team_feedback_led")

# This pin will be used to drive a led to indicate that the strategy has been set correctly in ROS
pin_strategy_feedback_1 = rospy.get_param("/pins/strategy_feedback_led_1")
pin_strategy_feedback_2 = rospy.get_param("/pins/strategy_feedback_led_2")

pin_initialization_switch = rospy.get_param("/pins/initialization_switch")
pin_initialization_feedback = rospy.get_param("/pins/initialization_feedback_led")

# This pin will be used to launch robot
pin_start = rospy.get_param("/pins/start")

def update_team(gpio, level, tick):
    if raspberry:
        if level:
           
 rospy.set_param("/team", "red")
            pi.write(pin_team_feedback, 1)
        else :
            rospy.set_param("/team", "green")
            # Blink 2 times for acknowledge
            pi.write(pin_team_feedback, 0)
    else:
        choice = random.choice(["green", "red"])
        team = rospy.set_param("/team", choice)
        rospy.logwarn("Setting team automatically to: " + choice)

def initi(gpio, level, tick):
    if raspberry:
        if not pi.read(pin_initialization_switch):
            global publish
            publish = True
            pi.write(pin_initialization_feedback, 1)
            time.sleep(.500)
            pi.write(pin_initialization_feedback, 0)
        else :
            pi.write(pin_initialization_feedback, 0)

def end_of_game_cb(event):
    # Stop the game
    rospy.loginfo("End of game")
    rospy.loginfo(event)

    pub_run.publish("stop")

publish = True

def start(gpio, level, tick):
    global publish
    rospy.sleep(0.2)
    if pi.read(pin_start) and publish:
        pi.write(pin_strategy_feedback_1, 1)
        pub_run.publish("start")
        rospy.Timer(rospy.Duration(5), end_of_game_cb, oneshot=True)
        publish = False

if raspberry:
    pi.set_mode(pin_initialization_switch, pigpio.INPUT)
    pi.set_mode(pin_initialization_feedback, pigpio.OUTPUT)

    pi.set_mode(pin_team, pigpio.INPUT)
    pi.set_mode(pin_team_feedback, pigpio.OUTPUT)

    pi.set_mode(pin_strategy_feedback_1, pigpio.OUTPUT)
    pi.set_pull_up_down(pin_strategy_feedback_1, pigpio.PUD_DOWN)

    pi.set_mode(pin_strategy_feedback_2, pigpio.OUTPUT)
    pi.set_pull_up_down(pin_strategy_feedback_2, pigpio.PUD_DOWN)

    pi.set_mode(pin_start, pigpio.INPUT)
    pi.set_pull_up_down(pin_start, pigpio.PUD_UP)

    teamInterrupt = pi.callback(pin_team, pigpio.EITHER_EDGE, update_team)
    startInterrupt = pi.callback(pin_start, pigpio.EITHER_EDGE, start)
    initInterrupt = pi.callback(pin_initialization_switch, pigpio.EITHER_EDGE, initi)
    update_team(None, pi.read(pin_team), None)

else:
    rospy.sleep(1)
    update_team(None, None, None)

while not rospy.is_shutdown():
    pass

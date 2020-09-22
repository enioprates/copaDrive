#!/usr/bin/env python

import rospy
from prius_msgs.msg import Control
import curses
forward = 0;
left = 0;

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyboard_talker', anonymous=True)
self.pub = rospy.Publisher('car_1/prius', Control, queue_size=1)

stdscr.refresh()

key = ''
while key != ord('q'):
    key = stdscr.getch()
    stdscr.refresh()

    command = Control()    

    # fill in the conditions to increment/decrement throttle/steer

    if key == curses.KEY_UP:
        command.throttle = message.axes[THROTTLE_AXIS]
        command.brake = 0.0
        command.shift_gears = Control.FORWARD
    elif key == curses.KEY_DOWN:
        forward = forward - 1
    elif key == curses.KEY_LEFT:
        left = left + 1
    elif key == curses.KEY_RIGHT:
        left = left - 1
    elif key == ord(' '):
        # this key will center the steer and throttle
        left = 0
        forward = 0

    pub.publish(command)

    curses.endwin()
#! /usr/bin/env python

from actionlib import SimpleActionClient
from geometry_msgs.msg import Twist
from rospy import resolve_name

from yamabros.msg import CommandAction, CommandGoal
from yamabros.topic import Topic


# Command codes
(APPROACH, BRAKE, CIRCLE, COAST, POSE, SPIN, TURN) = range(7)

# Mode codes
(BLOCK, ASYNC, DEFAULT) = range(3)


class Spur(object):
    def __init__(self, name='/spur', mode=BLOCK):
        self.mode = mode
        self.client = SimpleActionClient(resolve_name(name), CommandAction)
        self.client.wait_for_server()
        self.cmd_vel = Topic('cmd_vel', Twist)

    def __del__(self):
        del self.client
        del self.cmd_vel

    def send(self, command, mode=ASYNC, *arguments):
        goal = CommandGoal()
        goal.command = command
        goal.destination.extend(arguments)
        self.client.send_goal(goal)
        if mode == BLOCK or (mode == DEFAULT and self.mode == BLOCK):
            self.client.wait_for_result()

    def brake(self):
        self.send(BRAKE)

    def coast(self):
        self.send(COAST)

    def steer(self, v, w):
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel.publish(twist)

    def straight(self, d=0.1, mode=DEFAULT):
        self.approach(d, 0, None, mode)

    def approach(self, x, y, t=None, mode=DEFAULT):
        arguments = [x, y]
        if t != None:
            arguments.append(t)

        self.send(APPROACH, mode, *arguments)

    def circle(self, x, y, r, t=None, e=None, mode=DEFAULT):
        arguments = [x, y, r]
        if t != None:
            arguments.append(t)
        if e != None:
            arguments.append(e)

        self.send(CIRCLE, mode, *arguments)

    def spin(self, t, mode=DEFAULT):
        self.send(SPIN, mode, t)

    def spin(self, t, e=None, mode=DEFAULT):
        arguments = [t]
        if e != None:
            arguments.append(e)

        self.send(SPIN, mode, *arguments)

    def turn(self, t, e=None, mode=DEFAULT):
        arguments = [t]
        if e != None:
            arguments.append(e)

        self.send(TURN, mode, *arguments)

    def pose(self):
        self.send(POSE, BLOCK)
        return self.client.get_result().pose

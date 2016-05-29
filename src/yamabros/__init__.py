#! /usr/bin/env python

import rospy

import yamabros.hokuyo
import yamabros.spur


class Attributes(object):
    def __setattr__(self, name, value):
        object.__setattr__(self, name, value)


class Robot(Attributes):
    def __init__(self, name='yamabico_robot'):
        self.name = name

    def init_node(self):
        r'''Convenience method for starting a ROS node.
        '''
        rospy.init_node(self.name)

    def param(self, name, fallback=None):
        r'''Convenience method for getting system parameters.
        '''
        return rospy.get_param(name, fallback)

    def spin(self):
        rospy.spin()


# Alias for hokuyo.Hokuyo
Hokuyo = hokuyo.Hokuyo


# Alias for spur.Spur
Spur = spur.Spur

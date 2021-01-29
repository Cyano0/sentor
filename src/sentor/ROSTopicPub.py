#!/usr/bin/env python
"""
@author: Francesco Del Duchetto (FDelDuchetto@lincoln.ac.uk)
@author: Adam Binch (abinch@sagarobotics.com)

"""
#####################################################################################
import rospy


class ROSTopicPub(object):

    def __init__(self, topic_name, throttle_val):

        self.topic_name = topic_name
        self.pub_callbacks = []
        self.throttle_val = throttle_val
        self.throttle = self.throttle_val

    def callback_pub(self, msg):

        for func in self.pub_callbacks:
            func("'published'")
            
    def callback_pub_throttled(self, msg):
        
        if (self.throttle % self.throttle_val) == 0:
            self.callback_pub(msg)
            self.throttle = 1
        else:
            self.throttle += 1

    def register_published_cb(self, func):

        self.pub_callbacks.append(func)
#####################################################################################
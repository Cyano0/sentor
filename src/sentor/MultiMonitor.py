#!/usr/bin/env python
"""
Created on Fri Dec  6 08:51:15 2019

@author: Adam Binch (abinch@sagarobotics.com)
"""
#####################################################################################
from __future__ import division
import rospy
from sentor.msg import Monitor, MonitorArray
from threading import Event


class MultiMonitor(object):
    
    
    def __init__(self, rate=10):
        
        self.topic_monitors = []
        self.conditions = MonitorArray()
        self._stop_event = Event()
        
        self.pub = rospy.Publisher("/sentor/monitors", MonitorArray, latch=True, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/rate), self.cb)


    def register_monitors(self, topic_monitor):
        self.topic_monitors.append(topic_monitor)
        
        
    def cb(self, event=None):
        
        if not self._stop_event.isSet():

            conditions_new = MonitorArray()
            conditions_new.header.stamp = rospy.Time.now()
            
            for monitor in self.topic_monitors:
                for expr in monitor.crit_conditions:
                    condition = Monitor()
                    condition.topic = monitor.topic_name
                    condition.expression = expr
                    condition.safe = monitor.crit_conditions[expr]
                    conditions_new.monitors.append(condition)

            if self.conditions.monitors != conditions_new.monitors:
                self.conditions = conditions_new
                self.pub.publish(self.conditions)
                
                
    def stop_monitor(self):
        self._stop_event.set()


    def start_monitor(self):
        self._stop_event.clear()
#####################################################################################
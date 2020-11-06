#!/usr/bin/env python
"""
Created on Fri Dec  6 08:51:15 2019

@author: Adam Binch (abinch@sagarobotics.com)
"""
#####################################################################################
from __future__ import division
import rospy
from sentor.msg import Monitor, MonitorArray, ErrorCode
from threading import Event


class MultiMonitor(object):
    
    
    def __init__(self, rate=10):
        
        self.topic_monitors = []
        self._stop_event = Event()
        self.error_code = []
        
        self.monitors_pub = rospy.Publisher("/sentor/monitors", MonitorArray, latch=True, queue_size=1)
        self.error_code_pub = rospy.Publisher("/sentor/error_code", ErrorCode, latch=True, queue_size=1)
        
        rospy.Timer(rospy.Duration(1.0/rate), self.cb)


    def register_monitors(self, topic_monitor):
        self.topic_monitors.append(topic_monitor)
        
        
    def cb(self, event=None):
        
        if not self._stop_event.isSet():
            
            error_code_new = [monitor.crit_conditions[expr] for monitor in self.topic_monitors for expr in monitor.crit_conditions]
            
            if error_code_new != self.error_code:
                self.error_code = error_code_new
                
                conditions = MonitorArray()
                conditions.header.stamp = rospy.Time.now()
                
                error_code = ErrorCode()
                error_code.error_code = map(int,self.error_code)

                count = 0                
                for monitor in self.topic_monitors:
                    for expr in monitor.crit_conditions:
                        condition = Monitor()
                        condition.topic = monitor.topic_name
                        condition.expression = expr
                        condition.safe = self.error_code[count]
                        conditions.monitors.append(condition)
                        count+=1
                        
                self.monitors_pub.publish(conditions)
                self.error_code_pub.publish(error_code)
                        
                
    def stop_monitor(self):
        self._stop_event.set()


    def start_monitor(self):
        self._stop_event.clear()
#####################################################################################
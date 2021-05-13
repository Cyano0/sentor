#!/usr/bin/env python
"""
@author: Francesco Del Duchetto (FDelDuchetto@lincoln.ac.uk)
@author: Adam Binch (abinch@sagarobotics.com)
"""
##########################################################################################
from __future__ import division
from sentor.TopicMonitor import TopicMonitor
from sentor.SafetyMonitor import SafetyMonitor
from sentor.MultiMonitor import MultiMonitor
from std_msgs.msg import String
from sentor.msg import SentorEvent
from std_srvs.srv import Empty, EmptyResponse
import pprint
import signal
import rospy
import time
import yaml
import sys
import os

# TODO nice printing of frequency of the topic with curses
# TODO consider timeout

unpublished_topics_indexes = []
satisfied_filters_indexes = []

topic_monitors = []

event_pub = None
rich_event_pub = None

def __signal_handler(signum, frame):
    def kill_monitors():
        for topic_monitor in topic_monitors:
            topic_monitor.kill_monitor()
        safety_monitor.stop_monitor()
        multi_monitor.stop_monitor()
    def join_monitors():
        for topic_monitor in topic_monitors:
            topic_monitor.join()
    kill_monitors()
    join_monitors()
    print "stopped."
    os._exit(signal.SIGTERM)
    

def stop_monitoring(_):
    for topic_monitor in topic_monitors:
        topic_monitor.stop_monitor()
        
    safety_monitor.stop_monitor()
    multi_monitor.stop_monitor()

    rospy.logwarn("sentor_node stopped monitoring")
    ans = EmptyResponse()
    return ans
    

def start_monitoring(_):    
    for topic_monitor in topic_monitors:
        topic_monitor.start_monitor()

    safety_monitor.start_monitor()
    multi_monitor.start_monitor()

    rospy.logwarn("sentor_node started monitoring")
    ans = EmptyResponse()
    return ans
    

def event_callback(string, type, msg="", nodes=[], topic=""):
    if type == "info":
        rospy.loginfo(string + '\n' + str(msg))
    elif type == "warn":
        rospy.logwarn(string + '\n' + str(msg))
    elif type == "error":
        rospy.logerr(string + '\n' + str(msg))

    if event_pub is not None:
        event_pub.publish(String("%s: %s" % (type, string)))

    if rich_event_pub is not None:
        event = SentorEvent()
        event.header.stamp = rospy.Time.now()
        event.level = SentorEvent.INFO if type == "info" else SentorEvent.WARN if type == "warn" else SentorEvent.ERROR
        event.message = string
        event.nodes = nodes
        event.topic = topic
        rich_event_pub.publish(event)
##########################################################################################
    

##########################################################################################
if __name__ == "__main__":
    signal.signal(signal.SIGINT, __signal_handler)
    rospy.init_node("sentor")

    config_file = rospy.get_param("~config_file", "")
    try:
        items = [yaml.load(file(item, 'r')) for item in config_file.split(',')]
        topics = [item for sublist in items for item in sublist]
    except Exception as e:
        rospy.logerr("No configuration file provided: %s" % e)
        topics = []

    stop_srv = rospy.Service('/sentor/stop_monitor', Empty, stop_monitoring)
    start_srv = rospy.Service('/sentor/start_monitor', Empty, start_monitoring)

    event_pub = rospy.Publisher('/sentor/event', String, queue_size=10)
    rich_event_pub = rospy.Publisher('/sentor/rich_event', SentorEvent, queue_size=10)

    safe_operation_timeout = rospy.get_param("~safe_operation_timeout", 10.0)    
    safety_pub_rate = rospy.get_param("~safety_pub_rate", 10.0)    
    auto_safety_tagging = rospy.get_param("~auto_safety_tagging", True)        
    safety_monitor = SafetyMonitor(safe_operation_timeout, safety_pub_rate, auto_safety_tagging, event_callback) 
    
    multi_monitor = MultiMonitor()

    topic_monitors = []
    print "Monitoring topics:"
    for i, topic in enumerate(topics):
        
        include = True
        if 'include' in topic:
            include = topic['include']
            
        if not include:
            continue
        
        try:
            topic_name = topic["name"]
        except Exception as e:
            rospy.logerr("topic name is not specified for entry %s" % topic)
            continue

        rate = 0
        N = 0
        signal_when = {}
        signal_lambdas = []
        processes = []
        timeout = 0
        default_notifications = True
        
        if 'rate' in topic:
            rate = topic['rate']
        if 'N' in topic:
            N = int(topic['N'])
        if 'signal_when' in topic:
            signal_when = topic['signal_when']
        if 'signal_lambdas' in topic:
            signal_lambdas = topic['signal_lambdas']
        if 'execute' in topic:
            processes = topic['execute']
        if 'timeout' in topic:
            timeout = topic['timeout']
        if 'default_notifications' in topic:
            default_notifications = topic['default_notifications']

        topic_monitor = TopicMonitor(topic_name, rate, N, signal_when, signal_lambdas, processes, 
                                     timeout, default_notifications, event_callback, i)

        topic_monitors.append(topic_monitor)
        safety_monitor.register_monitors(topic_monitor)
        multi_monitor.register_monitors(topic_monitor)
            
    time.sleep(1)

    # start monitoring
    for topic_monitor in topic_monitors:
        topic_monitor.start()

    rospy.spin()
##########################################################################################
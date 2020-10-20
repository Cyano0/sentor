#!/usr/bin/env python
"""
@author: Adam Binch (abinch@sagarobotics.com)
"""
##########################################################################################
from __future__ import division
from sentor.TopicMapper import TopicMapper
from sentor.TopicMapServer import TopicMapServer
from std_srvs.srv import Empty, EmptyResponse

import signal
import rospy
import time
import yaml
import os
##########################################################################################


##########################################################################################
def __signal_handler(signum, frame):
    def kill_mappers():
        for topic_mapper in topic_mappers:
            topic_mapper.kill_mapper()
    def join_mappers():
        for topic_mapper in topic_mappers:
            topic_mapper.join()
    kill_mappers()
    join_mappers()
    print "stopped."
    os._exit(signal.SIGTERM)
    

def stop_mapping(_):
    topic_map_server.stop()

    rospy.logwarn("topic_mapping_node stopped mapping")
    ans = EmptyResponse()
    return ans
    

def start_mapping(_):
    topic_map_server.start()

    rospy.logwarn("topic_mapping_node started mapping")
    ans = EmptyResponse()
    return ans
##########################################################################################
    

##########################################################################################
if __name__ == "__main__":
    signal.signal(signal.SIGINT, __signal_handler)
    rospy.init_node("topic_mapper")

    config_file = rospy.get_param("~config_file", "")

    try:
        topics = yaml.load(file(config_file, 'r'))
    except Exception as e:
        rospy.logerr("No configuration file provided: %s" % e)
        topics = []

    stop_srv = rospy.Service('/sentor/stop_mapping', Empty, stop_mapping)
    start_srv = rospy.Service('/sentor/start_mapping', Empty, start_mapping)

    topic_mappers = []
    print "Mapping topics:"
    for topic in topics:
        try:
            topic_name = topic["name"]
        except Exception as e:
            rospy.logerr("topic name is not specified for entry %s" % topic)
            continue

        include = True
        if "include" in topic.keys():
            include = topic["include"]

        if include:
            topic_mappers.append(TopicMapper(topic))
            
    time.sleep(1)
    
    map_pub_rate = rospy.get_param("~map_pub_rate", 0) 
    map_plt_rate = rospy.get_param("~map_plt_rate", 0) 
    topic_map_server = TopicMapServer(topic_mappers, map_pub_rate, map_plt_rate)

    # start mapping
    for topic_mapper in topic_mappers:
        topic_mapper.start()

    rospy.spin()
##########################################################################################
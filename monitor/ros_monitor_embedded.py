#!/usr/bin/python
import subprocess
import os
import time
import logging

import rosgraph
import rosnode
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

#
# This library is experiment-specific
# Each class should have methods to monitor the experiment and should return a status
#

class EmbeddedMonitor:
    def __init__(self, required_node,subscriber_topic):
        self.create_logger()
        self.logger.info("Starting Embedded Monitor instance")

        self.required_node = required_node
        self.logger.debug(f"Required node: {required_node}")

        self.subscriber_topic = subscriber_topic
        self.logger.debug(f"Subscriber topic: {subscriber_topic}")

        self.rospy = rospy
        self.rospy.init_node("embeddedmonitor")#, anonymous = True)
        self.logger.info("Init node embeddedmonitor")
        
        self.status_subscriber()
        
        # Status: { "Not Started", "Started", "Error", "Finished"}
        self.ros_status, self.trial_status = "Not Started", "Not Started"
        self.status = "Not Started"
        # Flags indicate if status was once 'Started' to monitor for errors
        self.ros_status_flag = False

    # We'll assume that the ROS system is up in the embedded experiment if:
    # rosmaster is up;
    # required node is up; 
    # This is not foul proof but should work
    def check_ros(self):
        if rosgraph.is_master_online(): 
            node_list = rosnode.get_node_names()
            # Be careful here: if the node name is anonymous, this will break
            req_node = [True if node in node_list else False for node in self.required_node]
            if all(req_node):
                self.ros_status = "Started"
                self.ros_status_flag = True
        # If the status was once "Started" and 
        # the ros master is not up anymore, something happened.
        # Ideally we would be parsing the launch subprocess' stderr 
        elif self.ros_status_flag:
            self.ros_status = "Error" 

    # Subscribe to a topic that points the status and monitor its value
    def status_subscriber(self):
        self.logger.info("Initializing subscriber")
        self.sub_exp_status = self.rospy.Subscriber(self.subscriber_topic, String, self.callback_exp_status)
    
    def callback_exp_status(self, msg):
        self.status = msg.data
        self.logger.debug(f"Subscriber callback - msg arrived: {self.status}")

    def check_trial(self):
        if self.trial_status == "Not Started": 
            if self.status == "Started": 
                self.trial_status = "Started"
        elif self.trial_status == "Started":
            if self.status == "Finished": 
                self.trial_status = "Finished"

    def update_status(self):
        self.check_ros()
        self.check_trial()
        self.logger.debug(f"ROS status: {self.ros_status}, Trial status: {self.trial_status}")
        return self.ros_status, self.trial_status

    def create_logger(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s:%(name)s:%(levelname)s:%(message)s')

        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(formatter)
        stream_handler.setLevel(logging.INFO)
        
        self.logger.addHandler(stream_handler)

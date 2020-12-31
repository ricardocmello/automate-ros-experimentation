#!/usr/bin/python
import signal
import subprocess
import os
import time
import psutil
import rosgraph
import rosnode
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

import logging

class GzStatsMonitor:
    def __init__(self, name, trial):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s:%(name)s:%(levelname)s:%(message)s')

        file_handler = logging.FileHandler('./logs/{}-gzstats.log'.format(time.strftime("%y%m%d-%H%M%S")))
        file_handler.setFormatter(formatter)
        file_handler.setLevel(logging.DEBUG)

        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(formatter)
        stream_handler.setLevel(logging.INFO)

        self.logger.addHandler(file_handler)
        self.logger.addHandler(stream_handler)
        
        self.logger.info("Starting gz stats monitor")

        gzstats_file_name = "_".join([str(name), str(trial), "gzstats.txt"])
        self.logger.info(f"Saving to file {gzstats_file_name}")
        self.cmd = "gz stats --verbose -p | ts '%s, ' > ./files/" + gzstats_file_name
        self.gz_stats_started = self.check_gz()

    def __del__(self):
        #self.logger.debug("Deleting instance of gz stats object (on __del__())")
        self.close()

    def close(self):
        #self.logger.debug("In close()")
        try:
            #self.logger.debug("Sending SIGINT to gz subprocess")
            while self.gz.poll() == None: 
                os.killpg(os.getpgid(self.gz.pid), signal.SIGINT)
                time.sleep(0.5)
                #if self.gz.poll() != None:
                #    self.logger.debug("gz subprocess closed")
        except:
            #self.logger.exception("Exception when closing gzstats")
            pass

    def check_gz(self):
        self.logger.debug("Checking if gazebo has already started")
        gz_started = False
        while not gz_started:
            for proc in psutil.process_iter():
                if "gzserver" in proc.name():
                    gz_started = True
                    self.logger.debug(f"Found string gzserver in proc.name {proc.name()}")
                    break
            time.sleep(0.2)
        return self.start_gzstats()
    
    def start_gzstats(self):
        self.logger.info("Starting gz stats monitor subprocess")
        try:
            self.gz = subprocess.Popen(self.cmd, shell=True, preexec_fn=os.setpgrp)
            self.logger.info("Started gz stats monitor subprocess")
            return True
        except:
            self.logger.exception("Failed to start gz stats monitor subprocess")
            return False
    
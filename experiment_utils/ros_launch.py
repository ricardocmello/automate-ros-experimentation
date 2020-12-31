#!/usr/bin/python
import subprocess
import os
import time
import psutil
import signal
import rosgraph
import logging

# Library with experiment-specific functions
from monitor import EmbeddedMonitor

class ROSLaunch:
    def __init__(self, config):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s:%(name)s:%(levelname)s:%(message)s')

        file_handler = logging.FileHandler('./logs/{}-launch.log'.format(time.strftime("%y%m%d-%H%M%S")))
        file_handler.setFormatter(formatter)
        file_handler.setLevel(logging.DEBUG)

        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(formatter)
        stream_handler.setLevel(logging.INFO)

        self.logger.addHandler(file_handler)
        self.logger.addHandler(stream_handler)
        
        self.logger.info("Starting {}".format(str(__name__)))

        # Parse config
        pkg  = config["pkg"]
        launchfile  = config["launchfile"]
        ros_setup_script  = config["ros_setup_script"]
        ros_launch_path  = config["ros_launch_path"]
        required_node = config["ros_required_node"]
        monitor_subscriber_topic = config["monitor_subscriber_topic"]

                
        self.launch(pkg, launchfile, ros_setup_script, ros_launch_path)
        self.monitor = globals()[config["monitor"]](required_node, monitor_subscriber_topic) 
        
        self.ros_status = ''
        self.update_status()
        
      
    def close(self):        
        #self.logger.debug("In close()")
        try:
            ## Kill launch
            self.close_launch()
            ## Check for alive procs
            self.check_ros_proc()
        except AttributeError:
            #self.logger.debug("Name error in close()")
            pass
        # Force kill ros
        self.force_kill()     

    # Start ROS launch
    def launch(self, pkg, launchfile, ros_setup_script, ros_launch_path):
        command = ' '.join((ros_launch_path, pkg, launchfile))
        self.logger.debug(f"Starting roslaunch with command: {command}")
        cmd = ". %s; %s" % (ros_setup_script, command)
        if rosgraph.is_master_online():
            self.ros_status, self.experiment_status = 'Error', 'Error'
            self.logger.warning("There is already a ROS master online. Aborting launch.")
            self.close()
        else:
            self.p = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.logger.debug("Started launch as subprocess")

    def update_status(self):
        if self.ros_status == 'Error':
            self.logger.debug(f"Update status. ROS: {self.ros_status}; Trial {self.experiment_status}")            
        else:
            self.ros_status, self.experiment_status = self.monitor.update_status()
            self.logger.debug(f"Update status. ROS: {self.ros_status}; Trial {self.experiment_status}")

    def close_launch(self):
        self.logger.info("Sending SIGINT to launch subprocess")
        while self.p.poll() == None: 
            os.killpg(os.getpgid(self.p.pid), signal.SIGINT)
            time.sleep(1)
        time.sleep(0.5)

    def check_ros_proc(self):
        self.logger.debug("Checking if anything is still alive")
        for proc in psutil.process_iter():
            if proc.name() == "gzserver" or proc.name() == "gzclient":
                self.logger.debug(f"Still alive {proc.name()}")
                try:
                    subprocess.call("pkill {}".format(proc.name()))
                except:
                    self.logger.debug(f"Error when closing {proc.name()}")
                    pass

    def force_kill(self):
        self.logger.debug("Force kill any reminiscent process")
        subprocess.call("pkill -f rosmaster", shell=True)
        subprocess.call("pkill -f gz", shell=True)
        subprocess.call("pkill -f gazebo", shell=True)

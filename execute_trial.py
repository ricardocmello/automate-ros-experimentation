#!/usr/bin/python
import signal
from threading import Event
import subprocess
import time
import yaml
import argparse
import sys
import psutil 
import time
import rospy 

# Experiment-specific functions
from experiment_utils import ROSLaunch
from monitor import GzStatsMonitor

import logging

'''
This script manages one trial of a given experiment. It is meant to be called from the Experiment() class, but should work by itself.

We rely on an object of the ROSLaunch class to provide information regarding the trial.

Change the timeout_started and timeout_finished variables to define how long to way before throwing an error.
'''


# For arg parse https://stackoverflow.com/questions/15008758/parsing-boolean-values-with-argparse
def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def save_file_times(name, trial, final_status, start_time, finish_time, start_sim_time, finish_sim_time):
    file_name = "./files/" + name + "_trials_timming.csv"
    logger.debug("Saving trial info in file {}".format(file_name))
    append_line = ",".join([trial, final_status, start_time, finish_time, start_sim_time, finish_sim_time+"\n"])
    with open(file_name, "a+") as f:
        f.write(append_line)

## Parse arguments
parser = argparse.ArgumentParser(
        description='Executes one trial of an experiment involving ROS')
parser.add_argument('--config', '-c', type=str, required=True, help='Name of the config file (.yaml) in the config directory.')
parser.add_argument('--name', '-n', type=str, required=True, help='Name of the experiment')
parser.add_argument('--trial', '-t', type=int, default=0, help='Trial number')
parser.add_argument("--gzstats", type=str2bool, nargs='?', const=True, default=False, help="Save 'gz stats' output to file")
args = parser.parse_args()

# Set logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s:%(name)s:%(levelname)s:%(message)s')
file_handler = logging.FileHandler('./logs/{}-trial{}.log'.format(time.strftime("%y%m%d-%H%M%S"),str(args.trial)))
file_handler.setFormatter(formatter)
file_handler.setLevel(logging.DEBUG)
stream_handler = logging.StreamHandler()
stream_handler.setFormatter(formatter)
stream_handler.setLevel(logging.INFO)
logger.addHandler(file_handler)
logger.addHandler(stream_handler)

logger.info("Starting {}".format(str(__name__)))

config_file = "config/" + args.config
with open(config_file, "r") as ymlfile:
    logger.debug("Opening config file {}".format(config_file))
    cfg = yaml.load(ymlfile)

logger.info("Starting trial #{}".format(args.trial))

logger.info("Creating new launch object")
ros_side = ROSLaunch(cfg)

if args.gzstats:
    logger.info("Creating new gz monitor object")
    gz = GzStatsMonitor(args.name, args.trial)

logger.info("Starting to monitor the execution of the trial")
started_flag, finished_flag = False, False
start_time, finish_time = "0.0", "0.0"

timeout_finish = time.time() + 200 # in seconds
timeout_started = time.time() + 25 # in seconds
while ros_side.experiment_status != "Finished" and time.time() < timeout_finish and time.time() < timeout_started:
    logger.debug("Ask for status update")
    ros_side.update_status()    
    logger.debug("Status update: {}".format(ros_side.experiment_status))
    if ros_side.experiment_status == "Started" and not started_flag:
        timeout_started = 10**10
        start_time, start_sim_time = int(time.time()), int(rospy.get_time())
        started_flag = True
        logger.debug("Trial {}, started at time {}".format(args.trial,start_time))
    elif ros_side.experiment_status == "Finished" and not finished_flag:
        finish_time, finish_sim_time = int(time.time()), int(rospy.get_time())
        finished_flag = True
        logger.debug("Trial {}, finished at time {}".format(args.trial,finish_time))
    if ros_side.experiment_status == "Fail":
        break
    time.sleep(0.5)  
 
logger.debug("Trial {} is finished. ROS status: {}".format(args.trial,ros_side.experiment_status))

if ros_side.experiment_status == "Finished":
    success = True 
    final_status = "Success"
else:
    success = False
    final_status = "Fail"
    
save_file_times(str(args.name), str(args.trial), str(final_status), str(start_time), str(finish_time), str(start_sim_time), str(finish_sim_time))
logger.info("Trial {} is finished. Final status: {}".format(args.trial,final_status))

if args.gzstats:
    logger.debug("Closing gz monitor object")
    gz.close()

logger.debug("Closing launch object")
ros_side.close()
time.sleep(0.5)  
if success:
    logger.debug("sys.exit(0)")
    sys.exit(0)
else:
    logger.debug("sys.exit(1)")
    sys.exit(1)



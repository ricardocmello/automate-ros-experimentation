#!/usr/bin/python
import signal
from threading import Event
import subprocess
import time
import psutil 
import os
import logging

# Experiment-specific functions
from experiment_utils import ROSLaunch
from monitor import ResmonMonitor

"""
We are working with ROS Melodic only (python 2), but this script is written in python 3.
This means that our scripts won't find python 2 modules such as Melodic's rospkg and roscomm.
To overcome this, I'm using a python 3 virtualenv in which I've installed ROS' modules using pip3:
'sudo -H pip3 install rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools catkin_pkg'
"""

class Experiment:
    def __init__(self, name, trials=1): 
        self.create_logger()
        self.logger.info("Creating instance of Experiment()")
        
        self.name = name
        self.trials = trials
        self.logger.info(f"Experiment name: {name}")

        self.config_file = self.config()

        self.initialize_resmonmonitor()    
        
    def close(self):
        self.close_local()
            
    def config(self):
        config_file = ["config.yaml"]
        return config_file

    def initialize_resmonmonitor(self):
        self.logger.debug("Starting resmon monitor")
        self.resmonmonitor = ResmonMonitor(self.name)
    
    def main(self):
        self.logger.debug("Inside main function")
        
        desired_successful_trials = self.trials
        self.logger.debug(f"We want {desired_successful_trials} successful trials")

        successful_trials, current_trial = 0, 0
        while successful_trials < desired_successful_trials:
            current_trial += 1
            self.logger.info(f"Start new trial: Trial {current_trial}")
            success = self.execute_trial(self.config_file, self.name, current_trial)
            
            self.logger.info(f"Trial {current_trial} finished")
            self.logger.debug("Check trial for success")
            if success:
                successful_trials += 1
                self.logger.info(f"Successful trials: {successful_trials}")
            else:
                self.logger.info(f"Trial {current_trial} failed")
            
            self.logger.debug(f"Check if successful_trials {successful_trials} < {desired_successful_trials} desired_successful_trials")
            if successful_trials < desired_successful_trials:
                self.logger.debug(f"Still {desired_successful_trials - successful_trials} trials left")
                wait = 5
                self.logger.info(f"Waiting {wait}s before next trial")
                time.sleep(wait)
            else:
                break
            self.logger.debug("Check while conditions")
            self.logger.debug(f"successful_trials < desired_successful_trials - {successful_trials} < {desired_successful_trials}")

        self.logger.info("Experiment finished")
        self.close()

    def execute_trial(self, config_file, name, trial):
        self.logger.debug("Calling subprocess for embedded experiment")
        self.p = subprocess.Popen("python ./execute_trial.py --config {} --name {} --trial {} --gzstats".format(config_file[0], name, trial),shell=True)
        self.logger.debug("Entering while loop: waiting for subprocess to end")
        while self.p.poll() is None:
            time.sleep(0.5)
        return_code = self.p.returncode # A negative value -N indicates that the child was terminated by signal N
        self.logger.debug(f"Subprocess has ended with return code {return_code}")
        if return_code != 1:
            success = True
        else:
            success = False
        return success

    def close_local(self):
        self.logger.debug("Closing resmon monitor")
        self.resmonmonitor.close()
        time.sleep(1)
        while self.p.poll() == None: 
            os.killpg(os.getpgid(self.p.pid), signal.SIGTERM)
            time.sleep(1)
            if self.p.poll() != None:
                pass

    def create_logger(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s: %(name)s : %(levelname)s : %(message)s')
        
        if (self.logger.hasHandlers()):
            self.logger.handlers.clear()
        
        file_handler = logging.FileHandler('./logs/{}-experiment.log'.format(time.strftime("%y%m%d-%H%M%S")))
        file_handler.setFormatter(formatter)
        file_handler.setLevel(logging.DEBUG)

        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(formatter)
        stream_handler.setLevel(logging.DEBUG)

        self.logger.addHandler(file_handler)
        self.logger.addHandler(stream_handler)

if __name__ == '__main__':
    '''
    Provide a set of experiment scenarios and number of trials to be performed in each scenario
    Different scenarios can be provided in the form of a dictionary, in which the keys are the id of each scenario
    and the values can be anything you need to characterize and configure your scenario (e.g., paths to .launch files)
    If you are working with a single scenario, you can leave the value as an empty list
    '''
    scenarios = {"scenario1": []}
    trials = 1

    exp_dict={}
    for exp_name in scenarios.keys():
        """
        If you are working with more than one scenario, insert here your update routine.
        E.g., a script to change the config/config.yaml file to update the path to a .launch file 
        """


        '''
        Perform a set of trials in a given scenario using the Experiment() class.
        '''
        exp_dict[exp_name] = Experiment(exp_name,trials)
        try:
            exp_dict[exp_name].main()
            print("-"*50)
            exp_dict[exp_name].close()
        except KeyboardInterrupt:
            print("-"*5)
            exp_dict[exp_name].close()
        except:
            raise
        time.sleep(5)
        exp_dict.pop(exp_name, None)
#!/usr/bin/python
import signal
import subprocess
import os
import time
import psutil

import logging

class ResmonMonitor:
    def __init__(self, name):
        self.create_logger()
        self.logger.info("Starting resmon monitor")
        self.logger.debug(f"Experiment type {exp_type}")
        
        self.start_monitor_local(name)
        
    def close(self):
        self.logger.debug("In close()")
        self.logger.debug("Sending SIGINT to resmon subprocess [Local]")
        try:
            while self.resmon_local.poll() == None: 
                os.killpg(os.getpgid(self.resmon_local.pid), signal.SIGTERM)
                time.sleep(0.5)
                self.logger.debug("Waiting subprocess to close [Local]")
                if self.resmon_local.poll() != None:
                    self.logger.debug("Resmon subprocess closed [Local]")
        except:
            self.logger.exception("Exception when closing resmon [Local]")
            pass
        return True

    def start_monitor_local(self, name):
        resmon_fname_cpu = "_".join(["./files/"+str(name), "local", "resmon_cpu.csv"])
        resmon_fname_nic = "_".join(["./files/"+str(name), "local", "resmon_nic.csv"])
        self.logger.debug(f"Will save to files {resmon_fname_cpu} and {resmon_fname_nic}")
        
        command = ["python3",
                "~/python-resmon/resmon/resmon.py",
                "--delay 1 -f",
                f"-o {resmon_fname_cpu}",
                "-n wlxe8de271cc1f0",
                f"--nic-outfile {resmon_fname_nic}"]
        cmd = " ".join(command)
        self.logger.debug(f"Will execute the command {cmd}")
        self.logger.debug(f"Note that the iface name is hardcoded")

        self.logger.info(f"Starting resmon monitor subprocess [Local]")
        try:
            self.resmon_local = subprocess.Popen(cmd, shell=True, preexec_fn=os.setpgrp)
            self.logger.info(f"Started resmon monitor subprocess [Local]")
            self.resmon_local_status =  True
        except:
            self.logger.exception(f"Failed to start resmon monitor subprocess [Local]")
            self.resmon_local_status = False
    
    def create_logger(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s:%(name)s:%(levelname)s:%(message)s')

        file_handler = logging.FileHandler('./logs/{}-resmon.log'.format(time.strftime("%y%m%d-%H%M%S")))
        file_handler.setFormatter(formatter)
        file_handler.setLevel(logging.DEBUG)

        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(formatter)
        stream_handler.setLevel(logging.INFO)

        self.logger.addHandler(file_handler)
        self.logger.addHandler(stream_handler)
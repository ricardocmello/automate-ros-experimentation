#!/usr/bin/python
import subprocess

from paramiko import SSHClient
import paramiko

cmd = "pkill -f rosmaster & " + "pkill ping & " + "pkill -f resmon.py & " + "pkill -f execute_trial.py" 

a = subprocess.Popen(cmd,shell=True)





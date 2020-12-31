#!/usr/bin/python
import subprocess
a = subprocess.Popen("rm ./files/* && rm ./logs/*",shell=True)

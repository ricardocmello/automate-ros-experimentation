## Automate ROS experimentation

This set of scripts automate experiments using ROS Melodic and Gazebo. It will start a ROS launch, monitor the experiment status and close the launch when the experiment is finished. The `main.py` script allows you to automate the execution of multiple trials of an experiment.

The `execute_trial.py` file manages a single realization of an experimental trial. It relies on the `ROSLaunch()` class (defined on `experiment_utils/ros_launch.py`) to launch ROS nodes and to monitor when the experiment has started and ended. After the trial, `execute_trial.py` script saves the execution status and times to a file in the `files/` folder. Example of usage:

```
python ./execute_trial.py --config "config.yaml" --name "experiment-name" --trial 1 --gzstats
```

The script will look for the `config.yaml` file inside the `config/` folder and will log the given trial name and number. The `--gzstats` optional argument stores the real-time factor of the simulation as provided by Gazebo. Note that the `config.yaml` contains the package and launchfile names so that the `ROSLaunch()` object can launch it.

The `ROSLaunch()` relies on an experiment-specific library to monitor the experiment status. **You must provide your own library tailored to your needs** and add it to the `config.yaml` or equialent file. We provide the `EmbeddedMonitor()` class (defined on `monitor/ros_monitor_embedded`) as an example to guide you. This class must provide a `update_status()` method that returns the experiment status ("Not Started", "Started", "Error" or "Finished"). This method is called by a `ROSLaunch()` object. The monitor class is experiment-specific because it is up to you to define what each status means. In our example, we inserted a publisher in one of the nodes used in the experiment and our monitor subscribes to this topic. Thus, our `config.yaml` file contains the required ROS node (or nodes) and the topic to subscribe.

Other monitors are defined in the `gzstats_monitor.py` and `resmon_monitor.py` scripts. The `gzstats_monitor.py` should work out-of-the-box. The `resmon_monitor.py` depends on the [python-resmon](https://github.com/ricardocmello/python-resmon) and assumes that `~/python-resmon/resmon/resmon.py` is a valid path (i.e., you git clone'd the repository to your home folder).

If you want to perform several trials or work with multiple experiment scenarios, use the `main.py` script and modify its main section according to your needs. 

Since ROS Melodic uses Python 2 and our scripts are written in Python 3, our scripts won't find Python 2 modules such as Melodic's `rospkg` and `roscomm`. To overcome this, use a Python 3 virtualenv and install ROS' modules using pip3:

```
sudo -H pip3 install rosdep rospkg rosinstall_generator rosinstall wstool vcstools catkin_tools catkin_pkg
```

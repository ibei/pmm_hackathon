### TIAGo PMM Workspace
*ROS Workspace for the IROS 2018 Hackathon*

Assuming you already have a working TIAGo installation:
*After cloning this repo, do the following*
* Run

`rosinstall src /opt/ros/indigo tiago_public.rosinstall`

```python
# Use rosdep to install all dependencies. Just in case.
```

`rosdep install --from-paths src --ignore-src --rosdistro kinetic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit sensor_to_cloud hokuyo_node libdw-dev"`

`source /opt/ros/indigo/setup.bash`

```python
# Initialize workspace and build packages
```

`catkin init`

`catkin build -DCATKIN_ENABLE_TESTING=0`

`source ./devel/setup.bash`

#### Test launch
`roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel world:=empty`

#### For gazebo headless launch
`roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel world:=empty gui:=false`

For the instructions to install TIAGo with ROS from scratch, see the following pages:
* [Official Installation Page](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/TiagoSimulation). *May run into some problem.*

See this [file](https://gitlab.com/toluwajosh/pmm_hackathon_ws/blob/master/run_instructions.txt) for run instructions

Please share any other issues encountered.

#### To contribute;
Please create your own branch and work on it, only send merge requests to the devel branch. 
All accepted updates will be merged to the master branch.

#### command to find the play_motions
```
rosparam list | grep  "play_motion/motions" | grep "meta/name" | cut -d '/' -f 4
```
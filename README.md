# motion-planning
##Overview
This is a collection of algorithms and other infrastructure I have developed to learn about motion planning in robotics. The code base is primarily Python. I have developed made the motion planners and compnents modular, in order to make the code extensible to arbitrary robot kinematics and environment dimensionality, although at the moment only 2D environments are implemented.

A sample of the RRT\* algorithm planning on an empty environment can be seen below.

![alt text](https://github.com/lanton97/motion-planning/blob/main/assets/rrt\*_run.gif)

## Installing dependencies

To install the correct environment using anaconda, run 
```req
conda env create -f environment.yml
```

## Run a Planner

We use a single script with various options to run a planner in the defined environments and with the given vehicle dynamics. To run the script with the default arguments, run

```sample
run.py
```

This runs the RRT algorithm with the particle dynamics and an empty planning environment. Further options for the script are detailed below.

```run
usage: run.py [-h] [--alg alg_name] [--vehicle veh_name] [--map map_name] [--map-file map_file] [--render rend] [--break-on-path break] [--save-run save]

This script handles running motion planning algorithms

optional arguments:
  -h, --help            show this help message and exit
  --alg alg_name        Name of the algorithm we want to try. Valid algorithms: dict_keys(['rrt', 'rrt*', 'bd-rrt', 'bd-rrt*'])
  --vehicle veh_name    Name of the vehicle dynamics we want. Valid vehicle dynamics: dict_keys(['particle-2d', 'dubins'])
  --map map_name        Name of the map we want to evaluate on. Valid maps: dict_keys(['empty-2D', 'maze-2D'])
  --map-file map_file   File name for loading the configuration. The file should reside in env/map_data/ .
  --render rend         Whether to render the environment or not. Leave empty for no, insert anything for yes.
  --break-on-path break
                        Whether to finish planning if a path is founc. Leave empty for no, insert anything for yes.
  --save-run save       Whether to save the run as an animation. Any string is considered as true.
```

## Maze Environment
We define a basic maze environment that utilizes user defined walls and starting/ending positions. We can alter the setup by editing an XML file. The file should be included in the "env/map_data/" directory, and can be specified using the run.py script. A sample file is provided, and can be seen below.
 
```maze
<?xml version="1.0" ?>
<root>
	<start_position pos="200 -200"/>
	<goal_position pos="-200 200">-200 200</goal_position>
	<walls>
		<wall1 start="-250 170" end="-50 170"/>
		<wall2 start="0 170" end="-250 170"/>
		<wall3 start="-300 120" end="-100 120"/>
		<wall4 start="-50 120" end="-300 120"/>
		<wall5 start="-150 50" end="30 50"/>
		<wall6 start="-100 50" end="200 50"/>
		<wall7 start="-300 0" end="-200 0"/>
		<wall8 start="-50 0" end="150 0"/>
		<wall9 start="200 0" end="250 0"/>
		<wall10 start="-300 80" end="-200 80"/>
		<wall11 start="0 80" end="180 80"/>
		<wall12 start="-200 150" end="200 150"/>
		<wall13 start="0 180" end="300 180"/>
		<wall14 start="250 -170" end="50 -170"/>
		<wall15 start="0 -170" end="250 -170"/>
		<wall16 start="300 -120" end="100 -120"/>
		<wall17 start="50 -120" end="300 -120"/>
		<wall18 start="150 -50" end="-30 -50"/>
		<wall19 start="100 -50" end="-200 -50"/>
		<wall20 start="300 0" end="200 0"/>
		<wall21 start="50 0" end="-150 0"/>
		<wall22 start="-200 0" end="-250 0"/>
		<wall23 start="300 -80" end="200 -80"/>
		<wall24 start="0 -80" end="-180 -80"/>
		<wall25 start="200 -150" end="-200 -150"/>
		<wall26 start="0 -180" end="-300 -180"/>
	</walls>
</root>
```
As we can see, the various positions of the walls are simple to edit. An example of the Bidirectional RRT algorithm running on this maze can be seen below.

![alt text](https://github.com/lanton97/motion-planning/blob/main/assets/bd-run.gif)

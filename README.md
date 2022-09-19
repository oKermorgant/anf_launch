Launch file examples for ANF ROS 2

## Basic syntax

The `launch` folder includes a few files to run `turtlesim`.

### Standard syntax

`turtlesim_classic_launch.py` uses the classical syntax to run `turtlesim` with an option to run either a manual control or an open-loop one

### `simple_launch`

`turtlesim_launch.py` does exactly the same as `turtlesim_classic_launch.py`, but with the `simple_launch` package

Two additional files can spawn new robots in the simulation:
    - `single_launch.py` spawn a single robot at some (x,y,theta) position, that may or may not track a target
    - `spawn_launch.py` includes `single_launch.py` several times to spawn multiple robots in the simulation


## Robot description and RViz

The `bb8` folder includes a launch file that:
    - loads a robot description from a `xacro` file
    - runs a small simulation (with an open-loop control) on this robot
    - runs RViz with a custom configuration file
    

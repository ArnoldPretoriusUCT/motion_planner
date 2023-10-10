# Motion planning software for a car-like robot

## Notes

This repository is a fork of [this template](https://github.com/duckietown/template-ros).

The source code for the nodes is located in `packages/motion_planner/src`.

A Duckiebot (DB21-M) with the name `duckiebot` was used for testing.

The software was built using ROS Noetic Ninjemys.

First read [The Duckiebot Operation Manual](https://docs.duckietown.com/daffy/opmanual-duckiebot/intro.html) and [The Duckietown Developer Manual](https://docs.duckietown.com/daffy/devmanual-software/intro.html).

## Setup

1. Set up the Duckiebot and its environment.
2. Open two terminals.
3. In the first terminal, run `dts devel build -f` to build the software locally or `dts devel build -f -H duckiebot` to build the software on the Duckiebot.
4. In MATLAB, open `parameters.m`, `initialise.m`, `animate.m` and `stop.m`.
5. In `parameters.m`, set the parameter values.

## Run

1. In the first terminal, run `dts devel run` if the software was built locally or `dts devel run -H duckiebot` if the software was built on the Duckiebot.
2. Wait for the nodes to begin running.
3. In the second terminal, run `export ROS_MASTER_URI=http://duckiebot.local:11311` and then `rosbag record /duckiebot/NODE_NAME/TOPIC_NAME ...` to record to a bag file.
4. Wait for the recording to begin.
5. In MATLAB, run `initialise.m`.

## Stop

1. Wait for the `control complete` message to appear in the first terminal, or run `stop.m` in MATLAB to stop the Duckiebot prematurely.
2. Press `Ctrl`+`C` in both terminals.

## Animate

1. In `animate.m`, set `bag_file_name` to the name of the bag file and set `create_gif` to `0`.
2. In MATLAB, run `animate.m`.

## Create a GIF of the animation

1. In `animate.m`, set `gif_name` to the desired name of the GIF, set `gif_resolution` to the desired resolution of the GIF and set `create_gif` to `1`.
2. In MATLAB, run `animate.m`.
3. Wait for MATLAB to finish creating the GIF.

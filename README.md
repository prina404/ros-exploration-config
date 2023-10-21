# ROS exploration configs
This repo contains the code and configs used to perform a distance-greedy 
exploration of indoor environments.

Main nodes used: `move_base`, `gmapping`, `stage_ros`, `explore_lite`.
Their configurations can be found either in the `params` folder or 
in the `exploreambient_gmapping.launch` file.

The `explore` package under `src/` is based on my fork of the `explore_lite` package, 
the implemented changes are [documented here](https://github.com/prina404/m-explore).

## Details

The config provided (roughly) simulates a turtlebot 3 with a 360° laser sensor, 
the exploration policy is as distance-greedy as possible: 
the gain component (size) of the frontier is completely ignored, and the closest frontier is chosen instead.

*Note:* as explained in the explore_lite fork documentation, a frontier may be flagged as closest even if 
it's not *actually* the closest one (it should be amongst the closer ones anyway).

---

The `singlerun.py` script is responsible for:
- Launching the `exploreambient_gmapping.launch` file.
- Saving every 60 seconds (wall time) a snapshot of the explored map
- Stopping the exploration when there are no frontiers available/reachable.

The exploration actually stops if a new goal isn't being published for more than 500 (ROS) seconds, 
the `explore` node is then respawned to try reaching any frontiers previously marked as unreachable. 
In case no old frontiers are present the process is killed after 100 (ROS) seconds.

The speed-up of the simulation is controlled by the `speedup` parameter in the `worldcfg.inc` config file,
any half-decent laptop should be capable of handling at least 2x ~ 4x, remember to monitor per-core CPU usage
and in case of core saturation (*looking at you, gmapping*) reduce the multiplier.
On a 9th gen intel desktop processor I managed to get decent results with speeds up to 15x.

The map images as well as the bagfiles are saved under `runs/outputs/` in the corresponding directories.
If exploration has ended correctly the latest map image saved should be called `Map.png`.


## Usage Examples

### Bare Metal or VM 

Copy the `src` directory into your local `catkin_ws` directory then build with
```shell
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```
*Note:* if the build fails make sure your ROS environment is set up correctly and all the dependencies are installed. The full requirements list can be seen in the Dockerfile (all the `ros-*` entries).

To verify that everything is working correctly simply navigate to the python folder and launch the script:
```shell
$ cd src/my_navigation_configs/python/
$ python3 singlerun.py ../worlds/E34-2.world
```
There should be a flooding of `TF_REPEATED_DATA` warning messages which can be ignored, 
most importantly make sure there are no error messages highlighted in red.
The fix to avoid the flooding is discussed below.

To enable Rviz simply uncomment line 21 in `exploreambient_gmapping.launch` 
(remember to comment it back when building the docker image).

### Docker

Build the image:
```shell
$ docker build --tag 'rosnoetic:explore' .
```
*Note:* in case of a `Got permission denied [...]` error from docker follow Method 3 of [this guide](https://phoenixnap.com/kb/docker-permission-denied) and reboot your pc.

---
To launch a batch run of every map inside the `worlds` folder use the `spawnContainers.py` script:
```shell
$ python3 spawnContainers.py
```
A prompt will ask the number of simultaneous containers to spawn, when all pending jobs are 
terminated results are saved in the `output` folder.

*Note:* I haven't set up the correct permissions of files and folders created by the containers, 
if write privileges are needed a quick fix is `$ sudo chown -R $USER output/**`

## Known Issues
PRs are *very* welcome :)

- When launching the containers in batch through `spawnContainers.py`, keyboard interrupt 
  signals are intercepted by the `ThreadPoolExecutor` context manager, 
  which by default waits for the running jobs to end before releasing the control.
  In case a quick termination is needed, a good amount of `Ctrl+C` spam should do the trick.
  Use `kill` or a process manager otherwise, and make sure with `docker ps` that no containers
  are running afterwards
- Sometimes after a successful run there is no `Map.png` file in the folder, 
  this happens when two threads in the same container try calling simultaneously
  the `map_saver` node, luckily it is a pretty rare event.
- Terminal input/output is broken after launching the containers, this is due to the
  (*brutal*) output redirection used to avoid the `TF_REPEATED_DATA` message spam. 
  Simply close the terminal. 
- When terminating early a run on a baremetal/VM install if the map_saver node
  is active when the other nodes get shut down, it will hang, preventing roscore from terminating. 
  A couple of quick fixes are `$ rosnode kill map_saver` or `$ killall map_saver`.

---
## Tips and Tricks
The following collection contains all the caveats, bugs and 
undocumented stuff that gave me plenty of headaches while working with ROS.   

### TF_REPEATED_DATA spam
As discussed [here](https://github.com/ros/geometry2/issues/467#issuecomment-1238639474) a not-so-elegant
fix consists in launching the script with a stdout+stderr redirection, filtering out
every message not matching with the provided pattern. As an example the resulting command should
look like this:
```shell
$ python3 singlerun.py ../worlds/E34-2.world 2> >(grep -v TF_REPEATED_DATA buffer_core)
```
To be fair the syntax of the command is not entirely correct, the pattern specified for the inverse
matching is separated by a space, thus `buffer_core` is interpreted as a subsequent parameter 
and not as part of the pattern. It filters more output than it should but works 
okay in practice, further investigation on the grep manual would probably yield a better solution.

Be careful using redirection while trying new configs, because most warnings/errors will be suppressed, 
and terminal IO formatting will probably be broken afterwards.
Keep an eye also on CPU usage, in some early tests filtering out the warning spam reduced the CPU load
by a 10~15%

---
### Stage
Cropping of the empty space surrounding the building is performed automatically by stage, 
this means that when creating the `.world` files the `size [x y z]` parameters should be relative
to the building's bounding box, not the whole image size.

In case using the image size is preferred, adding a single-pixel black edge is sufficient to prevent
stage from cropping. In the `util` folder the `makePicBorder.py` script does just that.

---
By documentation headless mode can be activated with the `-g` option 
__but__, if enabled, stage completely ignores the `speedup` parameter and publishes the `/clock` 
topic updates as quickly as possible, thus making it impossible for other nodes to keep up
with. 

For this reason in the docker image an X virtual framebuffer (`Xvfb`) 
is required to run stage in headful mode, without actually having a display output.

---
Sometimes stage's simulated laser scans will pass through solid walls, this will result
in the detection of unreachable frontiers, wasting time until all frontiers are invalidated.

---
### Gmapping CPU usage

When looking at gmapping's default parameters the `xmax, xmin, ymax, ymin` are set to $\pm 100$, 
resulting in an output map resolution of 4000x4000 px. This causes a lot of unnecessary load,
missed updates, stutters and worse SLAM performance.

The config provided in this repo saves 2000x2000 px maps, 
which should be plentiful for most environments.
If the laser scan reaches the map borders, gmapping will add a constant amount of padding, 
this way early map images may have a smaller resolution than the later ones.

Also note that the current implementation of gmapping is single threaded, meaning that when deploying
a container on a cluster the bottleneck will be the single core performance of the CPU(s).

---
### Global Planner

By [documentation,](http://wiki.ros.org/global_planner) it is possible to specify the
`defafult_tolerance` parameter, but as discussed [here](https://answers.ros.org/question/239236/setting-default-tolerance-on-global-planner-with-move-base/), 
it doesn't do anything. Sometimes (mostly due to SLAM inaccuracy or stage's bugs) 
some goals are published right on lethal obstacles, resulting in a failure to compute a path towards 
such goals.

A fix was [proposed but never merged](https://github.com/ros-planning/navigation/pull/1041), 
there are also some forks that re-implemented the expected tolerance behavior.

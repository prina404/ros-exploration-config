# ROS exploration configs
This repo contains the code and configs used to perform a distance-greedy 
exploration of environments.

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
- Launching the `exploreambient_gmapping.launch` file
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

To verify that everything is working correctly simply navigate in the python folder and launch the script:
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

*Note:* I haven't managed the permissions of files and folders created by the containers, 
if write privileges are needed a quick fix is `$ chown -R $USER output/**`

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


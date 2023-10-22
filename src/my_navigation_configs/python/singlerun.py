import os
import sys
import signal
import time
from os import killpg, getpgid, setsid, makedirs, listdir, remove
from os.path import basename, join, exists
from subprocess import Popen, PIPE
from threading import Thread
from PIL import Image
import rospy
from geometry_msgs.msg import PoseStamped
import traceback

lastUpdate = 0


def killProcess(p):
    """
    Kills the process of the launchNavigation
    """
    killpg(getpgid(p.pid), signal.SIGTERM)


def getMap(p, folder):
    """
    Saves the map every 60 seconds
    """
    minutes = 0
    maxmapsave = 60
    seconds_mapsave = 60

    if not exists(join(folder, "Maps/")):
        makedirs(join(folder, "Maps/"))

    while True:
        if p.poll() is not None:
            return

        getmapString = (
            "rosrun map_server map_saver -f " + folder + "Maps/" + str(minutes) + "Map"
        )
        process = Popen(getmapString, shell=True, preexec_fn=setsid)
        process.wait()
        imStr = folder + "Maps/" + str(minutes)
        try:
            print(f"processing {imStr}Map.*")
            Image.open(imStr + "Map.pgm").save(imStr + "Map.png")
            remove(imStr + "Map.pgm")
        except IOError:
            print("cannot convert")

        if int(minutes) > maxmapsave:
            print("KILLING PROCESS DUE TO TIMEOUT")
            killProcess(p)
            return

        minutes += seconds_mapsave / 60
        time.sleep(seconds_mapsave)


def callback(data: PoseStamped):
    global lastUpdate
    lastUpdate = rospy.get_rostime().secs
    print(f"Received goal @ {lastUpdate} seconds (ROStime)")


def goal_listener():
    print("Started goal_listener")
    rospy.init_node("listener", anonymous=False, disable_signals=True)
    rospy.Subscriber("move_base/current_goal", PoseStamped, callback)


def saveMap(folder):
    if not exists(join(folder, "Maps/")):
        makedirs(join(folder, "Maps/"))
    getmapString = "rosrun map_server map_saver -f " + folder + "Maps/Map"
    process = Popen(getmapString, shell=True, preexec_fn=setsid)
    process.wait()
    imStr = folder + "Maps/"
    try:
        print(f"processing {imStr}Map.*")
        Image.open(imStr + "Map.pgm").save(imStr + "Map.png")
        remove(imStr + "Map.pgm")
    except IOError:
        print("cannot convert")


def checkActiveGoal(process, folder):
    global lastUpdate
    firstRun = True
    print("Started goal Thread, ")
    while True:
        time.sleep(3)
        print(f"-- Delta since last update: {rospy.get_rostime().secs - lastUpdate} ")
        if process.poll() is not None or rospy.get_rostime().secs - lastUpdate > 500:
            if firstRun:
                print("Restarting explore node to finish exploration")
                p = Popen("rosnode kill explore", shell=True)
                p.wait()
                firstRun = False
                lastUpdate = rospy.get_rostime().secs - 100
                continue
            saveMap(folder)
            print("SHUTTING DOWN DUE TO GOAL TIMEOUT")
            killProcess(process)
            return


def launchNavigation(world, folder):
    """
    Calls the launch file and starts the exploration, waits until the process is done
    """
    try:
        worldfile = basename(world)
        launchString = (
            "roslaunch my_navigation_configs exploreambient_gmapping.launch worldfile:="
            + world
            + " bag:="
            + folder
            + worldfile[:-6]
            + ".bag "
        )

        p = Popen(launchString, shell=True, preexec_fn=setsid)

        time.sleep(5)

        goal_listener()
        Thread(None, checkActiveGoal, "goal_worker", [p, folder], daemon=True).start()
        Thread(None, getMap, "map_worker", [p, folder], daemon=True).start()
        p.wait()
        return

    except KeyboardInterrupt:
        print("KILL PROCESS")
        killProcess(p)
        return
    except:
        traceback.print_exc()
        killProcess(p)
        return


def exploreWorlds(project_path, world: str):
    """
    Given a folder with world file it runs 5 times each environment exploration
    """
    out_dir = project_path + "/runs/outputs/"
    f = world.split("/")[-1]
    folder = join(out_dir, f[:-6])
    print(f"PATH: {folder}")

    maxrun = 0

    if world[-6:] == ".world":
        if not exists(folder):
            print(f"Making dir: {folder}")
            makedirs(folder)
        for i in listdir(folder):
            if int(i[3:]) >= maxrun:
                maxrun = int(i[3:])
        if not exists(join(folder, "run" + str(maxrun + 1) + "/")):
            makedirs(join(folder, "run" + str(maxrun + 1) + "/"))
        print("START")
        launchNavigation(world, join(folder, "run" + str(maxrun + 1) + "/"))
        print("END")
        time.sleep(1)


if __name__ == "__main__":
    # retrieve current path
    project = os.path.expanduser("~/catkin_ws/src/my_navigation_configs")
    exploreWorlds(project, os.path.abspath(sys.argv[1]))

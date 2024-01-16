import subprocess
import os
from concurrent.futures import ThreadPoolExecutor
import progressbar


def spawn_container(mapName: str, i, bar):
    bar.update(i)

    launchstr = f"""docker run -it \\
        -v ./worlds:/root/catkin_ws/src/my_navigation_configs/worlds \\
        -v ./output:/root/catkin_ws/src/my_navigation_configs/runs/outputs \\
        'rosnoetic:explore' worlds/{mapName}"""
    p = subprocess.Popen(launchstr, shell=True, stdout=subprocess.DEVNULL)
    p.wait()


def main(workers: int):
    pool = ThreadPoolExecutor(max_workers=workers)
    try:
        with progressbar.ProgressBar(max_value=len(os.listdir("worlds/")) - 2) as bar:
            futures = []
            for i, name in enumerate(os.listdir("worlds/")):
                if name.endswith(".world"):
                    futures.append(pool.submit(spawn_container, name, i, bar))
            pool.shutdown(wait=True)
    except:
        pool.shutdown(wait=False)
        subprocess.Popen("docker kill $(docker ps -q)", shell=True)
        return


if __name__ == "__main__":
    n = input("Insert number of workers to spawn:  ")
    main(int(n))
    print("All workers finished")

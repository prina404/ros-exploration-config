import sys
import subprocess
import os
from concurrent.futures import ThreadPoolExecutor


def spawn_container(mapName: str):
    print(f"Started run: {mapName}")
    launchstr = f"""docker run -it \\
        -v ./worlds:/root/catkin_ws/src/my_navigation_configs/worlds \\
        -v ./output:/root/catkin_ws/src/my_navigation_configs/runs/outputs \\
        'rosnoetic:explore' worlds/{mapName}"""
    p = subprocess.Popen(launchstr, shell=True, stdout=subprocess.DEVNULL)
    p.wait()
    print(f"----> Finished exploration of {mapName}")


def main(workers: int):
        pool = ThreadPoolExecutor(max_workers=workers) 
        try:
            futures = []
            for name in os.listdir("worlds/"):
                if name.endswith(".world"):
                    futures.append(pool.submit(spawn_container, name))
            pool.shutdown(wait=True)
        except:
            pool.shutdown(wait=False)
            subprocess.Popen("docker kill $(docker ps -q)", shell=True)
            return


if __name__ == "__main__":
    n = input("Insert number of workers to spawn:  ")
    main(int(n))
    print("All workers finished")


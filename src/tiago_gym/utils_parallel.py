import os
import subprocess
import rospkg
import time

def launch_simulations(n:int,speed:float = 0.0025):
    sim_pkg_path = rospkg.RosPack().get_path("elsa_tiago_gym")


    #launch the simulation environments
    sim_path = os.path.join(sim_pkg_path,'src/simulations/run_simulations_parallel.sh')
    subprocess.run(['gnome-terminal', '--', sim_path, str(n), str(speed)])


def kill_simulations():
    sim_pkg_path = rospkg.RosPack().get_path("gazebo_parallel")
    kill_path = os.path.join(sim_pkg_path,'src/simulations/kill_simulations.sh')
    subprocess.run(['gnome-terminal', '--', kill_path])
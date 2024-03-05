import os
import subprocess
import rospkg
import time
import rospy

sim_pkg_path = rospkg.RosPack().get_path("elsa_tiago_gym")

def launch_simulations(n:int,speed:float = 0.0025,gui:bool=True):
    gui = 'true' if gui else 'false'
    print(f'Launching simulation for {n} workers with speed of {speed}, with GUI = {gui}')
    #launch the simulation environments
    sim_path = os.path.join(sim_pkg_path,'src/simulations/run_simulations_parallel.sh')
    subprocess.run(['gnome-terminal', '--',sim_path, 
                    str(n),
                    str(speed),
                    gui])
    #set simulations velocities
    vel_path = os.path.join(sim_pkg_path,'src/simulations/set_velocity.sh')
    subprocess.run(['gnome-terminal', '--',vel_path, 
                    str(n),
                    str(speed)])

def set_velocity(n:int,speed:float = 0.0025):
    vel_path = os.path.join(sim_pkg_path,'src/simulations/set_velocity.sh')
    subprocess.run(['gnome-terminal', '--',vel_path, 
                    str(n),
                    str(speed)])

def set_sim_velocity(uri:str,speed:float = 0.0025):
    vel_path = os.path.join(sim_pkg_path,'src/simulations/set_sim_velocity.sh')
    subprocess.run(['gnome-terminal', '--',vel_path, 
                    uri,
                    str(speed)])
    rospy.loginfo(f"Physics imposed to {speed} at URI {uri}")


def kill_simulations():
    kill_path = os.path.join(sim_pkg_path,'src/simulations/kill_simulations.sh')
    subprocess.run(['gnome-terminal', '--', kill_path])

def set_v(gazebo_master_uri,speed):
    gazebo_command = f"execute GAZEBO_MASTER_URI={gazebo_master_uri}; gz physics -u 0 -s {speed}"
    result = subprocess.run(gazebo_command, shell=True, capture_output=True, text=True)
    return result.stdout


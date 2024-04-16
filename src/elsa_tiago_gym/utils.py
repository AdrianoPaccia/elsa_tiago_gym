import os
import rospkg
import rospy
import random
from gym import spaces
import numpy as np
from gym.envs.registration import register
import sys
import math
from gazebo_msgs.srv import GetModelState,SetModelState
from gazebo_msgs.msg import ModelState 
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from elsa_tiago_gym.utils_parallel import set_sim_velocity
from PIL import ImageColor


max_episode_steps = 100



def generate_random_config_2d(low:list, high:list, n_cubes = 5,  dist = 0.6, threshold = 0.02):
    x_start, y_start = low
    x_stop, y_stop = high
    n_row = int(abs(x_start - x_stop)/dist)
    n_col = int(abs(y_start - y_stop)/dist)
    if n_row*n_col < n_cubes:
        raise Exception('Distance not feasible')



    x_coords = np.linspace(x_start, x_stop, num=n_row)
    y_coords = np.linspace(y_start, y_stop, num=n_col)

    grid_x, grid_y = np.meshgrid(x_coords, y_coords)

    grid_points = np.column_stack((grid_x.ravel(), grid_y.ravel()))

    random_indices = np.random.choice(len(grid_points), size=n_cubes, replace=False)

    random_points = grid_points[random_indices]
    #random_points += np.random.uniform(low=[-threshold, threshold], high=[0.2, 0.3], size=random_points.shape)

    for rp in random_points:
        n = np.random.uniform(low = [-threshold,threshold], high = [threshold,threshold])
        rp+=n

    return random_points.tolist()



class ObservationSpace:
    def __init__(self, num_items, num_boxes,
                 grip_low=-1.0,grip_high=-1.0,
                 item_low=-1.0,item_high=-1.0,
                 box_low=-1.0,box_high=1.0
                 ):
        
        self.grip_space = spaces.Box(low=grip_low, grip_high=grip_high, shape=(3,), dtype=float)

        self.items_space = spaces.Dict({
            f'box_{i}': spaces.Dict({
            'position': spaces.Box(low=item_low, high=item_high, shape=(3,), dtype=float),
            'type': spaces.Discrete(n=10), 
            'held': spaces.Discrete(n=2) #bool
            }) for i in range(num_items)
        })

        self.boxes_space = spaces.Dict({
            f'box_{i}': spaces.Dict({
                'position': spaces.Box(low=box_low, high=box_high, shape=(3,), dtype=float),
                'type': spaces.Discrete(n=5)
            }) for i in range(num_boxes)
        })

        self.observation_space = spaces.Dict({
            'grip_poses': self.grip_space,
            'items': self.items_space,
            'boxes':self.boxes_space
            })
    
    def sample(self):
        sample_dict = {
            'image': self.image_space.sample(),
            'grip_poses': self.position_space.sample()
        }
        return sample_dict
    
    def shape(self):
        return self.observation_space.shape
        #return (self.grip_space.shape, self.items_space,self.boxes_space)


class ObservationSpaceMultimodal:
    def __init__(self, image_shape=(64, 64, 3), 
                 grip_low=-1.0,grip_high=-1.0,
                 ):
        self.image_space = spaces.Box(low=0, high=255, shape=image_shape, dtype=np.uint8)
        self.grip_space = spaces.Box(low=grip_low, grip_high=grip_high, shape=(3,), dtype=float)
        self.observation_space = spaces.Dict({
            'image': self.image_space,
            'positions': self.position_space
        })

    def sample(self):
        sample_dict = {
            'image': self.image_space.sample(),
            'positions': self.position_space.sample()
        }
        return sample_dict

    def contains(self, obs):
        return self.observation_space.contains(obs)

    def shape(self):
        return self.observation_space.shape


class ActionSpace:
    def __init__(self,dpos_low:np.array=[-1.0, -1.0, -1.0],dpos_high:np.array=[-1.0, -1.0, -1.0]):
        self.dpos_space = spaces.Box(low=dpos_low, high=dpos_high, shape=(3,), dtype=np.float32) # Space for increments in position
        self.hold_space = spaces.Discrete(2)  # Space for the booleans for picking/droping act 
        self.action_space = spaces.Tuple((self.dpos_space, self.pick_space))

    def sample(self):
        d_pos = self.pos_space.sample()
        hold = self.hold_space.sample()
        return (d_pos, hold)

    def contains(self, action):
        return self.action_space.contains(action)

    def get_shape(self):
        return self.action_space.shape
    

def objects_from_scene(model_names):
        '''Returns the list of all objects in the scene 
        and the dict for all cubes and for all cylinders
        '''
        get_gazebo_object_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        cubes = {}
        cylinders = {}
        for id in model_names:
            state_i = get_gazebo_object_state(id,'')
            pos_i = [state_i.pose.position.x, 
                     state_i.pose.position.y,
                     state_i.pose.position.z] 
            name_tkn = id.split('_')
            type_i = name_tkn[0]
            if type_i == 'cube':
                color_i = name_tkn[1]
                cube_i = Cube(id=id,position=pos_i,type_code=color_to_code(color_i),gazebo_state=state_i)
                cubes[id] = cube_i
            elif  type_i == 'cylinder':
                    color_i = name_tkn[1]
                    cylinder_i = Cylinder(id=id,position=pos_i,type_code=color_to_code(color_i),gazebo_state=state_i)
                    cylinders[id] = cylinder_i

            model = Model(cubes,cylinders)

        return model
    

        

class Observation:
    def __init__(self,g_pose:list, items:str, boxes:bool):
        self.gripper_pose = g_pose
        self.items = items
        self.boxes = boxes

class ObservationMultimodal:
    def __init__(self,g_pose:list, image:np.array):
        self.gripper_pose = g_pose
        self.image = image

def color_to_code(color):
    code = [val / 255 for val in ImageColor.getcolor(color, 'RGB')]
    return code


class Object:
    def __init__(self,id:str,position:list, type_code:str, gazebo_state):
        self.id=id
        self.position = position
        self.init_position = position
        self.type_code = type_code
        self.gazebo_state = gazebo_state
    def set_state(self,state):
        #store the state with the frma in the base_footprint
        self.gazebo_state=state
        self.gazebo_state.header.frame_id = "base_footprint"
        #store the position 
        position = state.pose.position
        self.position =[position.x,position.y,position.z]
        orientation = state.pose.orientation
        quaternion = (orientation.x,orientation.y,orientation.z,orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.orientation = euler


tollerance =0.001

class Cylinder(Object):
    def __init__(self,id:str,position:list, type_code:str, gazebo_state, radius:float=0.10,heigth:float=0.01):
        super().__init__(id,position, type_code,gazebo_state)
        self.radius = radius
        self.heigth = heigth

    def contains(self, cube:Object):
        dist_xy = math.sqrt((self.position[0]-cube.position[0])**2+(self.position[1]-cube.position[1])**2) - tollerance
        xy_check =  dist_xy < self.radius
        dist_z = cube.position[2] - self.position[2]
        z_check = dist_z <= self.heigth/2 + cube.side/2 + 0.1
        if xy_check and z_check:
            return True
        else:
            return False 

class Cube(Object):
    def __init__(self,id:str, position:list, type_code:str, gazebo_state, held:bool=False, side:float=0.06):
        super().__init__(id,position, type_code,gazebo_state)
        self.held = held
        self.side = side

    def is_inside(self,box:Object):
        dist_xy = math.sqrt((self.position[0]-box.position[0])**2+(self.position[1]-box.position[1])**2) - tollerance
        xy_check =  dist_xy < box.radius
        dist_z = self.position[2] - box.position[2]
        z_check = dist_z <= 0.5

        if xy_check and z_check and not self.held:
            return True
        else:
            return False


class Model:
    """Contains all cubes and cylinders dicts
    """
    def __init__(self,cubes:dict, cylinders:dict):
        self.cubes=cubes
        self.cylinders=cylinders
        self.n_cubes = len(cubes)
        self.n_cylinders = len(cylinders)
        # define a dict indicating if a cube (key = cube_id) is in the goal (value = bool)
        self.cubes_subgoals = {}

    def set_cube_state(self,id,state):
        self.cubes[id].set_state(state)

    def set_cylinder_state(self,id,state):
        self.cylinders[id].set_state(state)

    def show(self):
        # prints all cubes and cylinders
        print('Cubes:')
        for id in self.cubes:
            print('{:} :'.format(id))
            print('  - pos: {:}'.format(self.cubes[id].position))
            print('  - type: {:}'.format(self.cubes[id].type_code))
            print('  - held: {:}'.format(self.cubes[id].held))
        print('\nCylinders:')
        for id in self.cylinders:
            print('{:} :'.format(id))
            print('  - pos: {:}'.format(self.cylinders[id].position))
            print('  - type: {:}'.format(self.cylinders[id].type_code))
    
    def get_obs(self):
        # outputs the observation stack for cubes and cylinders
        cubes = {}
        for cube_k, cube_v in self.cubes.items():
            cubes[cube_k] = {
                'position':cube_v.position,
                'color':cube_v.type_code,
                'held':[cube_v.held]
            }
        cylinders = {}
        for cy_k, cy_v in self.cylinders.items():
            cylinders[cy_k] = {
                'position':cy_v.position,
                'color':cy_v.type_code
            }

        return cubes, cylinders
        '''print(self.cuber)
        cubes=[]
        cylinders=[]
        for id in self.cubes:
            cubes.append([
                *self.cubes[id].position,
                int(str(self.cubes[id].type_code)),
                self.cubes[id].held
            ])
        for id in self.cylinders:
            cylinders.append([
                *self.cylinders[id].position,
                int(str(self.cylinders[id].type_code)),
            ])
        return np.array(cubes), np.array(cylinders)'''
    
    def get_objects(self):
        '''Outputs the list with the id of all objects. 
        '''
        return {**self.cubes, **self.cylinders}
    
    def cubes_in_cylinders(self):
        '''Outputs a list of 0, 1, -1.Each item refers to a pair cube/cylinder and is:
            1/-1 if the cube is inside the cylinder and the type is compatible/not-compatible
            0 if the cube is outside the cylinder
        '''
        res = []
        for cube in self.cubes.values():
            inside = 0
            for cyl in self.cylinders.values():
                if cube.is_inside(cyl):
                    if cyl.type_code==cube.type_code:
                        inside = 1
                    else:
                        inside = -1
            res.append(inside)
        return res

    def check_subgoals(self):
        for cube_id, cube in self.cubes.items():
            for cyl in self.cylinders.values():
                if cube.is_inside(cyl) and cyl.type_code==cube.type_code:
                    self.cubes_subgoals[cube_id] = True
                else:
                    self.cubes_subgoals[cube_id] = False
        return self.cubes_subgoals
        
    
    def all_cubes_in_cylinders(self):
        check = np.array(self.cubes_in_cylinders())
        if check.prod() == 1:
            return True
        else:
            return False
    
    def get_distances(self):
        dist = []
        for cube in self.cubes.values():
            for cyl in self.cylinders.values():
                if cube.type_code==cyl.type_code:
                    dist_i = math.sqrt((cube.position[0]-cyl.position[0])**2 + 
                                       (cube.position[1]-cyl.position[1])**2)
                    dist.append(dist_i)
        return dist

    def cylinder_of_type(self, code):
        for cylinder in self.cylinders.values():
            if cylinder.type_code == code:
                return cylinder
        return None           

    def cube_of_type(self, code):
        for cube in self.cubes.values():
            if cube.type_code == code:
                return cube
        return None        




            




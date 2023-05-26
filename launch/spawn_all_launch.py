from simple_launch import SimpleLauncher
import random
from math import pi

sl = SimpleLauncher(use_sim_time = False)
sl.declare_arg('n', 4, description = 'How many turtles')


# we need to use opaque function to resolve arguments as Python types (needed for range)
def launch_setup():

    for k in range(sl.arg('n')):

        name = f'turtle{k+2}'

        x = 10*random.random()
        y = 10*random.random()
        theta = 2*pi * random.random()

        # turtle #n tracks turtle #(n-1)
        target = f'turtle{k+1}'
        
        with sl.group(ns = name):
        
            sl.include('anf_launch', 'spawn_single_launch.py',
                        launch_arguments={'x': x,
                                            'y': y,
                                            'theta': theta,
                                            'name': name,
                                            'target': target,
                                            'wait': k})
    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)

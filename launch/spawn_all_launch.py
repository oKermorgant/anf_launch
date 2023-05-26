from simple_launch import SimpleLauncher
import random
from math import pi

sl = SimpleLauncher(use_sim_time = False)
sl.declare_arg('n', 4, description = 'How many turtles')


# we need to use opaque function to resolve arguments as Python types (needed for range)
def launch_setup():

    # turtle 1 is spawned already
    for k in range(2, sl.arg('n')+3):

        turtle_args = {}
        turtle_args['name'] = name = f'turtle{k}'
        turtle_args['x'] = 10*random.random()
        turtle_args['y'] = 10*random.random()
        turtle_args['theta'] = 2*pi * random.random()

        # turtle #n tracks turtle #(n-1)
        turtle_args['target'] = f'turtle{k-1}'
        
        with sl.group(ns = name):
            sl.include('anf_launch', 'spawn_single_launch.py',
                        launch_arguments=turtle_args)

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)

from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time = False)
    sl.declare_arg('x', 0.)
    sl.declare_arg('y', 0.)
    sl.declare_arg('theta', 0.)
    sl.declare_arg('name', 'turtle')
    sl.declare_arg('target', '')
    sl.declare_arg('wait', 1)

    # spawn the turtle anyway with name and pose
    # done through a service call in turtlesim
    sl.service('/spawn', request=sl.arg_map('x','y','theta','name'))

    # tracking node with target and delay to start
    sl.node('anf_launch', 'track', parameters=sl.arg_map('target','wait'))

    return sl.launch_description()

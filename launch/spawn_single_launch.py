from simple_launch import SimpleLauncher


sl = SimpleLauncher(use_sim_time = False)
sl.declare_arg('x', 0.)
sl.declare_arg('y', 0.)
sl.declare_arg('theta', 0.)
sl.declare_arg('name', 'turtle')
sl.declare_arg('target', '')
sl.declare_arg('wait', 1)

def launch_setup():

    # spawn the turtle anyway
    sl.node('anf_launch', 'spawn',
        parameters = sl.arg_map('x','y','theta','name'))


    if len(sl.arg('target')) > 0:
        sl.node('anf_launch', 'track', parameters=sl.arg_map('target','wait'))
    else:
        # no target, manual control
        sl.node('slider_publisher', 'slider_publisher',
                name = sl.arg('name'),
                arguments=[sl.find('anf_launch', 'Turtle.yaml')])

    return sl.launch_description()

# wrap the above function as a launch description
generate_launch_description = sl.launch_description(launch_setup)

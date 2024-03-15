from simple_launch import SimpleLauncher

sl = SimpleLauncher()
sl.declare_arg('x', 0.)
sl.declare_arg('inc', False)


def launch_setup():

    print('root.x =', sl.arg('x'))

    if sl.arg('inc'):
        sl.include('anf_launch', 'nested_launch.py', launch_arguments={'x': sl.arg('x')+1})
    else:
        sl.include('anf_launch', 'nested_launch.py', launch_arguments={})

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)

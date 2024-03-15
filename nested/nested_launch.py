from simple_launch import SimpleLauncher

sl = SimpleLauncher()
sl.declare_arg('x', -1)  # this value is overwritten by root_launch anyway
sl.declare_arg('y', 0.)


def launch_setup():
    print('nested.x =', sl.arg('x'))
    print('nested.y =', sl.arg('y'))
    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)

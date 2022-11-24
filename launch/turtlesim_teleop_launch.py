from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    b = sl.declare_arg('b', 255)
    g = sl.declare_arg('g', 0)
    r = sl.declare_arg('r', 0)
    joy = sl.declare_arg('joy_config', 'xbox')
    turtle = sl.declare_arg('turtle', 'turtle1')
    
    # run turtlesim with turtle1
    params = {}
    for key in 'rgb':
        params['background_'+key] = sl.arg(key)

    sl.node('turtlesim', 'turtlesim_node', parameters=params)
    
    with sl.group(ns=turtle):

        sl.node('joy', 'joy_node')
        sl.node('teleop_twist_joy', 'teleop_node',
                parameters=[sl.find('teleop_twist_joy', 'xbox.config.yaml')])
        #sl.include('teleop_twist_joy', 'teleop-launch.py',
                   #launch_arguments = {'joy_config': joy})




    return sl.launch_description()

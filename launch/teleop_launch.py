from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    turtle = sl.declare_arg('turtle', 'turtle1')

    with sl.group(ns=sl.arg('turtle')):
        sl.node('joy', 'joy_node')
        sl.node('teleop_twist_joy', 'teleop_node',
            parameters=[sl.find('anf_launch', 'xbox.config.yaml')])
            #parameters=[sl.find('teleop_twist_joy', 'xbox.config.yaml')])   # wrong namespace

    return sl.launch_description()

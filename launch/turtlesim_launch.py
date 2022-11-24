from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    sl.declare_arg('manual', False)
    sl.declare_arg('control',True)
    b = sl.declare_arg('b', 255)
    g = sl.declare_arg('g', 0)
    r = sl.declare_arg('r', 0)
    
    # run turtlesim with turtle1
    params = dict(('background_'+key,sl.arg(key)) for key in 'rgb')
    sl.node('turtlesim', 'turtlesim_node', parameters=params)
    
    # run the open-loop or manual control
    with sl.group(ns='turtle1',if_arg='control'):
        
        with sl.group(unless_arg='manual'):
            # open loop
            sl.node('anf_launch', 'loop', respawn = True, output='screen')
        
        with sl.group(if_arg='manual'):
            # manual control
            sl.node('slider_publisher', 'slider_publisher',name='turtle1',
                    arguments=[sl.find('anf_launch', 'Turtle.yaml')])
    
    return sl.launch_description()

from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    sl.declare_arg('manual', False)
    
    # run turtlesim with turtle1
    sl.node('turtlesim', 'turtlesim_node')
    
    # run the open-loop or manual control
    with sl.group(ns='turtle1'):
        
        with sl.group(unless_arg='manual'):
            # open loop
            sl.node('anf_launch', 'loop', respawn = True, output='screen')
        
        with sl.group(if_arg='manual'):
            # manual control
            sl.node('slider_publisher', 'slider_publisher',name='turtle1',
                    arguments=[sl.find('anf_launch', 'Turtle.yaml')])
    
    return sl.launch_description()

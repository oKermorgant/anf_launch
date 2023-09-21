from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    
    # load robot desription
    with sl.group(ns = 'bb8'):
        sl.robot_state_publisher('anf_launch', 'bb8.xacro')
        sl.node('anf_launch','bb8_sim.py', output='screen')
    
    # run RViz2
    rviz_config_file = sl.find('anf_launch', 'bb8.rviz')
    sl.rviz(rviz_config_file)
        
    return sl.launch_description()

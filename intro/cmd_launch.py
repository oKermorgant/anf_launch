from simple_launch import SimpleLauncher

use_remap = True


def generate_launch_description():
    
    sl = SimpleLauncher()
    sl.declare_arg('Kp', 0.1)
    
    # run command node
    if use_remap:
        sl.node('anf_launch', 'control.py', output='screen',
                remappings = {'angle_setpoint': 'setpoint'},
                parameters = {'Kp': sl.arg('Kp')})
    else:
        sl.node('anf_launch', 'control.py', output='screen')

    # run slider - no remapping
    sl.node('slider_publisher',
            arguments = [sl.find('anf_launch', 'setpoint.yaml')])

    return sl.launch_description()

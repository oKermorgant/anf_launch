from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher()

    sl.node('anf_launch', 'control.py',
            remappings = {'angle_setpoint': 'setpoint'},
            parameters = {'Kp': 0.8})

    sl.node('slider_publisher', 'slider_publisher',
            arguments = [sl.find('anf_launch', 'setpoint.yaml')])



    return sl.launch_description()

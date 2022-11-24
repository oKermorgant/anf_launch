from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    sl.declare_arg('x', 0.)
    sl.declare_arg('y', 0.)
    sl.declare_arg('theta', 0.)
    sl.declare_arg('name', 'turtle')
    sl.declare_arg('target', '')
    sl.declare_arg('wait', 1)
    
    # manual control if no target
    has_target = sl.py_eval("len('", sl.arg('target'), "') > 0")
    
    # spawn the turtle anyway
    sl.node('anf_launch', 'spawn', 
        parameters = sl.arg_map(('x','y','theta','name')))
    
    with sl.group(if_condition = has_target):
        sl.node('anf_launch', 'track', parameters=sl.arg_map('target','wait'))
        
    with sl.group(unless_condition = has_target):
        sl.node('slider_publisher', 'slider_publisher', 
                name = sl.arg('name'),
                arguments=[sl.find('anf_launch', 'Turtle.yaml')])
    
    return sl.launch_description()

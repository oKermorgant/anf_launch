from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    sl.declare_arg('manual', False)
    
    # poses of the turtles
    turtles = ((2.,1.,0.),
               (1,2,0),
               (10,1,0),
               (1,10,0))
    
    for n,(x,y,theta) in enumerate(turtles):
        name = f'turtle{n+2}'
        
        target = sl.py_eval("'' if ", sl.arg('manual'), " else ", f'"turtle{n+1}"') 
        x = float(x)
        y = float(y)
        theta = float(theta)
        
        with sl.group(ns = name):
        
            sl.include('anf_launch', 'single_launch.py',
                        launch_arguments={'x': x,
                                            'y': y,
                                            'theta': theta,
                                            'name': name,
                                            'target': target})    
    return sl.launch_description()

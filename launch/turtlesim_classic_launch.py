from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory as lookup

def generate_launch_description():
    
    ld = LaunchDescription()
    
    # run turtlesim with turtle1
    sim_node = Node(package='turtlesim', executable='turtlesim_node')
    ld.add_action(sim_node)
    
    # declare a (Boolean) argument
    ld.add_action(DeclareLaunchArgument('manual', default_value='False'))
    manual = LaunchConfiguration('manual')

    ld.add_action(DeclareLaunchArgument('control', default_value='True'))
    control = LaunchConfiguration('control')
    
    # open-loop node
    loop_node = Node(package='anf_launch', executable='loop', condition = UnlessCondition(manual))
    
    # manual node
    slider_config = f"{lookup('anf_launch')}/launch/Turtle.yaml"
    slider_node = Node(package='slider_publisher', executable='slider_publisher', name='turtle1',
                       condition = IfCondition(manual),
                       arguments = [slider_config])

    
    # namespaced group with those 2 nodes under the control condition
    namespaced = GroupAction([PushRosNamespace('turtle1'),loop_node, slider_node], condition=IfCondition(control))
    ld.add_action(namespaced)
    
    return ld

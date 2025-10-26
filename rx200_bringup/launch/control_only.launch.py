from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    '''Ensure you have the '''
    ld = LaunchDescription()

    dc = DeclareLaunchArgument(
        'default_gr_state',
        default_value=True,
        description='What should the state of the gripper be'
    )

    moveit_control = Node(
        package='rx200_moveit_control',
        executable='rx200_moveit_client',
        parameters=[{
            'start_state_gripper': LaunchConfiguration('default_gr_state')
        }]
    )

    #Create your node actions using the Node Object
    # add Nodes using ld.add_action
    ld.add_action(moveit_control)

    return ld
    
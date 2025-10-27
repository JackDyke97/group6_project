from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    '''Ensure you have the '''
    ld = LaunchDescription()

    declare_gripper_state = DeclareLaunchArgument(
        'default_gr_state',
        default_value='True',
        description='The inital state of the gripper' #open = True, Closed= False
    )

    declare_waypoints = DeclareLaunchArgument(
        'waypoints',
        default_value="'[(0.35,0.0,0.25)]'",
        description='List of waypoints as [(x1, y1, z1), (x2, y2, z2), etc]'
    )

    moveit_control = Node(
        package='rx200_moveit_control',
        executable='rx200_moveit_client',
        name='rx200_moveit_control',
        parameters=[{
            'start_state_gripper': LaunchConfiguration('default_gr_state'),
            'waypoints': LaunchConfiguration('waypoints')
        }]
    )

    #Create your node actions using the Node Object
    # add Nodes using ld.add_action
    ld.add_action(moveit_control)
    ld.add_action(declare_gripper_state)
    ld.add_action(declare_waypoints)

    return ld
    
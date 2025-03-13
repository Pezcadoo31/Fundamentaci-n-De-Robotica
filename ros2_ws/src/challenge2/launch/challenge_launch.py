import launch
import launch_ros.actions
import launch.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Grupo 1
        launch_ros.actions.Node(
            package='challenge2',
            executable='ctrl',
            name='ctrl',
            namespace='group1',
            output='screen',
            parameters=[{'Kp': 2.0, 'Ki': 1.0, 'Kd': 0.5, 'sample_time': 0.01, 'integral_limit': 10.0}]
        ),
        launch_ros.actions.Node(
            package='challenge2',
            executable='motor_sys',
            name='motor_sys',
            namespace='group1',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='challenge2',
            executable='sp_gen',
            name='sp_gen',
            namespace='group1',
            output='screen'
        ),

        # Grupo 2
        launch_ros.actions.Node(
            package='challenge2',
            executable='ctrl',
            name='ctrl',
            namespace='group2',
            output='screen',
            parameters=[{'Kp': 2.0, 'Ki': 1.0, 'Kd': 0.5, 'sample_time': 0.01, 'integral_limit': 10.0}]
        ),
        launch_ros.actions.Node(
            package='challenge2',
            executable='motor_sys',
            name='motor_sys',
            namespace='group2',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='challenge2',
            executable='sp_gen',
            name='sp_gen',
            namespace='group2',
            output='screen'
        ),

        # Grupo 3
        launch_ros.actions.Node(
            package='challenge2',
            executable='ctrl',
            name='ctrl',
            namespace='group3',
            output='screen',
            parameters=[{'Kp': 2.0, 'Ki': 1.0, 'Kd': 0.5, 'sample_time': 0.01, 'integral_limit': 10.0}]
        ),
        launch_ros.actions.Node(
            package='challenge2',
            executable='motor_sys',
            name='motor_sys',
            namespace='group3',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='challenge2',
            executable='sp_gen',
            name='sp_gen',
            namespace='group3',
            output='screen'
        ),

        # Lanzar rqt_plot usando la ruta completa
        launch.actions.ExecuteProcess(
            cmd=['/opt/ros/humble/lib/rqt_plot/rqt_plot'],  # Ruta completa de rqt_plot
            output='screen',
            shell=True,
            emulate_tty=True  # Forzar la ejecución en un pseudo-terminal
        ),

        # Lanzar rqt_graph
        launch.actions.ExecuteProcess(
            cmd=['rqt_graph'],
            output='screen',
            emulate_tty=True
        ),

        # Lanzar rqt_reconfigure
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_reconfigure', 'rqt_reconfigure'],
            output='screen',
            emulate_tty=True
        )
    ])
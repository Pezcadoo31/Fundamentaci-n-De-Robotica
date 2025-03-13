import launch
import launch_ros.actions
import launch.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Inicia el micro-ROS Agent para comunicar ROS 2 con el microcontrolador
        launch_ros.actions.Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyUSB0']  # Ajusta el puerto según tu microcontrolador
        ),

        # Ejecutar rqt_plot para visualizar las señales
        launch.actions.ExecuteProcess(
            cmd=['rqt_plot', '/motor_input/data'],
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

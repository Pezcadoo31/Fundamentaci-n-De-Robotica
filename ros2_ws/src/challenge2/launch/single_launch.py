import launch
import launch_ros.actions
import launch.actions
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ruta al archivo YAML de parámetros
    params_file = os.path.join(
        get_package_share_directory('challenge2'),
        'config',
        'pid_params.yaml'
    )

    return launch.LaunchDescription([
        # Nodo del controlador (ctrl)
        launch_ros.actions.Node(
            package='challenge2',
            executable='ctrl',
            name='ctrl',
            output='screen',
            parameters=[params_file]  # Cargar parámetros desde el archivo YAML
        ),

        # Nodo del sistema del motor (motor_sys)
        launch_ros.actions.Node(
            package='challenge2',
            executable='motor_sys',
            name='motor_sys',
            output='screen',
            parameters=[params_file]  # Cargar parámetros desde el archivo YAML
        ),

        # Nodo generador de set point (sp_gen)
        launch_ros.actions.Node(
            package='challenge2',
            executable='sp_gen',
            name='sp_gen',
            output='screen',
            parameters=[params_file]  # Cargar parámetros desde el archivo YAML
        ),

        # Lanzar rqt_plot para visualizar señales en tiempo real
        launch.actions.ExecuteProcess(
            cmd=['/opt/ros/humble/lib/rqt_plot/rqt_plot'],
            output='screen',
            shell=True,
            emulate_tty=True
        ),

        # Lanzar rqt_graph para ver la conexión entre nodos
        launch.actions.ExecuteProcess(
            cmd=['rqt_graph'],
            output='screen',
            emulate_tty=True
        ),

        # Lanzar rqt_reconfigure para modificar parámetros en tiempo real
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_reconfigure', 'rqt_reconfigure'],
            output='screen',
            emulate_tty=True
        )
    ])
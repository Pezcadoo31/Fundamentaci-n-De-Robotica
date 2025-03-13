from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodo Generador de Señales (signal_generator)
    signal_generator_node = Node(
        name='signal_generator',  
        package='signal_processing',  
        executable='signal_generator', 
        output='screen'  
    )

    # Nodo Procesador de Señales (process)
    process_node = Node(
        name='process',  
        package='signal_processing',  
        executable='process',  
        output='screen'  
    )

    # rqt_plot para graficar las señales
    rqt_plot_node = Node(
        name='rqt_plot',  
        package='rqt_plot',  
        executable='rqt_plot',  
        arguments=['/signal', '/proc_signal'],  # Tópicos a graficar 
        output='screen'  
    )

    # rqt_graph para visualizar el grafo de nodos y conexiones
    rqt_graph_node = Node(
        name='rqt_graph',
        package='rqt_graph',
        executable='rqt_graph',
        output='screen'
    )

    # Retornar la descripción del lanzamiento
    l_d = LaunchDescription([
        signal_generator_node, 
        process_node, 
        rqt_plot_node,
        rqt_graph_node  # Agregar el nodo rqt_graph
    ])
    
    return l_d

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown
import sys
import os

class MissingArgumentError(Exception):
    pass

def generate_launch_description():
    # Verificar si se pasó el argumento json_config_path
    if 'json_config_path:=' not in ' '.join(sys.argv):
        error_message = "\n" + "="*80 + "\n"
        error_message += "ERROR: Missing required parameter 'json_config_path'\n\n"
        error_message += "The anchor_publisher node requires a JSON configuration file path.\n"
        error_message += "Please provide it using:\n\n"
        error_message += "    ros2 launch uwb_ranging anchor_publisher.launch.py json_config_path:=/path/to/your/config.json\n\n"
        error_message += "Example JSON format:\n"
        error_message += """
{
  "anchors": [
    {
      "id": "0x673A",
      "label": "Anchor1",
      "position": {
        "x": 0.0,
        "y": 0.0,
        "z": 2.5
      }
    },
    ...
  ]
}
        """
        error_message += "\n" + "="*80
        print(error_message)
        
        # Crear un LaunchDescription que solo emite un evento de cierre
        return LaunchDescription([
            EmitEvent(event=Shutdown(reason="Missing required parameter 'json_config_path'"))
        ])
    
    # Definir el argumento de línea de comando para la ruta del archivo JSON
    json_config_path_arg = DeclareLaunchArgument(
        'json_config_path',
        description='Absolute path to the JSON file with the anchor configuration',
    )
    
    # Referencia al argumento en los parámetros del nodo
    json_config_path = LaunchConfiguration('json_config_path')
    
    # Verificar que el archivo exista
    path_value = None
    for arg in sys.argv:
        if arg.startswith('json_config_path:='):
            path_value = arg.split('json_config_path:=')[1]
            break
    
    if path_value and not os.path.exists(os.path.expanduser(path_value)):
        error_message = f"\nERROR: The specified JSON file '{path_value}' does not exist.\n"
        error_message += "Please check the path and make sure the file exists.\n"
        print(error_message)
        
        return LaunchDescription([
            EmitEvent(event=Shutdown(reason=f"JSON file '{path_value}' not found"))
        ])
    
    # Definir el nodo anchor_publisher
    anchor_publisher_node = Node(
        package='ros2_uwb_tools',
        executable='anchor_publisher',
        name='anchor_publisher',
        output='screen',
        parameters=[
            {'json_config_path': json_config_path}
        ]
    )
    
    return LaunchDescription([
        LogInfo(msg=f"Starting Anchor Publisher with config file: {json_config_path}"),
        json_config_path_arg,
        anchor_publisher_node
    ]) 
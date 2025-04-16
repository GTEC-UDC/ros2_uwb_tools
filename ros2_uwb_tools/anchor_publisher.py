#!/usr/bin/env python

""" MIT License

Copyright (c) 2025 Group of Electronic Technology and Communications. University of A Coruna.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. """

import rclpy
from rclpy.node import Node
import json
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import time
import os

class Beacon:
    def __init__(self, id, x=0.0, y=0.0, z=0.0):
        self.id = id
        self.position = Point(x=x, y=y, z=z)

class AnchorInfo:
    def __init__(self):
        self.id = 0xff
        self.label = ""
        self.x = 0.0
        self.y = 0.0
        self.z = 3.0
        self.tagRangeCorection = [0] * 64

class AnchorPublisher(Node):
    def __init__(self):
        super().__init__('anchor_publisher')
        
        # Declare parameters
        self.declare_parameter('json_config_path', '')
        
        # Create publisher
        self.ros_pub_anchors = self.create_publisher(MarkerArray, '/gtec/toa/anchors', 10)
        
        # Initialize arrays and variables
        self._ancArray = [AnchorInfo() for _ in range(64)]
        self.beacons = []
        
        # Get configuration file path parameter
        json_config_path = self.get_parameter('json_config_path').get_parameter_value().string_value
        
        if not json_config_path:
            self.get_logger().error('json_config_path parameter is required but not provided')
            self.get_logger().error('Please provide a path to a JSON configuration file')
            rclpy.shutdown()
            return
            
        # Verify the file exists
        if not os.path.exists(json_config_path):
            self.get_logger().error(f'Config file does not exist: {json_config_path}')
            rclpy.shutdown()
            return
            
        self.get_logger().info(f'Loading anchor configuration from: {json_config_path}')
        
        # Load configuration from the JSON file
        config_loaded = self.load_config_from_file(json_config_path)
        if config_loaded:
            self.get_logger().info('Beacon configuration loaded successfully')
            
            # Create a timer to publish anchors periodically
            self.create_timer(1.0, self.timer_callback)
            
            self.get_logger().info('Anchor Publisher node initialized')
        else:
            self.get_logger().error('Failed to load beacon configuration')
            rclpy.shutdown()
            
    def timer_callback(self):
        self.publishAnchors()
        
    def load_config_from_file(self, config_file_path):
        try:
            # Read the JSON configuration file
            with open(config_file_path, 'r') as file:
                config_data = json.load(file)
                
            anchors = config_data.get('anchors', [])
            
            if not anchors:
                self.get_logger().error('No anchors found in configuration file')
                return False
                
            self.get_logger().info(f'Found {len(anchors)} anchors in configuration file')
            
            # Process each anchor in the configuration
            for index, anchor in enumerate(anchors):
                id_text = anchor.get('id', '')
                
                # Convert hex ID to integer
                try:
                    beacon_id = int(id_text, 16)
                except ValueError:
                    self.get_logger().error(f"Failed to parse ID: {id_text}")
                    continue
                    
                # Get anchor information
                label = anchor.get('label', '')
                position = anchor.get('position', {})
                x = float(position.get('x', 0.0))
                y = float(position.get('y', 0.0))
                z = float(position.get('z', 0.0))
                
                # Store anchor information
                self._ancArray[index].id = beacon_id
                self._ancArray[index].label = label
                self._ancArray[index].x = x
                self._ancArray[index].y = y
                self._ancArray[index].z = z
                
                # Initialize correction values to 0
                for j in range(8):
                    self._ancArray[index].tagRangeCorection[j] = 0
                
                # Add beacon to list
                self.beacons.append(Beacon(beacon_id, x, y, z))
                
                self.get_logger().info(f"Anchor {id_text} ({beacon_id}): ({x}, {y}, {z})")
                
            return True
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing JSON: {str(e)}")
            return False
        except Exception as ex:
            self.get_logger().error(f"Error loading configuration: {str(ex)}")
            return False
            
    def publishAnchors(self):
        marker_array = MarkerArray()
        
        for i, beacon in enumerate(self.beacons):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = beacon.id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = beacon.position.x
            marker.pose.position.y = beacon.position.y
            marker.pose.position.z = beacon.position.z
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0  # Don't forget to set the alpha!
            
            # Set color based on index
            if i == 0:
                marker.color.r = 0.6
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif i == 1:
                marker.color.r = 0.6
                marker.color.g = 0.2
                marker.color.b = 0.0
            elif i == 2:
                marker.color.r = 0.6
                marker.color.g = 0.5
                marker.color.b = 1.0
            elif i == 3:
                marker.color.r = 0.6
                marker.color.g = 0.8
                marker.color.b = 0.0
            elif i == 4:
                marker.color.b = 0.6
                marker.color.g = 0.0
                marker.color.r = 0.0
            elif i == 5:
                marker.color.b = 0.6
                marker.color.g = 0.2
                marker.color.r = 0.0
            elif i == 6:
                marker.color.b = 0.6
                marker.color.g = 0.5
                marker.color.r = 1.0
            elif i == 7:
                marker.color.b = 0.6
                marker.color.g = 0.8
                marker.color.r = 0.0
                
            marker_array.markers.append(marker)
            
        self.ros_pub_anchors.publish(marker_array)
        self.get_logger().debug('Published anchor markers')

def main(args=None):
    rclpy.init(args=args)
    
    node = AnchorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
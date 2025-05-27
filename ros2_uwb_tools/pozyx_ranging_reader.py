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

"""
Pozyx UWB Ranging Reader for ROS 2

This node interfaces with a Pozyx UWB device to perform ranging measurements
between a tag and multiple anchors. It publishes the ranging data to a ROS 2 topic.

The node requires anchor positions to be published on /gtec/toa/anchors 
as a MarkerArray message before it can begin ranging measurements.

Key features:
- Supports both local and remote Pozyx devices
- Robust error handling and recovery
- Background thread for continuous ranging operations
- Debug levels for detailed logging
- Compatible with various Pozyx firmware versions
"""

import rclpy
from rclpy.node import Node
from time import sleep
from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_3D, Coordinates, POZYX_SUCCESS, PozyxConstants, version,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, DeviceList, 
                     PozyxRegisters, DeviceRange, EulerAngles, Acceleration, Quaternion, AngularVelocity)
                     
from pypozyx.tools.version_check import perform_latest_version_check

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray
from gtec_msgs.msg import Ranging
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

# Define constants for registers that might not be available in all pypozyx versions
try:
    POZYX_WHO_AM_I = 0x0  # Identification register
    POZYX_FIRMWARE_VER = 0x1  # Firmware version
    POZYX_NETWORK_ID = PozyxRegisters.POZYX_NETWORK_ID
except:
    # Use known values if not available in the library
    POZYX_WHO_AM_I = 0x0 
    POZYX_FIRMWARE_VER = 0x1
    POZYX_NETWORK_ID = 0x1A  # Network ID register

class PozyxRanger(Node):
    """
    Main ROS 2 node class for interfacing with Pozyx UWB devices.
    
    This node handles the connection to the Pozyx device, configuration of
    ranging parameters, and processing of anchor positions. It uses a helper
    class ReadyToRange to perform the actual ranging operations.
    """
    def __init__(self):
        super().__init__('pozyx_ranger')
        
        # Declare parameters with default values
        self.declare_parameter('targetDeviceId', '0x0000')  # Remote device ID, or 0x0000 for local device
        self.declare_parameter('serial', '/dev/ttyUSB0')    # Serial port for Pozyx connection
        self.declare_parameter('debug_level', 0)  # 0 = none, 1 = normal, 2 = verbose
        self.declare_parameter('publish_imu', True)  # Enable/disable IMU publishing
        
        # Get parameters
        targetDeviceIdString = self.get_parameter('targetDeviceId').get_parameter_value().string_value
        serial_port = self.get_parameter('serial').get_parameter_value().string_value
        self.debug_level = self.get_parameter('debug_level').get_parameter_value().integer_value
        self.publish_imu = self.get_parameter('publish_imu').get_parameter_value().bool_value
        
        self.targetDeviceId = int(targetDeviceIdString, 16)
        
        # Create publisher for ranging data
        self.pub_ranging = self.create_publisher(Ranging, '/gtec/uwb/ranging/pozyx', 100)
        
        # Create publisher for IMU data if enabled
        self.pub_imu = None
        if self.publish_imu:
            imu_topic = f'/gtec/uwb/imu/pozyx_{self.targetDeviceId}'
            self.pub_imu = self.create_publisher(Imu, imu_topic, 100)
            self.get_logger().info(f"IMU publishing enabled on topic: {imu_topic}")
        
        # Set rate for ranging measurements
        self.rate = self.create_rate(5)  # 5 Hz
        
        self.get_logger().info("=========== POZYX Range reader ============")
        self.get_logger().info(f"targetDeviceId: {targetDeviceIdString}")
        self.get_logger().info(f"serial port: {serial_port}")
        self.get_logger().info(f"debug level: {self.debug_level}")
        self.get_logger().info("=========== [---------------] ============")
        
        # Check pypozyx version if needed
        check_pypozyx_version = False
        if check_pypozyx_version:
            perform_latest_version_check()
            
        if serial_port is None:
            self.get_logger().error("No Pozyx connected. Check your USB cable or your driver!")
            return
            
        try:
            # Establish connection to Pozyx device
            self.pozyxSerial = PozyxSerial(serial_port)
            self.get_logger().info(f"Successfully connected to Pozyx device at {serial_port}")
            
            # Basic connection verification (avoiding registers that might not exist)
            try:
                # Try to read network ID as basic communication check
                network_id = SingleRegister()
                if self.pozyxSerial.getRead(POZYX_NETWORK_ID, network_id) == POZYX_SUCCESS:
                    self.get_logger().info(f"Pozyx device network ID: 0x{network_id.value:04x}")
                else:
                    self.get_logger().warn("Could not read device network ID - device might not be responding properly")
            except Exception as e:
                self.get_logger().warn(f"Could not perform device validation: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Pozyx device: {e}")
            self.get_logger().error(f"Check if the serial port {serial_port} exists and has correct permissions.")
            return
            
        # Initialize ReadyToRange with the configured parameters
        self.r = ReadyToRange(self.pozyxSerial, self.pub_ranging, self.targetDeviceId, self, self.debug_level, self.pub_imu)
        self.hasAnchors = False
        
        # Create subscription for anchor position messages
        self.create_subscription(MarkerArray, '/gtec/toa/anchors', self.callbackAnchorsMessage, 10)
        
        self.get_logger().info("Waiting for anchor positions...")
        
        # Check if remote or local device will be used
        if self.targetDeviceId != 0:
            self.get_logger().info(f"Using remote device with ID: 0x{self.targetDeviceId:04x}")
        else:
            self.get_logger().info("Using local device (no remote)")

    def setAnchors(self, anchors):
        """
        Configure the anchors and start ranging measurements.
        
        Args:
            anchors: List of DeviceCoordinates objects containing anchor information
        """
        self.r.setAnchors(anchors)
        self.get_logger().info(f"Starting ranging measurements with {len(anchors)} anchors")
        
        # Log device capabilities - do this safely to handle different library versions
        if self.debug_level >= 1:
            try:
                self.get_logger().info("Checking UWB settings (if available):")
                
                try:
                    uwb_channel = SingleRegister()
                    if self.pozyxSerial.getUWBChannel(uwb_channel) == POZYX_SUCCESS:
                        self.get_logger().info(f"UWB Channel: {uwb_channel.value}")
                    else:
                        self.get_logger().warn("Could not retrieve UWB channel")
                except Exception as e:
                    self.get_logger().warn(f"Error getting UWB channel: {e}")
                    
                try:
                    uwb_bitrate = SingleRegister()
                    if self.pozyxSerial.getUWBBitrate(uwb_bitrate) == POZYX_SUCCESS:
                        bitrates = {0: "110 kbps", 1: "850 kbps", 2: "6.8 Mbps"}
                        bitrate_str = bitrates.get(uwb_bitrate.value, f"Unknown ({uwb_bitrate.value})")
                        self.get_logger().info(f"UWB Bitrate: {bitrate_str}")
                except Exception as e:
                    self.get_logger().warn(f"Error getting UWB bitrate: {e}")
                
                try:
                    uwb_prf = SingleRegister()
                    if self.pozyxSerial.getUWBPRF(uwb_prf) == POZYX_SUCCESS:
                        prf_str = "16 MHz" if uwb_prf.value == 1 else "64 MHz" if uwb_prf.value == 2 else f"Unknown ({uwb_prf.value})"
                        self.get_logger().info(f"UWB PRF: {prf_str}")
                except Exception as e:
                    self.get_logger().warn(f"Error getting UWB PRF: {e}")
                    
            except Exception as e:
                self.get_logger().warn(f"Error checking UWB settings: {e}")
        
        # Start ranging thread to avoid blocking the main node execution
        try:
            from threading import Thread
            
            # Define a separate function for the ranging loop
            def ranging_loop():
                last_error_time = 0.0
                error_count = 0
                import time
                
                while rclpy.ok():
                    try:
                        self.r.loop()
                        self.rate.sleep()
                        # Reset error counter if no problems occurred
                        error_count = 0
                    except Exception as e:
                        current_time = time.time()
                        error_count += 1
                        
                        # Limit error logging to avoid filling up the logs
                        if current_time - last_error_time > 5.0 or error_count <= 3:
                            self.get_logger().error(f"Error in ranging loop: {e}")
                            last_error_time = current_time
                        
                        # Try to recover from the error
                        try:
                            time.sleep(1.0)  # Short pause to avoid system overload
                        except:
                            pass
            
            # Start ranging thread as daemon (will terminate when main program ends)
            ranging_thread = Thread(target=ranging_loop, daemon=True)
            ranging_thread.start()
            self.get_logger().info("Ranging measurements started in background thread")
            
        except Exception as e:
            self.get_logger().error(f"Failed to start ranging thread: {e}")
            # Fallback: use original blocking loop if thread creation fails
            self.get_logger().info("Falling back to blocking ranging loop")
            while rclpy.ok():
                try:
                    self.r.loop()
                    self.rate.sleep()
                except Exception as e:
                    self.get_logger().error(f"Error in ranging loop: {e}")
                    try:
                        import time
                        time.sleep(1.0)  # Short pause to avoid system overload
                    except:
                        pass

    def callbackAnchorsMessage(self, anchorsMsg):
        """
        Callback for anchor position messages.
        
        Processes anchor positions received from the /gtec/toa/anchors topic
        and initializes ranging measurements.
        
        Args:
            anchorsMsg: MarkerArray message containing anchor positions
        """
        if not self.hasAnchors:
            self.get_logger().info("Anchors Msg received")
            anchors = []
            for anchor in anchorsMsg.markers:
                # Convert from meters to mm for Pozyx
                deviceCoordinates = DeviceCoordinates(anchor.id, 1, Coordinates(anchor.pose.position.x*1000, anchor.pose.position.y*1000, anchor.pose.position.z*1000))
                anchors.append(deviceCoordinates)
            self.setAnchors(anchors)
            self.hasAnchors = True


class ReadyToRange:
    """
    Helper class for performing ranging operations with the Pozyx device.
    
    This class handles the ranging logic, anchor configuration, and data publishing.
    It maintains a list of anchors, performs ranging measurements, and
    publishes the results to the specified ROS 2 topic.
    """
    def __init__(self, pozyx, ranging_pub, remote_id=None, node=None, debug_level=1, imu_pub=None):
        """
        Initialize the ReadyToRange object.
        
        Args:
            pozyx: PozyxSerial object for communicating with the device
            ranging_pub: ROS 2 publisher for ranging messages
            remote_id: ID of remote device (None for local device)
            node: ROS 2 node for logging
            debug_level: Level of debug information (0=none, 1=normal, 2=verbose)
            imu_pub: ROS 2 publisher for IMU messages (optional)
        """
        self.pozyx = pozyx
        self.remote_id = remote_id
        self.ranging_pub = ranging_pub
        self.imu_pub = imu_pub
        self.seq = -1
        self.node = node  # Store reference to the ROS node for logging
        self.debug_level = debug_level
        self.ranging_count = 0
        self.ranging_success_count = 0
        self.last_stats_time = 0
        
        # Initialize variables to track success rate
        if self.node:
            import time
            self.last_stats_time = time.time()

    def setAnchors(self, anchors):
        """
        Configure the anchors for ranging measurements.
        
        This method stores the anchor list and performs device discovery
        and configuration on the Pozyx device.
        
        Args:
            anchors: List of DeviceCoordinates objects containing anchor information
        """
        self.anchors = anchors
        self.printPublishAnchorConfiguration()
        
        # Store device list on the Pozyx for ranging
        if self.node and self.debug_level >= 1:
            self.node.get_logger().info("Setting device list on Pozyx...")
            
        try:
            # Try device discovery safely
            try:
                status = self.pozyx.doDiscovery(discovery_type=PozyxConstants.DISCOVERY_ANCHORS_ONLY)
                if self.node:
                    if status == POZYX_SUCCESS:
                        self.node.get_logger().info("Discovery complete")
                        
                        # Read the device list back
                        device_list_read = DeviceList()
                        status = self.pozyx.getDeviceListSize(device_list_read)
                        if status == POZYX_SUCCESS:
                            status = self.pozyx.getDeviceIds(device_list_read)
                            if status == POZYX_SUCCESS:
                                self.node.get_logger().info(f"Found {device_list_read.list_size} devices: {[hex(id) for id in device_list_read.devices[:device_list_read.list_size]]}")
                            else:
                                self.node.get_logger().warn("Could not get device IDs")
                        else:
                            self.node.get_logger().warn("Could not get device list size")
                    else:
                        self.node.get_logger().warn("Discovery failed")
            except Exception as e:
                if self.node:
                    self.node.get_logger().warn(f"Error during discovery: {e}")
            
            # Configure devices safely
            try:
                # Clear the device list in the Pozyx
                status = self.pozyx.clearDevices(remote_id=self.remote_id)
                if self.node and self.debug_level >= 2:
                    if status == POZYX_SUCCESS:
                        self.node.get_logger().info("Cleared device list")
                    else:
                        self.node.get_logger().warn("Failed to clear device list")
                
                # Add each anchor to the device list
                for anchor in self.anchors:
                    status = self.pozyx.addDevice(anchor, self.remote_id)
                    if self.node and self.debug_level >= 2:
                        if status == POZYX_SUCCESS:
                            self.node.get_logger().info(f"Added device 0x{anchor.network_id:04x} to device list")
                        else:
                            self.node.get_logger().warn(f"Failed to add device 0x{anchor.network_id:04x} to device list")
            except Exception as e:
                if self.node:
                    self.node.get_logger().warn(f"Error configuring device list: {e}")
                
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in anchor setup: {e}")

    def loop(self):
        """
        Perform a single ranging cycle with all configured anchors.
        
        This method is called repeatedly to perform ranging measurements
        and publish the results. It handles sequence numbers and statistics.
        """
        self.seq += 1
        if self.seq > 255:
            self.seq = 0
        current_seq = self.seq
        
        # Log statistics every 10 seconds
        if self.node and self.debug_level >= 1:
            import time
            current_time = time.time()
            if current_time - self.last_stats_time > 10:
                if self.ranging_count > 0:
                    success_rate = (self.ranging_success_count / self.ranging_count) * 100
                    self.node.get_logger().info(f"Ranging stats: {self.ranging_success_count}/{self.ranging_count} successful ({success_rate:.1f}%)")
                else:
                    self.node.get_logger().warn("No ranging attempts in the last 10 seconds")
                self.ranging_count = 0
                self.ranging_success_count = 0
                self.last_stats_time = current_time
        
        # Publish IMU data if enabled
        if self.imu_pub is not None:
            self.publishImu()
        
        # Perform ranging with each anchor
        for anchor in self.anchors:
            range_data = DeviceRange()
            try:
                self.ranging_count += 1
                status = self.pozyx.doRanging(anchor.network_id, range_data, self.remote_id)
                
                if status == POZYX_SUCCESS:
                    if range_data.RSS < 0.0:  # Valid RSS values are negative
                        self.ranging_success_count += 1
                        self.publishRanging(anchor.network_id, range_data, current_seq)
                        if self.node and self.debug_level >= 2:
                            self.node.get_logger().info(f"Range to anchor {hex(anchor.network_id)}: {range_data.distance} mm, RSS: {range_data.RSS} dB")
                    else:
                        if self.node and self.debug_level >= 1:
                            self.node.get_logger().warn(f"Invalid RSS value for anchor {hex(anchor.network_id)}: {range_data.RSS} dB")
                else:
                    if self.node and self.debug_level >= 1:
                        try:
                            error_code = SingleRegister()
                            self.pozyx.getErrorCode(error_code)
                            self.node.get_logger().warn(f"Ranging failed for anchor {hex(anchor.network_id)}, error code: 0x{error_code.value:02x}")
                            
                            # Common error codes
                            if error_code.value == 0x01:
                                self.node.get_logger().warn("Error: POZYX_FAILURE - General failure")
                            elif error_code.value == 0x02:
                                self.node.get_logger().warn("Error: POZYX_TIMEOUT - Function timeout")
                            elif error_code.value == 0x03:
                                self.node.get_logger().warn("Error: POZYX_INVALID - Invalid function parameters")
                            elif error_code.value == 0x04:
                                self.node.get_logger().warn("Error: POZYX_ANCHOR_NOT_FOUND - Anchor not found")
                            elif error_code.value == 0x05:
                                self.node.get_logger().warn("Error: POZYX_UWB_BUSY - UWB busy")
                            elif error_code.value == 0x06:
                                self.node.get_logger().warn("Error: POZYX_FUNCTION_NOT_AVAILABLE - Function not available")
                            elif error_code.value == 0x08:
                                self.node.get_logger().warn("Error: POZYX_NO_FLASH - No flash memory")
                        except Exception as e:
                            self.node.get_logger().warn(f"Could not get error code: {e}")
                        
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"Exception in ranging: {e}")
                else:
                    print("################## An exception occurred #########################")
                    print(f"Error: {e}")
                try:
                    # Try to reconnect if connection is lost
                    serial_port = get_first_pozyx_serial_port()
                    if serial_port is None:
                        if self.node:
                            self.node.get_logger().error("No Pozyx connected. Check your USB cable or your driver!")
                        else:
                            print("No Pozyx connected. Check your USB cable or your driver!")
                        quit()
                    self.pozyx = PozyxSerial(serial_port)
                except Exception as reconnect_error:
                    if self.node:
                        self.node.get_logger().error(f"Error reconnecting to Pozyx: {reconnect_error}")

    def publishRanging(self, anchorId, range, seq):
        """
        Publish a ranging measurement to the ROS 2 topic.
        
        Args:
            anchorId: ID of the anchor the measurement was taken with
            range: DeviceRange object containing ranging data
            seq: Sequence number for the measurement
        """
        try:
            ranging = Ranging()
            # Use the correct field names for the Ranging message
            ranging.anchor_id = int(anchorId)
            ranging.tag_id = int(self.remote_id)
            ranging.range = int(range.distance)
            ranging.rss = float(range.RSS)
            ranging.seq = int(seq)
            ranging.error_estimation = 0.00393973  # Constant error estimation for now
            self.ranging_pub.publish(ranging)
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error publishing ranging message: {e}")

    def printPublishAnchorConfiguration(self):
        """
        Log the anchor configuration.
        
        This method prints the details of all configured anchors,
        including their network IDs and 3D coordinates.
        """
        if self.node:
            self.node.get_logger().info("Anchor configuration:")
            for anchor in self.anchors:
                self.node.get_logger().info(f"ANCHOR {hex(anchor.network_id)}: {str(anchor.pos)}")
                sleep(0.025)  # Small delay to ensure log messages are properly displayed
        else:
            print("Anchor configuration:")
            for anchor in self.anchors:
                print(f"ANCHOR {hex(anchor.network_id)}: {str(anchor.pos)}")
                sleep(0.025)

    def publishImu(self):
        """
        Publish IMU data from the Pozyx device to the ROS 2 topic.
        
        Reads quaternion, linear acceleration, and angular velocity from the device
        and publishes them as a sensor_msgs/Imu message.
        """
        try:
            quaternion = Quaternion()
            acceleration = Acceleration()
            angular_velocity = AngularVelocity()
            
            # Get IMU data from Pozyx device
            status_quat = self.pozyx.getQuaternion(quaternion, remote_id=self.remote_id)
            status_accel = self.pozyx.getLinearAcceleration_mg(acceleration, remote_id=self.remote_id)
            status_gyro = self.pozyx.getAngularVelocity_dps(angular_velocity, remote_id=self.remote_id)
            
            # Check if all readings were successful
            if status_quat == POZYX_SUCCESS and status_accel == POZYX_SUCCESS and status_gyro == POZYX_SUCCESS:
                imu = Imu()
                imu.header.frame_id = "base_link"
                imu.header.stamp = self.node.get_clock().now().to_msg()
                
                # Set orientation (quaternion)
                imu.orientation.x = float(quaternion.x)
                imu.orientation.y = float(quaternion.y)
                imu.orientation.z = float(quaternion.z)
                imu.orientation.w = float(quaternion.w)
                
                # Set linear acceleration (convert from mg to m/s²)
                imu.linear_acceleration.x = float(acceleration.x) / 1000.0 * 9.80665
                imu.linear_acceleration.y = float(acceleration.y) / 1000.0 * 9.80665
                imu.linear_acceleration.z = float(acceleration.z) / 1000.0 * 9.80665
                
                # Set angular velocity (convert from degrees/s to rad/s)
                imu.angular_velocity.x = float(angular_velocity.x) / 57.2957795130824
                imu.angular_velocity.y = float(angular_velocity.y) / 57.2957795130824
                imu.angular_velocity.z = float(angular_velocity.z) / 57.2957795130824
                
                # Initialize covariance matrices
                for i in range(9):
                    imu.angular_velocity_covariance[i] = 0.0
                    imu.linear_acceleration_covariance[i] = 0.0
                    imu.orientation_covariance[i] = 0.0
                
                # Set diagonal covariance values
                for i in [0, 4, 8]:
                    imu.angular_velocity_covariance[i] = 0.001
                    imu.linear_acceleration_covariance[i] = 0.001
                    imu.orientation_covariance[i] = 0.001
                
                # Publish the IMU message
                self.imu_pub.publish(imu)
                
                if self.node and self.debug_level >= 2:
                    self.node.get_logger().info(f"Published IMU data - Quat: [{quaternion.w:.3f}, {quaternion.x:.3f}, {quaternion.y:.3f}, {quaternion.z:.3f}]")
                    
            else:
                if self.node and self.debug_level >= 1:
                    self.node.get_logger().warn(f"Failed to read IMU data - Quat: {status_quat}, Accel: {status_accel}, Gyro: {status_gyro}")
                    
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error publishing IMU data: {e}")


def main(args=None):
    """
    Main entry point for the Pozyx ranging reader node.
    
    Initialize ROS 2, create the node, and start the node lifecycle.
    
    Args:
        args: Command line arguments (passed to rclpy.init)
    
    Returns:
        Exit code (0 for normal exit)
    """
    rclpy.init(args=args)
    
    node = PozyxRanger()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main() 
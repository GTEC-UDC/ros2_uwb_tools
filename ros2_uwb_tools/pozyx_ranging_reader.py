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
                     PozyxRegisters, DeviceRange, EulerAngles, Acceleration, Quaternion, AngularVelocity, UWBSettings)
                     
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
    POZYX_HARDWARE_VER = 0x2  # Hardware version
    POZYX_ST_RESULT = 0x11  # Self-test result
    POZYX_NETWORK_ID = PozyxRegisters.POZYX_NETWORK_ID
except:
    # Use known values if not available in the library
    POZYX_WHO_AM_I = 0x0 
    POZYX_FIRMWARE_VER = 0x1
    POZYX_HARDWARE_VER = 0x2
    POZYX_ST_RESULT = 0x11
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
        self.declare_parameter('debug_level', 1)  # 0 = none, 1 = normal, 2 = verbose
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
        
        # Store initial rate for potential firmware adjustments
        self.base_rate_hz = 5
        
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
            
            # Check firmware and hardware versions for compatibility
            self.checkDeviceCompatibility()
            
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
        
        # Pass firmware version to ReadyToRange if available
        if hasattr(self, '_firmware_version'):
            self.r._firmware_version = self._firmware_version
            
        self.hasAnchors = False
        
        # Create subscription for anchor position messages
        self.create_subscription(MarkerArray, '/gtec/toa/anchors', self.callbackAnchorsMessage, 10)
        
        self.get_logger().info("Waiting for anchor positions...")
        
        # Check if remote or local device will be used
        if self.targetDeviceId != 0:
            self.get_logger().info(f"Using remote device with ID: 0x{self.targetDeviceId:04x}")
        else:
            self.get_logger().info("Using local device (no remote)")

    def checkDeviceCompatibility(self):
        """
        Check device firmware and hardware versions for compatibility issues.
        
        This method reads the device's firmware and hardware versions and provides
        warnings about known compatibility issues.
        """
        try:
            # Read WHO_AM_I register to verify device
            who_am_i = SingleRegister()
            if self.pozyxSerial.getRead(POZYX_WHO_AM_I, who_am_i) == POZYX_SUCCESS:
                if who_am_i.value == 0x43:
                    self.get_logger().info("✅ Device identified as Pozyx (WHO_AM_I: 0x43)")
                else:
                    self.get_logger().warn(f"⚠️  Unexpected WHO_AM_I value: 0x{who_am_i.value:02x} (expected 0x43)")
            else:
                self.get_logger().warn("Could not read WHO_AM_I register")
            
            # Read firmware version
            firmware_ver = SingleRegister()
            if self.pozyxSerial.getRead(POZYX_FIRMWARE_VER, firmware_ver) == POZYX_SUCCESS:
                # Decode firmware version
                if firmware_ver.value == 1:
                    fw_str = "v1.0 (legacy format)"
                else:
                    major = (firmware_ver.value >> 4) & 0x0F
                    minor = firmware_ver.value & 0x0F
                    fw_str = f"v{major}.{minor}"
                
                self.get_logger().info(f"📋 Firmware version: {fw_str} (raw: 0x{firmware_ver.value:02x})")
                
                # Store firmware version for later use
                self._firmware_version = firmware_ver.value
                
                # Check for known firmware compatibility issues
                self.checkFirmwareCompatibility(firmware_ver.value)
                
            else:
                self.get_logger().warn("Could not read firmware version")
            
            # Read hardware version
            hardware_ver = SingleRegister()
            if self.pozyxSerial.getRead(POZYX_HARDWARE_VER, hardware_ver) == POZYX_SUCCESS:
                # Decode hardware version
                hw_type = (hardware_ver.value >> 6) & 0x03
                hw_version = (hardware_ver.value >> 4) & 0x03
                
                type_str = "Anchor" if hw_type == 0 else "Arduino Shield" if hw_type == 1 else f"Unknown ({hw_type})"
                version_str = "1.2" if hw_version == 2 else "1.3" if hw_version == 3 else f"Unknown ({hw_version})"
                
                self.get_logger().info(f"🔧 Hardware: {type_str} v{version_str} (raw: 0x{hardware_ver.value:02x})")
                
                # Warn about device type compatibility
                if hw_type == 0 and self.targetDeviceId == 0:
                    self.get_logger().warn("⚠️  Hardware shows 'Anchor' but being used as local tag - this may cause issues")
                
            else:
                self.get_logger().warn("Could not read hardware version")
            
            # Read self-test results
            self.checkSelfTest()
            
        except Exception as e:
            self.get_logger().error(f"Error checking device compatibility: {e}")

    def checkFirmwareCompatibility(self, firmware_raw):
        """
        Check for known firmware compatibility issues.
        
        Args:
            firmware_raw: Raw firmware version value from register
        """
        # Known firmware versions and their characteristics
        known_versions = {
            0x10: {"version": "v1.0", "issues": ["Legacy format", "Limited API support"]},
            0x11: {"version": "v1.1", "issues": ["Some UWB settings not available"]},
            0x12: {"version": "v1.2", "issues": ["Improved but some pypozyx incompatibilities"]},
            0x13: {"version": "v1.3", "issues": ["Better compatibility"]},
            0x14: {"version": "v1.4", "issues": ["Good compatibility"]},
            0x15: {"version": "v1.5", "issues": ["Latest features"]},
            0x20: {"version": "v2.0", "issues": ["New major version - generally stable"]},
            0x21: {"version": "v2.1", "issues": ["Improved stability"]},
            0x22: {"version": "v2.2", "issues": ["Latest stable"]},
            0x23: {"version": "v2.3", "issues": ["Latest firmware - very stable"]},
        }
        
        if firmware_raw in known_versions:
            fw_info = known_versions[firmware_raw]
            if fw_info["issues"]:
                self.get_logger().warn(f"⚠️  Known issues with {fw_info['version']}:")
                for issue in fw_info["issues"]:
                    self.get_logger().warn(f"   • {issue}")
        else:
            self.get_logger().warn(f"⚠️  Unknown firmware version 0x{firmware_raw:02x} - compatibility uncertain")
        
        # Recommendations based on firmware version
        if firmware_raw < 0x12:
            self.get_logger().warn("🔄 RECOMMENDATION: Update firmware to v1.2 or later for better compatibility")
            # Automatically reduce ranging rate for older firmware
            if hasattr(self, 'base_rate_hz'):
                new_rate = max(1, self.base_rate_hz // 2)  # Reduce rate by half, minimum 1 Hz
                self.rate = self.create_rate(new_rate)
                self.get_logger().info(f"🐌 Automatically reduced ranging rate to {new_rate} Hz for firmware compatibility")
        elif firmware_raw == 0x10:
            self.get_logger().warn("🔄 CRITICAL: v1.0 firmware has many limitations - update strongly recommended")
            # Even slower rate for v1.0
            if hasattr(self, 'base_rate_hz'):
                new_rate = 1  # Very slow for v1.0
                self.rate = self.create_rate(new_rate)
                self.get_logger().info(f"🐌 Set very slow ranging rate ({new_rate} Hz) for v1.0 firmware compatibility")

    def checkSelfTest(self):
        """
        Check device self-test results for hardware issues.
        """
        try:
            selftest = SingleRegister()
            if self.pozyxSerial.getRead(POZYX_ST_RESULT, selftest) == POZYX_SUCCESS:
                # Decode self-test bits
                acc_ok = (selftest.value & 0x01) != 0
                magn_ok = (selftest.value & 0x02) != 0
                gyro_ok = (selftest.value & 0x04) != 0
                imu_ok = (selftest.value & 0x08) != 0
                press_ok = (selftest.value & 0x10) != 0
                uwb_ok = (selftest.value & 0x20) != 0
                
                self.get_logger().info(f"🧪 Self-test results (0x{selftest.value:02x}):")
                self.get_logger().info(f"   • UWB: {'✅' if uwb_ok else '❌'} {'PASS' if uwb_ok else 'FAIL'}")
                self.get_logger().info(f"   • Accelerometer: {'✅' if acc_ok else '❌'} {'PASS' if acc_ok else 'FAIL'}")
                self.get_logger().info(f"   • Magnetometer: {'✅' if magn_ok else '❌'} {'PASS' if magn_ok else 'FAIL'}")
                self.get_logger().info(f"   • Gyroscope: {'✅' if gyro_ok else '❌'} {'PASS' if gyro_ok else 'FAIL'}")
                self.get_logger().info(f"   • IMU: {'✅' if imu_ok else '❌'} {'PASS' if imu_ok else 'FAIL'}")
                self.get_logger().info(f"   • Pressure: {'✅' if press_ok else '❌'} {'PASS' if press_ok else 'FAIL'}")
                
                # Critical warnings
                if not uwb_ok:
                    self.get_logger().error("❌ UWB self-test FAILED - ranging will not work!")
                    self._uwb_selftest_failed = True  # Store for later diagnostics
                    
                    # Automatically reduce ranging rate for unstable hardware
                    if hasattr(self, 'base_rate_hz'):
                        new_rate = 1  # Very slow rate for faulty hardware
                        self.rate = self.create_rate(new_rate)
                        self.get_logger().warn(f"🐌 Automatically reduced ranging rate to {new_rate} Hz due to UWB hardware issues")
                        self.get_logger().warn("     This may improve stability but expect intermittent failures")
                        
                if not acc_ok and self.publish_imu:
                    self.get_logger().warn("⚠️  Accelerometer self-test failed - IMU data may be unreliable")
                if not (magn_ok or gyro_ok or imu_ok) and self.publish_imu:
                    self.get_logger().warn("⚠️  IMU sensors failed self-test - disabling IMU publishing")
                    self.publish_imu = False
                    
            else:
                self.get_logger().warn("Could not read self-test results")
                
        except Exception as e:
            self.get_logger().warn(f"Error reading self-test results: {e}")

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
                
                # Use the correct method to read all UWB settings at once
                try:
                    uwb_settings = UWBSettings()
                    if self.pozyxSerial.getUWBSettings(uwb_settings) == POZYX_SUCCESS:
                        # Get channel
                        self.get_logger().info(f"UWB Channel: {uwb_settings.channel}")
                        
                        # Get bitrate and provide human-readable description
                        bitrates = {0: "110 kbps", 1: "850 kbps", 2: "6.8 Mbps"}
                        bitrate_str = bitrates.get(uwb_settings.bitrate, f"Unknown ({uwb_settings.bitrate})")
                        self.get_logger().info(f"UWB Bitrate: {bitrate_str}")
                        
                        # Get PRF (Pulse Repetition Frequency) and provide human-readable description
                        prf_str = "16 MHz" if uwb_settings.prf == 1 else "64 MHz" if uwb_settings.prf == 2 else f"Unknown ({uwb_settings.prf})"
                        self.get_logger().info(f"UWB PRF: {prf_str}")
                        
                        # Get preamble length
                        self.get_logger().info(f"UWB Preamble Length: {uwb_settings.plen}")
                        
                        # Get gain if available
                        try:
                            self.get_logger().info(f"UWB Gain: {uwb_settings.gain_db} dB")
                        except AttributeError:
                            # Some versions might not have gain_db attribute
                            pass
                            
                    else:
                        self.get_logger().warn("Could not retrieve UWB settings")
                except Exception as e:
                    self.get_logger().warn(f"Error getting UWB settings: {e}")
                    
                # Try to get individual channel setting as fallback if getUWBSettings fails
                try:
                    uwb_channel = SingleRegister()
                    if self.pozyxSerial.getUWBChannel(uwb_channel) == POZYX_SUCCESS:
                        self.get_logger().info(f"UWB Channel (fallback): {uwb_channel.value}")
                    else:
                        self.get_logger().warn("Could not retrieve UWB channel")
                except Exception as e:
                    self.get_logger().warn(f"Error getting UWB channel: {e}")
                    
            except Exception as e:
                self.get_logger().warn(f"Error checking UWB settings: {e}")
        
        # Start continuous ranging loop directly (no background thread)
        self.get_logger().info("Starting continuous ranging measurements...")
        
        # Check if remote device is reachable before starting ranging
        if self.targetDeviceId != 0:
            self.get_logger().info(f"Checking communication with remote device 0x{self.targetDeviceId:04x}...")
            try:
                # Try to read network ID from remote device
                remote_network_id = SingleRegister()
                status = self.pozyxSerial.getRead(POZYX_NETWORK_ID, remote_network_id, self.targetDeviceId)
                if status == POZYX_SUCCESS:
                    self.get_logger().info(f"✅ Remote device 0x{self.targetDeviceId:04x} is reachable (Network ID: 0x{remote_network_id.value:04x})")
                else:
                    self.get_logger().error(f"❌ Cannot communicate with remote device 0x{self.targetDeviceId:04x}")
                    self.get_logger().error("   This could be due to:")
                    self.get_logger().error("   • UWB hardware failure on USB device (self-test failed)")
                    self.get_logger().error("   • Remote device is not powered on")
                    self.get_logger().error("   • Remote device is out of range")
                    self.get_logger().error("   • Network ID mismatch")
                    self.get_logger().error("   • UWB channel/settings mismatch")
                    return
            except Exception as e:
                self.get_logger().error(f"❌ Exception checking remote device: {e}")
                return
        
        last_error_time = 0.0
        error_count = 0
        import time
        
        self.get_logger().info("🔄 Entering main ranging loop...")
        loop_iteration = 0
        
        while rclpy.ok():
            loop_iteration += 1
            self.get_logger().info(f"📍 Loop iteration {loop_iteration} - About to call r.loop()...")
            
            try:
                self.r.loop()
                self.get_logger().info(f"✅ Loop iteration {loop_iteration} completed successfully")
                
                # Calculate sleep time in seconds
                sleep_time_seconds = 1.0 / self.base_rate_hz
                self.get_logger().info(f"💤 Sleeping for {sleep_time_seconds*1000:.0f}ms before next iteration...")
                
                # Use time.sleep instead of rate.sleep to avoid ROS2 timer issues
                time.sleep(sleep_time_seconds)
                
                self.get_logger().info(f"⏰ Sleep completed, checking rclpy.ok()...")
                
                if not rclpy.ok():
                    self.get_logger().info("🛑 rclpy.ok() returned False, exiting loop")
                    break
                else:
                    self.get_logger().info(f"✅ rclpy.ok() is True, continuing to iteration {loop_iteration + 1}")
                
                # Reset error counter if no problems occurred
                error_count = 0
            except Exception as e:
                current_time = time.time()
                error_count += 1
                
                # Limit error logging to avoid filling up the logs
                if current_time - last_error_time > 5.0 or error_count <= 3:
                    self.get_logger().error(f"❌ Error in ranging loop iteration {loop_iteration}: {e}")
                    last_error_time = current_time
                
                # Try to recover from the error
                try:
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
        
        # Perform anchor connectivity check
        if self.node and self.debug_level >= 1:
            self.checkAnchorConnectivity()
        
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
        if self.node and self.debug_level >= 1:
            self.node.get_logger().info("🔍 ReadyToRange.loop() called - starting ranging cycle")
        
        self.seq += 1
        if self.seq > 255:
            self.seq = 0
        current_seq = self.seq
        
        if self.node and self.debug_level >= 2:
            self.node.get_logger().info(f"Starting ranging cycle {current_seq} with {len(self.anchors)} anchors")
        
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
        for i, anchor in enumerate(self.anchors):
            if self.node and self.debug_level >= 2:
                self.node.get_logger().info(f"Attempting ranging with anchor {i+1}/{len(self.anchors)}: 0x{anchor.network_id:04x}")
            
            range_data = DeviceRange()
            try:
                self.ranging_count += 1
                
                # Add firmware-specific delays and configurations
                if hasattr(self, '_firmware_version') and self._firmware_version <= 0x11:
                    # Older firmware needs more time between operations
                    import time
                    time.sleep(0.05)  # 50ms delay for older firmware
                
                if self.node and self.debug_level >= 2:
                    self.node.get_logger().info(f"Calling doRanging(0x{anchor.network_id:04x}, range_data, 0x{self.remote_id:04x if self.remote_id else 0:04x})")
                
                # Add timeout protection for doRanging to prevent indefinite blocking
                import signal
                import time
                
                def timeout_handler(signum, frame):
                    raise TimeoutError("doRanging call timed out")
                
                try:
                    # Set a 5-second timeout for ranging operation
                    signal.signal(signal.SIGALRM, timeout_handler)
                    signal.alarm(5)  # 5 second timeout
                    
                    start_time = time.time()
                    status = self.pozyx.doRanging(anchor.network_id, range_data, self.remote_id)
                    end_time = time.time()
                    
                    # Clear the timeout
                    signal.alarm(0)
                    
                    if self.node and self.debug_level >= 2:
                        self.node.get_logger().info(f"doRanging completed in {(end_time - start_time)*1000:.1f}ms, status: {status}")
                        
                except TimeoutError:
                    signal.alarm(0)  # Clear timeout
                    if self.node:
                        self.node.get_logger().error(f"⏰ doRanging with anchor 0x{anchor.network_id:04x} timed out after 5 seconds!")
                        self.node.get_logger().error("   This suggests the UWB hardware is not responding properly")
                    continue  # Skip to next anchor
                except Exception as ranging_exception:
                    signal.alarm(0)  # Clear timeout
                    raise ranging_exception  # Re-raise the original exception
                
                if self.node and self.debug_level >= 2:
                    self.node.get_logger().info(f"doRanging returned status: {status}")
                
                if status == POZYX_SUCCESS:
                    if range_data.RSS < 0.0:  # Valid RSS values are negative
                        self.ranging_success_count += 1
                        self.publishRanging(anchor.network_id, range_data, current_seq)
                        if self.node and self.debug_level >= 1:
                            self.node.get_logger().info(f"✅ Range to anchor 0x{anchor.network_id:04x}: {range_data.distance} mm, RSS: {range_data.RSS} dB")
                    else:
                        if self.node and self.debug_level >= 1:
                            self.node.get_logger().warn(f"❌ Invalid RSS value for anchor 0x{anchor.network_id:04x}: {range_data.RSS} dB")
                else:
                    if self.node:
                        try:
                            error_code = SingleRegister()
                            self.pozyx.getErrorCode(error_code)
                            
                            # Error code mapping for concise logging
                            error_names = {
                                0x01: "POZYX_FAILURE",
                                0x02: "POZYX_TIMEOUT", 
                                0x03: "POZYX_INVALID",
                                0x04: "POZYX_ANCHOR_NOT_FOUND",
                                0x05: "POZYX_UWB_BUSY",
                                0x06: "POZYX_FUNCTION_NOT_AVAILABLE",
                                0x08: "POZYX_NO_FLASH",
                                0x0A: "POZYX_NOT_ENOUGH_ANCHORS",
                                0x0B: "POZYX_DISCOVERY",
                                0x0E: "POZYX_ANCHOR_NOT_FOUND",
                                0x11: "POZYX_RANGING",
                                0x12: "POZYX_RTIMEOUT1",
                                0x13: "POZYX_RTIMEOUT2", 
                                0x14: "POZYX_TXLATE",
                                0x15: "POZYX_UWB_BUSY",
                                0x16: "POZYX_POSALG",
                                0x17: "POZYX_NOACK",
                                0xFF: "POZYX_GENERAL"
                            }
                            
                            error_name = error_names.get(error_code.value, f"UNKNOWN_0x{error_code.value:02x}")
                            self.node.get_logger().warn(f"❌ Anchor 0x{anchor.network_id:04x} failed: {error_name} (0x{error_code.value:02x})")
                            
                        except Exception as e:
                            self.node.get_logger().warn(f"❌ Anchor 0x{anchor.network_id:04x}: Could not get error code - {e}")
                        
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"❌ Exception in ranging with anchor 0x{anchor.network_id:04x}: {e}")
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
                    if self.node:
                        self.node.get_logger().info("Reconnected to Pozyx device")
                except Exception as reconnect_error:
                    if self.node:
                        self.node.get_logger().error(f"Error reconnecting to Pozyx: {reconnect_error}")
        
        if self.node and self.debug_level >= 2:
            self.node.get_logger().info(f"Completed ranging cycle {current_seq}")

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
                    error_msgs = []
                    if status_quat != POZYX_SUCCESS:
                        error_msgs.append(f"Quaternion: {status_quat}")
                    if status_accel != POZYX_SUCCESS:
                        error_msgs.append(f"Acceleration: {status_accel}")
                    if status_gyro != POZYX_SUCCESS:
                        error_msgs.append(f"Gyro: {status_gyro}")
                    
                    self.node.get_logger().warn(f"Failed to read IMU data - {', '.join(error_msgs)}")
                    
                    # Check if this might be a device compatibility issue
                    if all(status == 0 for status in [status_quat, status_accel, status_gyro]):
                        self.node.get_logger().warn("  → All IMU functions returned 0 - device might not support IMU or wrong device type (anchor vs tag)")
                        # Disable IMU publishing if it's consistently failing
                        if hasattr(self, '_imu_failure_count'):
                            self._imu_failure_count += 1
                            if self._imu_failure_count > 10:
                                self.node.get_logger().warn("  → Disabling IMU publishing due to repeated failures")
                                self.imu_pub = None
                        else:
                            self._imu_failure_count = 1
                    
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error publishing IMU data: {e}")
                if self.debug_level >= 1:
                    self.node.get_logger().error("  → This might indicate the device doesn't support IMU functionality")

    def checkAnchorConnectivity(self):
        """
        Perform a basic connectivity check with configured anchors.
        
        This method attempts to perform ranging with each anchor to verify
        that they are reachable and responding properly.
        """
        if not self.node:
            return
            
        if self.remote_id and self.remote_id != 0:
            self.node.get_logger().info(f"Performing anchor connectivity check...")
            # Get USB device network ID safely
            try:
                usb_network_id = SingleRegister()
                if self.pozyx.getRead(POZYX_NETWORK_ID, usb_network_id) == POZYX_SUCCESS:
                    self.node.get_logger().info(f"📡 Testing communication: USB anchor (0x{usb_network_id.value:04x}) → Remote tag (0x{self.remote_id:04x}) → Target anchors")
                else:
                    self.node.get_logger().info(f"📡 Testing communication: USB anchor → Remote tag (0x{self.remote_id:04x}) → Target anchors")
            except:
                self.node.get_logger().info(f"📡 Testing communication: USB anchor → Remote tag (0x{self.remote_id:04x}) → Target anchors")
        else:
            self.node.get_logger().info(f"Performing anchor connectivity check...")
            self.node.get_logger().info(f"📡 Testing direct ranging from local device to anchors")
        
        reachable_anchors = 0
        total_anchors = len(self.anchors)
        
        for anchor in self.anchors:
            try:
                range_data = DeviceRange()
                status = self.pozyx.doRanging(anchor.network_id, range_data, self.remote_id)
                
                if status == POZYX_SUCCESS:
                    if range_data.RSS < 0.0:  # Valid RSS values are negative
                        reachable_anchors += 1
                        if self.remote_id and self.remote_id != 0:
                            self.node.get_logger().info(f"✓ Anchor 0x{anchor.network_id:04x}: Range={range_data.distance}mm, RSS={range_data.RSS}dB (via remote tag 0x{self.remote_id:04x})")
                        else:
                            self.node.get_logger().info(f"✓ Anchor 0x{anchor.network_id:04x}: Range={range_data.distance}mm, RSS={range_data.RSS}dB (direct)")
                    else:
                        self.node.get_logger().warn(f"✗ Anchor 0x{anchor.network_id:04x}: Invalid RSS value ({range_data.RSS}dB)")
                else:
                    error_code = SingleRegister()
                    self.pozyx.getErrorCode(error_code)
                    self.node.get_logger().warn(f"✗ Anchor 0x{anchor.network_id:04x}: Error 0x{error_code.value:02x}")
                    
            except Exception as e:
                self.node.get_logger().warn(f"✗ Anchor 0x{anchor.network_id:04x}: Exception - {e}")
        
        # Provide diagnostic summary with hardware-specific advice
        if reachable_anchors == 0:
            self.node.get_logger().error("⚠️  NO ANCHORS REACHABLE!")
            self.node.get_logger().error("Troubleshooting suggestions:")
            self.node.get_logger().error("  1. Check that anchors are powered on")
            self.node.get_logger().error("  2. Verify anchor positions are published correctly")
            self.node.get_logger().error("  3. Check UWB channel settings match between tag and anchors")
            self.node.get_logger().error("  4. Ensure tag and anchors are on the same network")
            self.node.get_logger().error("  5. Check that tag is within ranging distance of anchors")
            self.node.get_logger().error("  6. FIRMWARE COMPATIBILITY:")
            self.node.get_logger().error("     • Verify tag and anchors have compatible firmware versions")
            self.node.get_logger().error("     • Update all devices to same firmware version if possible")
            self.node.get_logger().error("     • Check if anchors are using legacy firmware (v1.0)")
            self.node.get_logger().error("  7. Try reducing ranging frequency (increase sleep time)")
            
            # Add specific advice for failed UWB self-test
            if hasattr(self.node, '_uwb_selftest_failed') and self.node._uwb_selftest_failed:
                self.node.get_logger().error("  8. HARDWARE ISSUE DETECTED:")
                self.node.get_logger().error("     • USB anchor UWB self-test failed - hardware may be faulty")
                self.node.get_logger().error("     • Try using a different Pozyx device as USB bridge")
                self.node.get_logger().error("     • Contact Pozyx support for hardware replacement")
                
        elif reachable_anchors < total_anchors:
            self.node.get_logger().warn(f"⚠️  Only {reachable_anchors}/{total_anchors} anchors reachable")
            self.node.get_logger().warn("Some anchors may be out of range, powered off, or have communication issues")
            self.node.get_logger().warn("Consider checking firmware compatibility for unreachable anchors")
        else:
            self.node.get_logger().info(f"✅ All {reachable_anchors}/{total_anchors} anchors reachable")
            
            # Add warning if UWB self-test failed but connectivity works
            if hasattr(self.node, '_uwb_selftest_failed') and self.node._uwb_selftest_failed:
                self.node.get_logger().warn("⚠️  WARNING: UWB self-test failed but connectivity check passed")
                self.node.get_logger().warn("     This suggests the USB anchor UWB is partially working but may be unreliable")
                self.node.get_logger().warn("     Expect intermittent failures during continuous operation")
                self.node.get_logger().warn("     Consider replacing the USB anchor for stable operation")
            
        self.node.get_logger().info("Connectivity check complete")


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
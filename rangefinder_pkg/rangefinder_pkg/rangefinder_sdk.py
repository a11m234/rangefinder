import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range # Use standard ROS 2 message for range data
import serial
import time
import struct
import math # For math.nan

class LidarReader(Node):
    def __init__(self):
        super().__init__('rangefinder_data')
        self.publisher_ = self.create_publisher(Range, 'rangefinder/range', 10)
        self.timer = None # Timer will be set up after serial connection
        self.serial_port = None
        self.declare_parameter('serial_port', '/dev/ttyUSB0') # Default serial port
        self.declare_parameter('baud_rate', 115200) # Default baud rate
        self.declare_parameter('frame_id', 'rangefinder_link') # Frame ID for the sensor data

        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.get_logger().info(f"Attempting to open serial port: {self.serial_port_name} at {self.baud_rate} baud.")
        self.connect_serial()

    def connect_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1 # Read timeout in seconds
            )
            if self.serial_port.isOpen():
                self.serial_port.close()
                self.serial_port.open()
            self.get_logger().info(f"Successfully connected to {self.serial_port_name}")
            time.sleep(1) # Give it a moment after opening
            self.start_continuous_ranging()
            # Start timer after serial is ready
            self.timer = self.create_timer(0.05, self.read_and_publish_data) # Polls roughly 20 Hz, but lidar sends at 1Hz or faster
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {self.serial_port_name}: {e}")
            rclpy.shutdown() # Shutdown ROS node if serial connection fails

    def send_command(self, command_bytes):
        """Sends a command with checksum to the lidar."""
        checksum = sum(command_bytes[2:7]) & 0xFF # Sum bytes 3 to 7 (0-indexed 2 to 6)
        full_command = command_bytes + struct.pack('B', checksum)
        try:
            self.serial_port.write(full_command)
            # self.get_logger().info(f"Sent command: {full_command.hex()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def start_continuous_ranging(self):
        """Sends the continuous ranging command."""
        self.get_logger().info("Sending continuous ranging command...")
        # Command: 0x55 0xAA 0x89 0xFF 0xFF 0xFF 0xFF Checksum 
        command = bytes([0x55, 0xAA, 0x89, 0xFF, 0xFF, 0xFF, 0xFF])
        self.send_command(command)

    def stop_ranging(self):
        """Sends the stop ranging command."""
        self.get_logger().info("Sending stop ranging command...")
        # Command: 0x55 0xAA 0x8E 0xFF 0xFF 0xFF 0xFF Checksum 
        command = bytes([0x55, 0xAA, 0x8E, 0xFF, 0xFF, 0xFF, 0xFF])
        self.send_command(command)

    def read_and_publish_data(self):
        """Reads data from the serial port and publishes a Range message."""
        if not self.serial_port or not self.serial_port.isOpen():
            return

        try:
            # Read until we find a full message or timeout
            while self.serial_port.in_waiting >= 8: # A full message is 8 bytes 
                header_byte1 = self.serial_port.read(1)
                if header_byte1 != b'\x55': # Check first header byte 
                    continue # Keep reading until header is found

                header_byte2 = self.serial_port.read(1)
                if header_byte2 != b'\xAA': # Check second header byte 
                    continue

                command_byte = self.serial_port.read(1) # Byte 3: 0x88 for single, 0x89 for continuous 
                
                # Read remaining 5 bytes
                remaining_bytes = self.serial_port.read(5)
                if len(remaining_bytes) < 5:
                    self.get_logger().warn("Incomplete message received after header.")
                    continue

                full_message = header_byte1 + header_byte2 + command_byte + remaining_bytes
                
                # Unpack the bytes
                byte1, byte2, byte3, status, byte5, data_h, data_l, received_checksum = struct.unpack('<BBBBBBBB', full_message) [cite: 13, 19]

                # Verify command byte (0x88 or 0x89) 
                if byte3 not in [0x88, 0x89]:
                    # self.get_logger().warn(f"Unexpected command byte: {hex(byte3)}")
                    continue # Not a ranging data packet

                # Calculate checksum for received data 
                # For received data, checksum is sum of byte 1 to 7 (0-indexed 0 to 6)
                calculated_checksum = (sum(full_message[0:7])) & 0xFF

                if calculated_checksum != received_checksum:
                    # self.get_logger().warn(f"Checksum mismatch. Calculated: {hex(calculated_checksum)}, Received: {hex(received_checksum)}")
                    continue # Discard invalid packet

                if status == 0: # Status = 0 indicates measurement failure 
                    # self.get_logger().warn("Lidar reported measurement failure.")
                    continue # Don't publish bad data

                # Combine DATA_H and DATA_L to get the raw measurement 
                raw_distance = (data_h << 8) | data_l
                
                # Convert raw distance to meters (divide by 10) 
                distance_m = raw_distance / 10.0

                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.radiation_type = Range.INFRARED # Assuming it's an IR laser rangefinder
                msg.field_of_view = 0.0 # 1D lidar has no FOV in this context
                msg.min_range = 0.0 # Based on typical lidar range, adjust if known
                msg.max_range = 1500.0 # From documentation 

                if distance_m > msg.max_range or distance_m < msg.min_range:
                    msg.range = float('inf') # Indicate out of range if it exceeds max
                else:
                    msg.range = distance_m

                self.publisher_.publish(msg)
                # self.get_logger().info(f'Published range: {msg.range:.2f} m')
                break # Process one message per timer callback
        
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
            self.destroy_node()
            rclpy.shutdown()
        except struct.error as e:
            self.get_logger().error(f"Error unpacking data: {e}. Data received might be malformed.")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")

    def on_shutdown(self):
        """Called when the node is about to be shut down."""
        self.get_logger().info("Shutting down lidar_reader node.")
        if self.timer:
            self.timer.cancel()
        if self.serial_port and self.serial_port.isOpen():
            self.stop_ranging() # Send stop command before closing
            time.sleep(0.1) # Give lidar a moment to process
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
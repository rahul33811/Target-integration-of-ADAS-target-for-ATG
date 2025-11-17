import rclpy
from rclpy.node import Node
import socket
import struct

class UDPReceiverNode(Node):

    def __init__(self):
        super().__init__('udp_receiver')

        # Declare parameters instead of using hardcoded values
        self.declare_parameter('udp_ip', '0.0.0.0')   # Safe placeholder IP
        self.declare_parameter('udp_port', 0)         # Safe placeholder port

        udp_ip = self.get_parameter('udp_ip').value
        udp_port = int(self.get_parameter('udp_port').value)

        # Create UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((udp_ip, udp_port))

        # Timer to poll messages
        self.create_timer(0.1, self.receive_udp_message)

    def receive_udp_message(self):
        try:
            data, _ = self.socket.recvfrom(1024)
            self.process_udp_message(data)
        except Exception as e:
            self.get_logger().warn(f"UDP receive error: {e}")

    def process_udp_message(self, data):
        # Basic validation
        if len(data) < 8:
            self.get_logger().warn("Received incomplete UDP header")
            return

        version_number, message_number = struct.unpack('<B3xI', data[:8])
        payload = data[8:]

        # Example: process message number 0
        if message_number == 0:
            if len(payload) < 48:
                self.get_logger().warn("Received incomplete payload")
                return

            # Unpack payload (no sensitive values logged)
            steering, brake, accel, ext, trig, spare32, watchdog, gear, spare48 = \
                struct.unpack('<dddHHIBBQ', payload[:48])

            self.get_logger().info("Message 0 received and processed")
        else:
            # Additional message types can be handled here
            pass

    def destroy_node(self):
        self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UDPReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

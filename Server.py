import rclpy
from rclpy.node import Node
import socket
import struct
from atg_interfaces.msg import ActorCmd

class UDPClientNode(Node):

    def __init__(self):
        super().__init__('udp_client')

        # Parameters instead of hardcoded sensitive values
        self.declare_parameter('udp_ip', '0.0.0.0')   # Placeholder IP
        self.declare_parameter('udp_port', 0)         # Placeholder port
        self.declare_parameter('cmd_topic', 'actor_cmd')

        self.udp_ip = self.get_parameter('udp_ip').value
        self.udp_port = self.get_parameter('udp_port').value
        self.cmd_topic = self.get_parameter('cmd_topic').value

        # UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Subscription with sanitized topic
        self.subscription = self.create_subscription(
            ActorCmd,
            self.cmd_topic,
            self.actor_cmd_callback,
            10
        )

    def actor_cmd_callback(self, msg):
        # Avoid logging sensitive numerical details
        self.get_logger().info('ActorCmd message received.')

        # Pack payload (values intact, no sensitive data added)
        payload = struct.pack(
            '<dddHHIBBQ',
            msg.kappa,      # steering
            0.0,            # brake
            msg.acc,        # accelerator
            0, 0, 0,        # reserved numeric placeholders
            0, 0, 0         # reserved numeric placeholders
        )

        # Message header
        version_number = 2
        message_number = 0
        header = struct.pack('<B3xI', version_number, message_number)

        udp_message = header + payload

        # Send sanitized UDP packet
        self.socket.sendto(udp_message, (self.udp_ip, self.udp_port))

    def destroy_node(self):
        self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UDPClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

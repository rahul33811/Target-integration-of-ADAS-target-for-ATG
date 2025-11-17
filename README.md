# Target-integration-of-ADAS-target-for-ATG
A configurable ROS2 node that receives vehicle control commands and forwards them as binary UDP messages to external applications or simulators.ROS2 UDP Client–Server Nodes

This package provides a pair of lightweight, parameterized ROS2 nodes that communicate using UDP.
The Client Node sends structured binary control messages, while the Server Node receives and parses them.

Both nodes avoid any hardcoded sensitive information and use ROS2 parameters for configuration.

📦 Package Overview
1. UDP Client Node

Subscribes to a ROS2 topic containing control commands (ActorCmd)

Packs data into a binary payload using struct

Sends UDP messages to a specified IP and port

2. UDP Server Node

Listens for incoming UDP packets

Parses message headers and payloads

Handles Message Type 0 (default), with extensibility for more types

🛠️ Features

No hardcoded IPs, ports, or topic names

Parameterized configuration for safe deployment

Compatible client/server message format using a shared binary protocol

Works on local networks or distributed systems

Clean, sanitized code suitable for public repositories

⚙️ Parameters

Both nodes use ROS2 parameters instead of embedded configuration.

Client Node Parameters
Parameter	Type	Description
udp_ip	string	Destination IP address
udp_port	int	Destination UDP port
cmd_topic	string	Topic to subscribe to (ActorCmd)
Server Node Parameters
Parameter	Type	Description
udp_ip	string	IP to bind the server socket to
udp_port	int	Port to bind the server socket
📡 Message Format

Each UDP message includes:

Header (<B3xI>)

Version Number (1 byte)

3-byte padding

Message Number (unsigned 32-bit)

Payload for Message Number 0 (<dddHHIBBQ>)

Steering value

Brake value

Accelerator value

External comms flag

Trigger flag

Spare (32-bit)

Watchdog counter

Gear command

Spare (64-bit)

▶️ Running the Nodes
Start the Server
ros2 run <your_package> server \
  --ros-args -p udp_ip:=0.0.0.0 -p udp_port:=5000

Start the Client
ros2 run <your_package> client \
  --ros-args -p udp_ip:=127.0.0.1 -p udp_port:=5000 -p cmd_topic:=actor_cmd


Make sure both nodes use the same port so they can communicate.

📁 Project Structure
your_package/
├── server.py     # UDP receiver node
├── client.py     # UDP sender node
├── package.xml
├── setup.py
└── README.md

🧩 Integration with ROS2
Publishing to the client:
ros2 topic pub /actor_cmd atg_interfaces/msg/ActorCmd "{acc: 1.0, kappa: 0.5}"


The client will send the packed data to the server through UDP.

🧪 Testing on Localhost

Start server on port 5000:

ros2 run <your_package> server --ros-args -p udp_ip:=0.0.0.0 -p udp_port:=5000


Start client sending to localhost:

ros2 run <your_package> client --ros-args -p udp_ip:=127.0.0.1 -p udp_port:=5000

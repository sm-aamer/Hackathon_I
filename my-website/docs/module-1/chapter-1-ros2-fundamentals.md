---
id: chapter-1-ros2-fundamentals
title: Chapter 1 - ROS 2 Fundamentals
sidebar_label: ROS 2 Fundamentals
---

# Chapter 1: ROS 2 Fundamentals

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. ROS 2 is designed to be suitable for real-world robotics applications, including those that require high performance, safety, and security.

## Nodes

In ROS 2, processes that are to be run are known as nodes. A node is typically written to perform a specific task such as controlling a sensor, processing data, or controlling a robot's movement. Nodes are combined together to form the complete robot application. Each node is a lightweight process that performs computation.

### Creating a Node

To create a node in ROS 2, you typically:
1. Initialize the ROS 2 client library
2. Create a node instance
3. Add functionality to the node (publishers, subscribers, services, etc.)
4. Spin the node to process callbacks
5. Clean up when finished

## Topics and Pub/Sub Communication

Topics are named buses over which nodes exchange messages. The communication is based on the publish/subscribe pattern where publishers send messages and subscribers receive them asynchronously.

### Publisher-Subscriber Pattern

The publisher-subscriber pattern allows for asynchronous message passing between nodes. Publishers send messages to a topic without knowledge of what (if any) subscribers there are. Subscribers express interest in one or more topics and only receive messages that are published to those topics.

### Example: Creating a Publisher and Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_publisher)
    rclpy.spin(minimal_subscriber)

    minimal_publisher.destroy_node()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services

Services provide a request/response communication pattern. A service client sends a request message and waits for a response message from a service server. This is synchronous communication that blocks the client until the server responds.

### Creating a Service

A service consists of:
1. A service definition (`.srv` file)
2. A service server node
3. A service client node

## Actions

Actions are used for long-running tasks that require feedback and the ability to cancel before completion. They combine the features of services and topics to provide:
- Request/response like services
- Continuous feedback like topics
- Ability to cancel operations

## Exercises

1. Create a simple publisher node that publishes a counter value every second
2. Create a subscriber node that receives and logs the counter values
3. Implement a service that performs a simple calculation when requested
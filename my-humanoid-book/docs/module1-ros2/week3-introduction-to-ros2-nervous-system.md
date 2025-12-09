---
id: week3-introduction-to-ros2-nervous-system
title: 'Week 3: The ROS2 Nervous System'
---

In the previous weeks, we have learned about the basics of ROS2 and how to set up a workspace. This week, we will dive deeper into the core of ROS2 and learn how to create a robotic nervous system.

## Nodes, Topics, Services, and Actions

As we learned in week 1, the core concepts of ROS2 are nodes, topics, services, and actions. This week, we will learn how to use them in practice.

### Nodes

A node is the smallest executable unit in ROS2. It is a program that performs a specific task. For example, we can have a node that reads data from a sensor, a node that controls a motor, and a node that plans a path for the robot.

### Topics

Topics are the primary way of communication between nodes. A node can publish messages to a topic, and any node that is subscribed to that topic will receive the messages. This is a one-to-many communication model.

### Services

Services are used for request-response communication between nodes. A node can provide a service, and another node can call that service. The service will then perform some task and return a response. This is a one-to-one communication model.

### Actions

Actions are used for long-running tasks. They are similar to services, but they provide feedback while the task is running. For example, an action can be used to command a robot to move to a certain location. The robot will provide feedback on its progress while it is moving.

In the labs for this week, you will create nodes, topics, services, and actions to build a simple robotic nervous system.
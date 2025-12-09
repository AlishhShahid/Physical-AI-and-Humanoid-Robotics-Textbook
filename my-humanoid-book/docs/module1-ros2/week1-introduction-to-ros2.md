---
id: week1-introduction-to-ros2
title: 'Week 1: Introduction to ROS2'
---

Welcome to the first week of our journey into humanoid robotics! This week, we will start with the very basics of the Robot Operating System 2 (ROS2), which will serve as the nervous system for our robot.

## What is ROS2?

ROS2 is a set of software libraries and tools that help you build robot applications. It is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

## Core Concepts

- **Nodes:** A node is an executable that uses ROS2 to communicate with other nodes.
- **Topics:** Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
- **Services:** A service is another method of communication for nodes. Services are based on a call-and-response model, versus topics' publisher/subscriber model.
- **Actions:** Actions are for long-running tasks. They provide feedback while the task is running.

## Your First ROS2 Node

In the labs for this week, you will create your first ROS2 node and learn how to make nodes communicate with each other.

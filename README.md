# MPU6050-Based Motor Control with Encoder Feedback

## Description
This project uses an **MPU6050 module** to control the rotation of an **N20 micro DC geared motor** through an **L293D motor driver**. The MPU6050 provides rotation angle data, and the motor's **encoder feedback** ensures the motor rotates to the desired position with **minimum error**.

Real-time angle values from the **MPU6050** and encoder feedback are displayed on an **LCD screen** and the **Serial Monitor** for monitoring and comparison.

For example:
- If the **MPU6050** is rotated by **50 degrees**, the motor will follow and adjust its position to match the desired angle accurately.

## Features
- MPU6050 for rotation angle input.
- Encoder feedback to minimize error and ensure accurate positioning.
- Real-time data displayed on:
  - LCD screen.
  - Serial Monitor.

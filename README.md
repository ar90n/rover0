# rover0
My first plain rover

## CI Status

[![ROS2 Build and Test](https://github.com/ar90n/rover0/actions/workflows/ros2-build.yml/badge.svg)](https://github.com/ar90n/rover0/actions/workflows/ros2-build.yml)
[![Lint](https://github.com/ar90n/rover0/actions/workflows/lint.yml/badge.svg)](https://github.com/ar90n/rover0/actions/workflows/lint.yml)
[![Docker Build](https://github.com/ar90n/rover0/actions/workflows/docker-build.yml/badge.svg)](https://github.com/ar90n/rover0/actions/workflows/docker-build.yml)
[![Documentation](https://github.com/ar90n/rover0/actions/workflows/docs.yml/badge.svg)](https://github.com/ar90n/rover0/actions/workflows/docs.yml)
[![Security Scan](https://github.com/ar90n/rover0/actions/workflows/security.yml/badge.svg)](https://github.com/ar90n/rover0/actions/workflows/security.yml)

## Overview

This is a ROS2-based rover robot project with multiple modules for controlling motors, processing lidar data, and teleoperation.

## Development

### Prerequisites

- Docker and Docker Compose
- ROS2 Jazzy (for local development)

### Setup

1. Clone the repository with submodules:
   ```bash
   git clone --recursive https://github.com/ar90n/rover0.git
   ```

2. Copy the environment template and configure it:
   ```bash
   cp .env.template .env
   # Edit .env file with your configuration
   ```

3. Build and run the containers:
   ```bash
   docker-compose up -d
   ```

## Modules

- **rover0**: Main rover control module
- **teleop**: Teleoperation module
- **lidar**: Lidar processing module
- **motor**: Motor control module
- **uros-agent-lidar**: Micro-ROS agent for lidar

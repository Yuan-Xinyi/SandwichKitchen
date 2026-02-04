# Sandwich Kitchen - Robotic Sandwich Preparation System

A comprehensive robotics framework for autonomous sandwich preparation using the XArm Lite6 robot. This project combines robot simulation, motion planning, grasping, vision, and real-time control to enable a robot to prepare various types of sandwiches based on orders.

## Project Overview

The Sandwich Kitchen project demonstrates an integrated robotic system that:
- Receives sandwich orders via socket communication
- Plans and executes manipulation tasks
- Controls a real XArm robot with a gripper
- Handles various food items and assembly sequences
- Provides simulation and visualization capabilities

## Project Structure

```
sandwich_kitchen/
├── experiments/              # Experimental code and demonstrations
│   └── robot/
│       └── xarm/
│           ├── collect_one_demo.py
│           ├── data_recorder.py
│           ├── demo_collection_hdf5.py
│           └── sandwich_kitchen/        # Main sandwich robot control
│               ├── sandwich_order_gui.py
│               ├── sandwich_robot_control.py
│               └── food_items.json
│
├── mcp_control/             # MCP (Model Context Protocol) server for robot control
│   ├── mcp_server_robot.py
│   └── mcp.json
│
└── wrs/                     # Core robotics framework (WRS)
```

## Key Features

### Robot Control
- **XArm Lite6 Integration**: Direct control of XArm Lite6 robot with gripper
- **Real-time Communication**: Socket-based communication for order receiving
- **Gripper Control**: Integrated end-effector control for grasping

## Getting Started

### Prerequisites
- Python 3.9+
- XArm robot and controller (for real robot operation)

### Running the Sandwich Making Script

```bash
python experiments/robot/xarm/sandwich_kitchen/sandwich_robot_control.py (run first)
python experiments/robot/xarm/sandwich_kitchen/sandwich_order_gui.py
```

The robot will:
1. Initialize and wait for connection
2. Listen for sandwich orders on port 5555
3. Execute the assembly sequence based on the order
4. Support multiple sandwich recipes (Basic, Double, Veggie, Meat, Gluten-free)

### Supported Sandwiches

Defined in `food_items.json`:
- **Basic Sandwich**: Bread → Lettuce → Ham → Bread
- **Double Sandwich**: Double meat and lettuce layers
- **Veggie Sandwich**: Vegetable-only options
- **Meat Sandwich**: Multiple meat layers
- **Gluten-free Sandwich**: Special dietary option

## Architecture

### Control Flow
```
Order Reception → Recipe Selection → Motion Planning → 
Trajectory Execution → Item Pickup → Assembly → Completion
```

### Core Components
## Configuration

### Robot Configuration
- Located in `wrs/robot_con/xarm_lite6/`
- Define joint limits, workspace bounds, and tool parameters

### Food Items
- Located in `experiments/robot/xarm/sandwich_kitchen/food_items.json`
- Specify food item properties and placement coordinates

### Network Configuration
- Host: `0.0.0.0` (listen on all interfaces)
- Port: `5555` (configurable)

---

**Last Updated**: February 2026
**Framework**: WRS
**Primary Robot**: XArm Lite6
**Application**: Autonomous Sandwich Preparation

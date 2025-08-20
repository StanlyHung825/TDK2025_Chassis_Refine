# TDK2025 Chassis Refine

An advanced chassis control system for the TDK2025 robot platform, featuring micro-ROS integration, mecanum wheel kinematics, and real-time motor control with FreeRTOS.

## 🏗️ Project Architecture

### Core Components

```
Core/
├── Inc/
│   ├── Control_Lib/          # Control system libraries
│   │   ├── chassis_ctrl/     # High-level chassis motion control
│   │   ├── motor_ctrl/       # Low-level motor drivers and monitoring
│   │   └── servo_ctrl/       # Servo control utilities
│   └── uros/                 # micro-ROS communication layer
└── Src/
    └── main_program/         # Application entry point and RTOS tasks
```

### 🎯 Control Library Structure

#### **Chassis Control** (`Control_Lib/chassis_ctrl/`)
**Purpose:** High-level motion planning and mecanum wheel coordination
- **Key Files:**
  - `chassis.hpp` - Main chassis control class with mecanum kinematics
  - `chassis_config.h` - Chassis physical parameters and initial positions
- **Functionality:**
  - Forward/inverse kinematics for mecanum wheels
  - Coordinate transformations (x, y, θ)
  - Velocity setpoint distribution to individual motors
  - Real-time position tracking

#### **Motor Control** (`Control_Lib/motor_ctrl/`)
**Purpose:** Low-level motor driver interface and feedback control
- **Key Files:**
  - `motor_ctrl.hpp` - PID-controlled motor driver class
  - `motor_monitor.hpp` - Real-time motor status monitoring
  - `motor_config.h` - Motor hardware configuration and PID parameters
- **Functionality:**
  - Encoder-based speed feedback control
  - PWM generation and direction control
  - Individual motor PID tuning
  - Real-time performance monitoring

#### **Servo Control** (`Control_Lib/servo_ctrl/`)
**Purpose:** Auxiliary servo mechanisms (future expansion)

### 🌐 Communication Layer (`uros/`)

The project integrates **micro-ROS** for seamless ROS2 ecosystem communication:

#### **Configuration Files:**
- `config.h` - Node name, domain ID, and communication parameters
- `uros_init.h/.cpp` - micro-ROS initialization and message handling

#### **ROS Topics:**

| Topic Name | Message Type | Direction | Purpose |
|------------|-------------|-----------|---------|
| `robot/cmd_vel` | `geometry_msgs/Twist` | **Subscriber** | Receives velocity commands |
| `robot/pose` | `geometry_msgs/Twist` | **Publisher** | Publishes current pose feedback |

### 🛠️ Navigation Support (`main_program/`)

#### **Line Tracing** (`trace.hpp`)
**Purpose:** Vision-based navigation assistance
- Black line detection and following
- Positioning correction algorithms
- Court position validation (planned)

---

## ⚙️ Configuration Guide

### 🔧 Changing ROS Topic Names

**Location:** `Core/Inc/uros/uros_init.cpp`

```cpp
// Publisher topic configuration (line ~138)
rclc_publisher_init_default(
  &pose_pub,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  "robot/pose");  // ← Change topic name here

// Subscriber topic configuration (line ~150)
rclc_subscription_init_default(
  &cmd_vel_sub,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  "robot/cmd_vel");  // ← Change topic name here
```

### 📨 Changing Message Types

**Location:** `Core/Inc/uros/uros_init.h` and `uros_init.cpp`

1. **Add new message include** in `uros_init.h`:
```cpp
#include <your_msgs/msg/your_message_type.h>
```

2. **Update message type support** in `uros_init.cpp`:
```cpp
ROSIDL_GET_MSG_TYPE_SUPPORT(your_msgs, msg, YourMessageType)
```

3. **Update global message variables** in `uros_init.cpp`:
```cpp
your_msgs__msg__YourMessageType your_msg;
```

### 🎛️ Motor and Chassis Parameters

#### Motor Configuration
**Location:** `Core/Inc/Control_Lib/motor_ctrl/Inc/motor_config.h`
- PID gains (Kp, Ki, Kd)
- Encoder resolution
- Motor physical specifications

#### Chassis Configuration  
**Location:** `Core/Inc/Control_Lib/chassis_ctrl/Inc/chassis_config.h`
- Wheel dimensions and spacing
- Initial position coordinates
- Kinematic parameters

### 📡 Communication Settings
**Location:** `Core/Inc/uros/config.h`
```cpp
#define NODE_NAME "cubeMX_node"    // ROS node identifier
#define DOMAIN_ID 0                // ROS domain ID
#define FREQUENCY 20               // Main loop frequency (Hz)
#define USARTx huart2             // UART interface for micro-ROS
```

---

## 🚀 Quick Start

### Prerequisites
- STM32CubeIDE
- micro-ROS Agent running on host system
- ROS2 environment for topic interaction

### Build and Deploy
1. Import project into STM32CubeIDE
2. Build and flash to STM32F446RE
3. Start micro-ROS agent: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSBx`
4. Send velocity commands: `ros2 topic pub /robot/cmd_vel geometry_msgs/Twist "..."`

### Development Status
- ✅ Motor control with PID feedback
- ✅ Mecanum wheel kinematics  
- ✅ micro-ROS integration
- ✅ Real-time monitoring
- 🔄 Line tracing navigation
- ⏳ Advanced positioning algorithms

---

*For detailed API documentation and examples, refer to the header files in the respective control library directories.*

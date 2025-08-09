# TDK2025 Chassis Refine

This project implements a refined chassis velocity control system for the TDK2025 robot platform.

## Architecture Overview

The chassis control system is designed with a modular architecture consisting of several key components:

### Components

#### 1. Final Chassis Velocity Control Library
**Status:** TODO - Individual library for controlling chassis velocity
- Will provide high-level velocity control interface
- Planned to abstract complex motor coordination logic
- Expected to handle velocity setpoints and feedback control

#### 2. Motor Control (`motor.h`)
**Purpose:** Low-level motor control operations
- Direct motor speed and direction control
- Motor driver interface
- PWM signal generation and management
- Individual motor status monitoring

#### 3. Chassis Kinematics (`chassis.h`)
**Purpose:** Motion planning and motor coordination
- Computes required motor rotations for desired chassis movement
- Handles kinematic calculations for differential drive or mecanum wheels
- Translates high-level movement commands (forward, turn, strafe) into individual motor speeds
- Coordinate system transformations

#### 4. Line Tracing System (`trace.h`)
**Purpose:** Navigation assistance and positioning
- **Current functionality:** 
  - Assists chassis to move straight when following black lines
  - Line detection and tracking algorithms
- **Planned features:**
  - Point checking for position validation
  - True position refresh on the court
  - Additional navigation aids for autonomous operation

## Usage

The system follows a layered approach:
1. High-level commands → Chassis kinematics (`chassis.h`)
2. Motor commands → Motor control (`motor.h`)  
3. Navigation assistance → Line tracing (`trace.h`)

## Development Status

- ✅ Motor control implementation
- ✅ Chassis kinematics
- 🔄 Line tracing (basic functionality complete, enhancements planned)
- ⏳ Final velocity control library (TODO)

## Getting Started

[Add setup and usage instructions here]

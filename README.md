# Eye Controller

Arduino-based animatronic eye control system with realistic eye movements and blinking behavior.

## Features

* 6-servo eye system - Controls eye movement (LR/UD) and 4 independent eyelids
* Smooth movement - Exponential acceleration/deceleration curves for natural motion
* Idle behavior - Autonomous microsaccades, position drift, and natural blinking
* Coordinated actions - Multi-servo synchronized movements
* Interactive testing - Serial command interface for testing individual components

## Hardware Requirements

* Arduino (Uno/Nano/ESP32)
* Adafruit PWM Servo Driver (PCA9685)
* 6x Servo motors (SG90 or similar)
* 5V power supply for servos

## Servo Configuration

| Servo | Channel | Function |
|-------|---------|----------|
| 0 | SERVO_LOOK_LR | Eye left/right movement |
| 1 | SERVO_LOOK_UD | Eye up/down movement |
| 2 | SERVO_LID_TR | Top right eyelid |
| 3 | SERVO_LID_BR | Bottom right eyelid |
| 4 | SERVO_LID_TL | Top left eyelid |
| 5 | SERVO_LID_BL | Bottom left eyelid |

## Serial Commands

* `1-6` - Test individual servos
* `a` - Test all servos
* `c` - Center all servos
* `s` - Smooth movement demo




* `o` - Circular look-around
* `i` - Natural idle motion
* `f` - Final coordinated test

## Quick Start

1. Wire servos to PCA9685 board
2. Connect PCA9685 to Arduino (I2C: SDA/SCL + power)
3. Upload code and open Serial Monitor (9600 baud)
4. Use `c` to center, then `i` for idle behavior

Perfect for animatronics, robotics projects, or interactive displays requiring lifelike eye movement.

https://github.com/user-attachments/assets/c571d172-4f8d-4701-bc2a-3f71738db182



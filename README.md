Arduino-based animatronics for humanoid robot (KAAZI)
# Eye Controller (eyecontrollertest.ino)

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
## 3D model
https://willcogley.notion.site/7972fc916e4d425ab99d0a301dee5123?v=bd4c5db145434379af22f3a8193bfbe3&p=1af24779b64d80b19edfdd795d4b90e5&pm=c

https://github.com/user-attachments/assets/c571d172-4f8d-4701-bc2a-3f71738db182


# Neck & Jaw Controller (neckAndjawSmoothTest.ino)
Arduino-based animatronic head control system with smooth neck movement and jaw articulation for realistic character expressions.

## Features
* 3-DOF neck control - Yaw (left/right) and pitch (up/down) using dual-servo tilt mechanism
* Jaw articulation - Independent jaw control for speech and expressions
* Smooth motion - Exponential ease-in-out curves for natural movement
* Expression system - Pre-programmed emotions (happy, surprised, confused, talking)
* Multi-servo coordination - Synchronized movement of multiple servos
* Safety limits - Built-in constraints to prevent mechanical damage

## Hardware Requirements
* Arduino (Uno/Nano/ESP32)
* Adafruit PWM Servo Driver (PCA9685)
* 4x Servo motors (3x 270° for neck, 1x 180° for jaw)
* 5V power supply for servos

## Servo Configuration
| Servo | Channel | Function | Home Position |
|-------|---------|----------|---------------|
| 15 | SERVO_NECK_YAW | Neck yaw (left/right) | 120° |
| 14 | SERVO_NECK_LEFT | Neck pitch (left tilt) | 170° |
| 13 | SERVO_NECK_RIGHT | Neck pitch (right tilt) | 110° |
| 6 | JAW_CHANNEL | Jaw open/close | 96° (closed) |

## Serial Commands
* `c` - Center head and close jaw
* `l/r` - Look left/right
* `u/d` - Look up/down
* `o/j` - Open/close jaw
* `h` - Happy expression
* `s` - Surprised expression
* `t` - Talk sequence
* `n` - Nodding motion
* `k` - Head shake with jaw
* `z` - Test all movements

## Quick Start
1. Wire servos to PCA9685 board
2. Connect PCA9685 to Arduino (I2C: SDA/SCL + power)
3. Upload code and open Serial Monitor (9600 baud)
4. Use `c` to center, then try expressions with `h`, `s`, `t`

https://github.com/user-attachments/assets/833339ef-6eb4-4117-b984-a49a21ce2fa0




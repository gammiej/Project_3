

---

# Project 3 – Automotive Safety and Wiper Control System

## 1. Authors

* James Gammie
* Jacob Zeltser

Repository:
[https://github.com/gammiej/Project_3.git](https://github.com/gammiej/Project_3.git)

---

## 2. Project Overview

This project implements an embedded automotive control system that integrates:

* Ignition safety interlock logic
* Seat and seatbelt detection
* Windshield wiper speed and delay control
* LCD status display
* Servo-based wiper actuation

The system ensures vehicle safety requirements are satisfied before ignition activation and provides configurable windshield wiper operation.

---

## 3. System Features

### 3.1 Ignition Safety System

* Detects driver seat occupancy
* Detects passenger seat occupancy
* Detects seatbelt engagement
* Prevents ignition activation until all safety conditions are met
* Triggers alarm and terminal message if conditions are not satisfied

### 3.2 Wiper Control System

**Speed Modes:**

* HIGH
* LOW
* INT (Intermittent)
* OFF

**Delay Modes (INT only):**

* SHORT
* MEDIUM
* LONG

Additional behavior:

* HIGH and LOW operate at constant speed
* INT mode applies user-selected delay between wipe cycles
* LCD displays current system settings
* Wipers return to resting position when ignition is turned off

---

## 4. System Architecture

### 4.1 Hardware Components

* Microcontroller: ESP32-S3 
* LCD (HD44780)
* PARALLAX Servo motor
* Seat pressure sensors
* Seatbelt clip sensors
* Wiper (Servo ) Control knobs (frequency & delay)
* Alarm buzzer
* LCD Contrast Control Knobs

### 4.2 Software Modules

* Ignition Control Logic
* Sensor Input Handling
* Wiper Control State Machine
* LCD Interface Driver
* Servo Control Logic

---

## 5. Functional Behavior

### 5.1 Ignition Logic Flow

1. Check driver seat occupancy
2. Check passenger seat occupancy
3. Verify seatbelt engagement
4. If all conditions satisfied → Enable ignition
5. Otherwise → Trigger alarm and display error message

### 5.2 Wiper State Logic

* OFF → No motion
* LOW/HIGH → Continuous motion Fast/Slow
* INT → Intermittent motion with configurable delay

---

## 6. Design Decisions & Alternatives

* Why servo was selected
* Alternative safety logic approaches
* Possible interrupt-driven vs polling implementations
* Expansion possibilities (rain sensor, auto-wipers, etc.)

---

## 7. Testing

### 7.1 Wiper Subsystem Testing

| Specification  | Test Method                         | Result                                          |
| -------------- | ----------------------------------- | ----------------------------------------------- |
| Frequency Knob | Rotate knob through all positions   | Correct LCD display and wiper response — Passed |
| Delay Knob     | Rotate knob through all positions   | Delay timing adjusted correctly — Passed        |
| LCD Display    | Adjust setting changes              | Display updated correctly — Passed              |
| Servo          | compared operation to calculations  | Servo responded accurately — Passed             |

### 7.2 Ignition Subsystem Testing

| Specification      | Test Method                           | Result                                  |
| -------------------| ------------------------------------- | --------------------------------------- |
| Ignition Detection | Simulate occupied and buckled         | Ignition blocked when required — Passed |
| Alarm System       | press ignition without seats occupied | Alarm triggered if unfastened — Passed  |

---

## 8. How to install program

1. Clone or download repository
2. Unzip into desired folder
3. Open folder in Visual Studio Code
4. Build and upload to microcontroller

---

## 9. Future Improvements

* Automatic rain-sensing wipers
* Real-time vehicle speed integration
* Automatic Headlights integration

---


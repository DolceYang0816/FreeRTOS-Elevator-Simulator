# FreeRTOS Elevator Simulator

A four-floor elevator control system implemented on STM32 using FreeRTOS.

This project was developed as a FreeRTOS course project. It simulates the basic operation and scheduling logic of a four-floor elevator, including internal floor selection, external up/down calls, direction-based request scheduling, door timing, and real-time status output through UART.

---

## Features

- Four-floor elevator simulation (1F–4F)
- Internal floor selection using keypad buttons
- External up/down floor requests through UART
- Interrupt-based internal button input
- Direction-based request scheduling
- Priority handling for requests in the current moving direction
- Automatic door opening and closing
- 2-second door waiting time at each requested floor
- Multiple pending request management
- Real-time elevator status output through UART
- FreeRTOS-based real-time task scheduling

---

## Project Requirements

The simulated elevator operates in a four-floor building.

The system processes two types of input requests.

### 1. Internal Requests

Passengers inside the elevator can select a destination floor using the keypad.

Available destination floors:

- 1F
- 2F
- 3F
- 4F

The internal floor selection is handled using interrupts.

### 2. External Requests

Each floor can generate an external elevator call through UART.

The request contains the floor number and desired direction:

- `U` — Up
- `D` — Down

For example, an external request can indicate that a passenger on a certain floor wants to travel upward or downward.

---

## Elevator Scheduling Logic

The elevator follows the scheduling rules below.

### Rule 1: Direction Priority

When the elevator is moving in one direction, requests in the same direction are handled first.

For example, if the elevator is moving upward, it will continue serving valid upward requests before switching direction.

### Rule 2: Floor Arrival

When the elevator reaches a requested floor:

1. The elevator stops.
2. The door opens.
3. The door remains open for 2 seconds.
4. The door closes.
5. The elevator continues processing the remaining requests.

### Rule 3: Idle State

After all pending requests have been processed, the elevator remains at the last serviced floor with the door closed.

---

## System Status Output

The elevator status is displayed in real time through UART.

The output includes:

- Current floor
- Current moving direction
- Pending request list

Example:

```text
Current Floor: 2
Direction: UP
Pending Requests: 3, 4
```

Possible elevator directions include:

```text
UP
DOWN
IDLE
```

---

## System Workflow

The basic elevator control process is:

```text
Receive Request
      |
      v
Add Request to Queue
      |
      v
Determine Moving Direction
      |
      v
Move Toward Target Floor
      |
      v
Arrive at Requested Floor
      |
      v
Open Door
      |
      v
Wait 2 Seconds
      |
      v
Close Door
      |
      v
Process Remaining Requests
      |
      v
No Requests -> Idle
```

---

## Development Environment

This project uses:

- STM32 Microcontroller
- FreeRTOS
- STM32CubeMX
- STM32 HAL Library
- Keil MDK-ARM
- UART
- GPIO / External Interrupts

---

## Project Structure

```text
FreeRTOS-Elevator-Simulator/
│
├── BSP/
│   └── Board support and peripheral-related source files
│
├── Core/
│   ├── Inc/
│   └── Src/
│
├── Drivers/
│   ├── CMSIS/
│   └── STM32 HAL drivers
│
├── Fonts/
│   └── Font resources
│
├── MDK-ARM/
│   └── Keil MDK project files
│
├── Middlewares/
│   └── FreeRTOS middleware
│
├── .mxproject
├── myLCD.ioc
├── .gitignore
└── README.md
```

---

## FreeRTOS Design

FreeRTOS is used to manage the real-time behavior of the elevator control system.

The system separates elevator control logic from hardware input/output processing, allowing multiple events and requests to be handled in a structured real-time environment.

Typical system functions include:

- Receiving internal floor requests
- Receiving external elevator calls
- Maintaining pending requests
- Updating elevator direction
- Moving between floors
- Controlling door timing
- Printing system status through UART

---

## Request Handling

The elevator maintains pending floor requests and determines the next destination according to its current direction.

For example:

```text
Current Floor: 2
Direction: UP

Requests:
1F DOWN
3F UP
4F DOWN
```

While moving upward, the elevator gives priority to requests that can be served in the current direction.

After completing the requests in the current direction, the elevator can change direction to process the remaining requests.

---

## Door Control

When the elevator reaches a requested floor, the door follows this sequence:

```text
Elevator Arrives
      |
      v
Door Open
      |
      v
Delay 2 Seconds
      |
      v
Door Close
      |
      v
Continue Operation
```

The delay is controlled by the FreeRTOS timing mechanism.

---

## UART Communication

UART is used for:

1. Receiving simulated external elevator requests
2. Displaying elevator operating information

The serial output can be monitored using any standard serial terminal.

Examples include:

- Serial Assistant
- PuTTY
- MobaXterm
- XCOM
- Other UART terminal software

---

## Build and Run

### 1. Clone the Repository

```bash
git clone https://github.com/DolceYang0816/FreeRTOS-Elevator-Simulator.git
```

Enter the project directory:

```bash
cd FreeRTOS-Elevator-Simulator
```

### 2. Open the Keil Project

Open the Keil MDK project located in:

```text
MDK-ARM/
```

### 3. Build the Project

Build the project using Keil MDK-ARM.

### 4. Flash the Firmware

Connect the STM32 development board and download the firmware to the microcontroller.

### 5. Open a Serial Terminal

Connect to the configured UART interface to:

- Send external elevator requests
- View the current floor
- View the elevator direction
- View pending requests

---

## STM32CubeMX Configuration

The STM32CubeMX configuration file is included in the repository:

```text
myLCD.ioc
```

It can be opened with STM32CubeMX to inspect or modify:

- GPIO configuration
- UART configuration
- Interrupt configuration
- FreeRTOS configuration
- Clock configuration
- Peripheral configuration

---

## Example Scenario

Assume the elevator is currently at:

```text
Floor: 1
Direction: IDLE
```

The following requests are received:

```text
3F
2F
4F
```

The elevator begins moving upward.

A possible service sequence is:

```text
1F
 |
 v
2F
 |
 v
3F
 |
 v
4F
```

At every requested floor, the elevator:

```text
Stops
  ->
Opens Door
  ->
Waits 2 Seconds
  ->
Closes Door
  ->
Continues
```

After the final request is completed, the elevator remains at the last serviced floor.

---

## Possible Improvements

Future improvements could include:

- More advanced elevator scheduling algorithms
- Configurable number of floors
- Emergency stop functionality
- Door obstruction detection
- Overload detection
- LCD graphical interface improvements
- Elevator animation
- Multiple elevator coordination
- More detailed UART command parsing
- Request priority optimization

---

## Purpose

This project is designed for learning and practicing:

- FreeRTOS task scheduling
- Real-time embedded system design
- STM32 development
- Interrupt handling
- UART communication
- Embedded state-machine design
- Request scheduling algorithms
- Hardware and software integration

---

## Author

**DolceYang0816**

FreeRTOS Course Project

---

## License

This project is intended primarily for educational and learning purposes.

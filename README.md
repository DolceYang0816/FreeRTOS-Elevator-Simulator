## FreeRTOS Elevator Simulator

A four-floor elevator control system implemented on STM32 using FreeRTOS.

This project was developed as a FreeRTOS course project. It simulates basic elevator scheduling, including internal floor requests, external up/down calls, direction control, door timing, and UART status output.

### Features

- Four-floor elevator simulation (1F–4F)
- Internal floor selection
- External up/down requests through UART
- Direction-based request scheduling
- Automatic door opening and closing
- 4-second door waiting time
- Pending request management
- Real-time status output through UART
- FreeRTOS-based task scheduling

### Scheduling Logic

The elevator follows these basic rules:

1. Requests in the current moving direction are handled first.
2. When the elevator reaches a requested floor, the door opens for 4 seconds.
3. After the door closes, the elevator continues processing remaining requests.
4. When all requests are completed, the elevator stays at the last serviced floor.

### Development Environment

- STM32
- FreeRTOS
- STM32CubeMX
- Keil MDK-ARM
- STM32 HAL
- UART

### Project Structure

```text
FreeRTOS-Elevator-Simulator/
├── BSP/            # Board support files
├── Core/           # Main application code
├── Drivers/        # STM32 HAL and CMSIS drivers
├── Fonts/          # Font resources
├── MDK-ARM/        # Keil project files
├── Middlewares/    # FreeRTOS middleware
├── myLCD.ioc       # STM32CubeMX configuration
└── README.md
```

### Build

Clone the repository:

```bash
git clone https://github.com/DolceYang0816/FreeRTOS-Elevator-Simulator.git
```

Open the project in:

```text
MDK-ARM/
```

Then build and flash the project using Keil MDK-ARM.

### License

This project is intended primarily for educational and learning purposes.

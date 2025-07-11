# Servo_Modbus_RTU_lib

A comprehensive C++ library for controlling Lichuan servo drives using Modbus RTU protocol over RS485 serial communication.

## Overview

This project provides a complete C++ implementation for interfacing with Lichuan servo drives (LCDA630P and LCDA6 models) using the Modbus RTU protocol. It includes a robust library with interactive examples for testing and demonstration purposes.

## Features

- **Complete Modbus RTU Implementation**: Full support for reading/writing servo parameters
- **Multi-Model Support**: Compatible with Lichuan LCDA630P and LCDA6 servo drives
- **Multiple Control Modes**: Position, speed, and torque control with easy mode switching
- **Interactive Command Interface**: Menu-driven testing application for servo operations
- **Cross-Platform**: Designed for Linux systems, including Raspberry Pi
- **Comprehensive Parameter Management**: Support for 16-bit and 32-bit parameter operations
- **Real-time Monitoring**: Position, speed, and status monitoring capabilities
- **Error Handling**: Robust error handling and timeout management

## Key Capabilities

- **Position Control**: Absolute and relative positioning with configurable speed and torque
- **Speed Control**: Velocity-based movement with direction control
- **Torque Control**: Direct torque setting and monitoring
- **Parameter Configuration**: Read/write servo parameters for customization
- **Status Monitoring**: Real-time position, speed, and encoder feedback
- **Serial Communication**: Reliable RS485 communication using Boost.Asio

## Getting Started

### Prerequisites

- Lichuan LCDA630P or LCDA6 Servo Drive
- RS485 to USB converter (or equivalent)
- Computer with a serial port or USB port
- C++ compiler with C++17 support
- CMake build system
- Boost libraries (for serial communication)

### Installation

1. Clone the repository:
    ```sh
    git clone https://github.com/yourusername/Servo_Modbus_RTU_lib.git
    ```
2. Navigate to the project directory:
    ```sh
    cd Servo_Modbus_RTU_lib
    ```
3. Install required system packages:
    ```sh
    sudo apt update
    sudo apt-get install libnet1-dev
    sudo apt install libboost-all-dev
    ```
4. Build the project:
    ```sh
    mkdir build
    cd build
    cmake ..
    make
    ```

### Usage

1. Connect the RS485 to USB converter to your computer and the Lichuan servo drive
2. Configure the serial port settings (baud rate: 57600, parity: even, stop bits: 1)
3. Run the interactive example:
    ```sh
    clear; make; ./example_linux_LCDA6
    clear; make; ./example_linux_LCDA6
    ```
4. Use the menu interface to test various servo operations:
    - `r` - Read parameters
    - `w` - Write parameters
    - `m` - Move relative
    - `a` - Move absolute
    - `s` - Speed control
    - `t` - Torque control
    - `p` - Get position
    - `d` - Disable servo

### Development Workflow with Raspberry Pi

```sh
# Copy files to Raspberry Pi
scp -r lichuan/ examples/ CMakeLists.txt user@10.0.10.104:/home/user/git_workspace/Servo_Modbus_RTU_lib/

# SSH into Raspberry Pi
ssh user@10.0.10.104
```

## Library Architecture

- **`LichuanMotion`**: Base class providing core Modbus RTU functionality
- **`LCDA6`**: Specific implementation for LCDA6 servo model
- **`LCDA630P`**: Specific implementation for LCDA630P servo model
- **Interactive Examples**: Complete demonstration applications

## Documentation

For detailed documentation on the Modbus RTU protocol and the Lichuan servo drives, refer to the official manuals and datasheets provided by the manufacturer.

## Contributing
@tuptusstokrotka
Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contact

For any questions or support, please open an issue on the GitHub repository.

# Lichuan LCDA630 Modbus RTU

This project is designed to run the Lichuan LCDA630 Servo using Modbus RTU protocol.

## Overview

The Lichuan LCDA630 is a high-performance servo drive that can be controlled using the Modbus RTU protocol. This project provides the necessary code and instructions to interface with the servo drive and control its operations.

## Features

- Control Lichuan LCDA630 Servo via Modbus RTU
- Example code for basic operations
- Configuration instructions

## Getting Started

### Prerequisites

- Lichuan LCDA630 Servo Drive
- RS485 to USB converter (or equivalent)
- Computer with a serial port or USB port
- Modbus RTU library (e.g., pymodbus for Python)

### Installation

1. Clone the repository:
    ```sh
    git clone https://github.com/yourusername/Lichuan_LCDA630_Modbus_RTU.git
    ```
2. Navigate to the project directory:
    ```sh
    cd Lichuan_LCDA630_Modbus_RTU
    ```
3. Install the required submodules:
    ```sh
    git submodule update --init --recursive
    ```
4. Install required system packages:
    ```sh
    sudo apt update
    sudo apt-get install libnet1-dev
    sudo apt install libboost-all-dev
    ```
### Usage

1. Connect the RS485 to USB converter to your computer and the Lichuan LCDA630 Servo Drive.
2. Configure the serial port settings (e.g., baud rate, parity, etc.) as per the servo drive's manual.
3. Build the program with cmake:
    ```sh
    mkdir build
    cd build
    cmake ..
    ```
4. Run the program:
    ```sh
    make
    ./Lichuan_RS_Libary
    ```
5. Use folowing comand to run at once:
    ```sh
    clear; make; ./example_linux
    ```

## Documentation

For detailed documentation on the Modbus RTU protocol and the Lichuan LCDA630 Servo Drive, refer to the official manuals and datasheets provided by the manufacturer.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contact

For any questions or support, please open an issue on the GitHub repository.

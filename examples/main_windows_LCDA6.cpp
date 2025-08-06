#include <iostream>
#include <iomanip>
#include <bitset>
#include <windows.h>
#include <chrono>
#include <thread>
#include "LCDA6.hpp"
#include <algorithm>
#include <vector>
#include <sstream>
#include <string>

// Function declarations
std::vector<std::string> splitString(const std::string& str, char delimiter);

class WindowsSerial {
private:
    HANDLE hSerial;
    bool isOpen;

public:
    WindowsSerial() : hSerial(INVALID_HANDLE_VALUE), isOpen(false) {}
    
    bool open(const std::string& portName, int baudRate = 57600) {
        std::string fullPortName = "\\\\.\\" + portName; // Windows requires this format
        
        hSerial = CreateFileA(
            fullPortName.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            0,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            0
        );
        
        if (hSerial == INVALID_HANDLE_VALUE) {
            std::cerr << "Error opening serial port: " << GetLastError() << std::endl;
            return false;
        }
        
        // Configure serial port
        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Error getting serial port state" << std::endl;
            CloseHandle(hSerial);
            return false;
        }
        
        dcbSerialParams.BaudRate = baudRate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = EVENPARITY;
        
        if (!SetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "Error setting serial port state" << std::endl;
            CloseHandle(hSerial);
            return false;
        }
        
        // Set timeouts
        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;
        
        if (!SetCommTimeouts(hSerial, &timeouts)) {
            std::cerr << "Error setting timeouts" << std::endl;
            CloseHandle(hSerial);
            return false;
        }
        
        isOpen = true;
        return true;
    }
    
    bool write(const std::string& data) {
        if (!isOpen) return false;
        
        DWORD bytesWritten;
        return WriteFile(hSerial, data.c_str(), data.length(), &bytesWritten, NULL) != 0;
    }
    
    std::vector<uint8_t> read(size_t expectedSize, int timeoutMs = 100) {
        std::vector<uint8_t> buffer(expectedSize);
        
        if (!isOpen) return buffer;
        
        DWORD bytesRead = 0;
        DWORD totalRead = 0;
        
        auto startTime = std::chrono::steady_clock::now();
        
        while (totalRead < expectedSize) {
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime);
            
            if (elapsed.count() > timeoutMs) {
                std::cerr << "Read timeout" << std::endl;
                break;
            }
            
            if (ReadFile(hSerial, buffer.data() + totalRead, expectedSize - totalRead, &bytesRead, NULL)) {
                totalRead += bytesRead;
            } else {
                std::cerr << "Read error: " << GetLastError() << std::endl;
                break;
            }
        }
        
        buffer.resize(totalRead);
        return buffer;
    }
    
    void close() {
        if (isOpen) {
            CloseHandle(hSerial);
            isOpen = false;
        }
    }
    
    ~WindowsSerial() {
        close();
    }
};

// Global serial instance
WindowsSerial serial;

std::vector<uint8_t> send_wrapper(const std::vector<uint8_t>& request) {
    std::string request_string = "";
    
    for (int i = 0; i < request.size(); i++) {
        request_string += static_cast<char>(request[i]);
    }
    
    // Send data
    if (!serial.write(request_string)) {
        std::cerr << "Failed to write to serial port" << std::endl;
        return std::vector<uint8_t>();
    }
    uint16_t size = 5 + request[5] * 3 ;
    if (size < 8)
        size = 8;
    // Read response
    std::vector<uint8_t> response = serial.read(size, 20);
    
    // Print for debugging
    std::cout << "send: ";
    for (uint8_t byte : request) {
        std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(byte) << " ";
    }
    std::cout << std::endl;
    
    std::cout << "receive: ";
    for (uint8_t c : response) {
        std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(c) << " ";
    }
    std::cout << std::endl;
    
    return response;
}
int main()
{
    // Initialize serial port (change COM port as needed)
    if (!serial.open("COM2", 19200)) {
        std::cerr << "Failed to open serial port. Make sure the port exists and is not in use." << std::endl;
        return 1;
    }
    
    std::cout << "Serial port opened successfully!" << std::endl;    
    LCDA6 servo;
    char mode ;
    std::string s ; 
    while (true)
    {
        std::cout << "choose what what to do: a[absolute], b[read_servo_brief], c[config_for_modbus_control_position_speed],\nd[disable], e[enable], j[time based loop], l[position loop], m[move],\no[acutal pulse position], p[acutal position], q[quit], r[read], s[speed], t[torque], w[write]" << std::endl ; 
        std::cin >> mode ; 
        switch (mode)
        {
        case 'a' :
        {
            std::cout << "Type position to move absolute or q to quit" << std::endl ;
            std::cin >> s ;      
            int64_t position = std::stoi(s);  
            servo.config_for_modbus_control_position_speed(1, send_wrapper);
            servo.moveAbsolute(1, position, send_wrapper);
            break;
        }
        case 'b':
        {
            std::vector<std::vector<uint8_t>> params = servo.read_servo_brief(1, send_wrapper);
            break;
        }
        case 'c':
        {
            std::vector<std::vector<uint8_t>> config = servo.config_for_modbus_control_position_speed(1, send_wrapper);
            break;
        }
        case 'd' :
        {
            bool response = servo.disable(1, send_wrapper);
            break;
        }
        case 'e' :
        {
            servo.enable(1, send_wrapper);
            break;
        }
        case 'j' :
        {
            std::cout << "Endless loop (time-based toggle), press any key to stop" << std::endl;
            std::cout << "Type speed value, toggle interval in ms, and add_pos (comma separated, e.g. 50,1000,125), or q to quit" << std::endl;
            std::cin >> s;
            int32_t speed = 50;
            int32_t add_pos = 125;
            int interval_ms = 1000;
            {
                std::vector<std::string> params = splitString(s, ',');
                if (params.size() == 1) {
                    speed = std::stoi(params[0]);
                } else if (params.size() == 2) {
                    speed = std::stoi(params[0]);
                    interval_ms = std::stoi(params[1]);
                } else if (params.size() >= 3) {
                    speed = std::stoi(params[0]);
                    interval_ms = std::stoi(params[1]);
                    add_pos = std::stoi(params[2]);
                }
            }
            int64_t pos = servo.get_actual_pulse_position(1, send_wrapper);
            std::cout << "Actual Pulse position is : " << std::dec << pos << " hex : 0x" << std::hex << pos << std::flush;
            std::vector<std::vector<uint8_t>> config = servo.config_for_modbus_control_position_speed(1, send_wrapper);
            servo.moveVelocity(1, -speed, send_wrapper);
            servo.enable(1, send_wrapper);
            int64_t pos_to_toggle = pos - add_pos;
            bool direction = false;
            auto last_toggle = std::chrono::steady_clock::now();
            while (true) {
                system("clear");
                int64_t current_pos = servo.get_actual_pulse_position(1, send_wrapper);
                std::cout << "\rActual Pulse position is : " << std::dec << current_pos
                          << " pos_to_toggle: " << std::dec << pos_to_toggle
                          << " speed: " << speed
                          << " interval_ms: " << interval_ms
                          << std::flush;
                auto now = std::chrono::steady_clock::now();
                static bool waiting_extra = false;
                static auto extra_wait_start = std::chrono::steady_clock::now();

                if (!direction && waiting_extra) {
                    // Currently in -speed direction and waiting extra time
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - extra_wait_start).count() >= interval_ms) {
                        waiting_extra = false;
                        last_toggle = now;
                        direction = !direction;
                        pos_to_toggle = pos + add_pos;
                        servo.moveVelocity(1, speed, send_wrapper);
                    }
                } else if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_toggle).count() >= interval_ms) {
                    direction = !direction;
                    if (direction) {
                        pos_to_toggle = pos + add_pos;
                        servo.moveVelocity(1, speed, send_wrapper);
                    } else {
                        pos_to_toggle = pos - add_pos;
                        servo.moveVelocity(1, -speed, send_wrapper);
                        // Start extra wait for -speed direction
                        waiting_extra = true;
                        extra_wait_start = std::chrono::steady_clock::now();
                    }
                    if (!waiting_extra) {
                        last_toggle = now;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (std::cin.rdbuf()->in_avail()) {
                    std::cin.get(); // Clear the input buffer
                    break;
                }
            }
            break;
        }
        case 'l' :
        {
            std::cout << "Endless loop, press any key to stop" << std::endl ;
            std::cout << "Type speed, movement, or q to quit" << std::endl ;
            std::cin >> s ;      
            int32_t speed = 1000;
            int32_t movement = 10000;
            std::vector<std::string> params = splitString(s, ',');
            if (params.size() == 1)
            {
                speed = std::stoi(params[0]);
            }
            else if (params.size() == 2)
            {
                speed = std::stoi(params[0]);
                movement = std::stoi(params[1]);
            }
            std::vector<std::vector<uint8_t>> config = servo.config_for_modbus_control_position_speed(1, send_wrapper);
            int64_t pos = servo.get_actual_pulse_position(1, send_wrapper);
            std::cout << "Actual Pulse position is : " << std::dec << pos << " hex : 0x" << std::hex << pos << std::flush << std::endl;
            uint64_t i=1;
            int64_t pos_to_toggle = pos - movement;
            servo.moveAbsolute(1, pos_to_toggle, send_wrapper);
            while (true) {
                system("clear");
                if (i%2 == 0)
                    pos_to_toggle = pos + movement;
                else
                    pos_to_toggle = pos - movement * 2 ;
                std::cout << "\rActual Pulse position is : " << std::dec << pos << " i: " << std::dec << i << " pos_to_toggle: " << std::dec << pos_to_toggle << std::flush << std::endl;
                bool in_target_position = servo.inTargetPosition(1, send_wrapper);
                if (i%2 == 0 && in_target_position)
                {
                    servo.moveAbsolute(1, pos_to_toggle, send_wrapper);
                    pos = pos_to_toggle;
                    i++;
                }
                else if (i%2 == 1 && in_target_position)
                {
                    servo.moveAbsolute(1, pos_to_toggle, send_wrapper);
                    pos = pos_to_toggle;
                    i++;
                } 
                std::cout << std::endl;
            }
            break;
        }
        case 'm' :
        {
            std::cout << "Type position to move, speed, torque in format POSITION,SPEED,TORQUE" << std::endl ;
            std::cin >> s ;
            std::vector<std::string> params = splitString(s, ',');
            int32_t position = 10000;
            int32_t speed = 1000;
            float torque = 10.0;
            std::vector<std::vector<uint8_t>> config = servo.config_for_modbus_control_position_speed(1, send_wrapper);
            if (params.size() == 1)
            {
                position = std::stoi(params[0]);
                std::vector<std::vector<uint8_t>> one_rot = servo.moveRelative(1, position, send_wrapper);  
            }
            else if (params.size() == 2)
            {
                position = std::stoi(params[0]);
                speed = std::stoi(params[1]);
                std::vector<std::vector<uint8_t>> one_rot = servo.moveRelative(1, position, send_wrapper, speed);  
            }
            else if (params.size() == 3)    
            {
                position = std::stoi(params[0]);
                speed = std::stoi(params[1]);
                torque = std::stof(params[2]);
                std::vector<std::vector<uint8_t>> one_rot = servo.moveRelative(1, position, send_wrapper, speed, torque);  
            }
            else
            {
                std::vector<std::vector<uint8_t>> one_rot = servo.moveRelative(1, position, send_wrapper);  
            }
            break;
        } 
        case 'o' :
        {
            int64_t pos = servo.get_actual_pulse_position(1, send_wrapper);
            std::cout << "Actual Absolut position is : " << std::dec << pos << " hex : 0x" << std::hex << pos << " bin : 0b" << std::bitset<64>(pos) << std::endl ;
            break;
        }         
        case 'p' :
        {
            int64_t pos = servo.get_actual_mechanical_position(1, send_wrapper);
            std::cout << "Actual Absolut position is : " << std::dec << pos << " hex : 0x" << std::hex << pos << " bin : 0b" << std::bitset<64>(pos) << std::endl ;
            break;
        }   
        case 'q' : 
        {
            std::cout << "Quiting..." << std::endl;
            return 0 ;
        }       
        case 'r':
        {
            std::cout << "*****************Read Param*****************" << std::endl;
            std::cout << "Type Address in format ADDRESS, SIZE[OPTIONAL]" << std::endl ;
            std::cin >> s ;
            std::vector<std::string> params = splitString(s, ',');
            int address = std::stoi(params[0]);
            uint8_t size = 1;
            if (params.size() == 1)
                servo.read_parameter(1, address, size, send_wrapper);
            else 
            {
                size = std::stoi(params[1]);
                servo.read_parameter(1, address, size, send_wrapper);
            }
            std::cout << "*****************Read Param*****************" << std::endl;  
            break; 
        }
        case 's' :
        {
            std::cout << "Type speed value or q to quit" << std::endl ;
            std::cin >> s ;      
            int32_t speed = std::stoi(s);  
            std::vector<std::vector<uint8_t>> config = servo.config_for_modbus_control_position_speed(1, send_wrapper);
            std::vector<std::vector<uint8_t>> one_rot = servo.moveVelocity(1, speed, send_wrapper);  
            break;
        }         
        case 't' :
        {
            std::cout << "Type Torque[%]" << std::endl ;
            std::cin >> s ;      
            std::replace(s.begin(), s.end(), ',', '.');
            float torque = std::stof(s);  
            std::vector<std::vector<uint8_t>> config = servo.set_torque(1, torque, send_wrapper);
            break;
        }
        case 'w':
        {
            std::cout << "*****************Write Param*****************" << std::endl;
            int32_t value ;  
            int address = 0;
            std::cout << "Type Address and value in format ADDRESS,VALUE" << std::endl ;
            std::cin >> s ;
            std::vector<std::string> params = splitString(s, ',');
            if (params.size() >= 2 )
            {
                address = std::stoi(params[0]);
                value = std::stoi(params[1]);
            }
            else
            {
                std::cerr << "type at least 2 values, typed: " << params.size() << std::endl ;
                break;
            }
            if (abs(value) > 65535)
                servo.write_parameter_32(1, address, value, send_wrapper);
            else
                servo.write_parameter(1, address, value, send_wrapper);
            std::cout << "*****************Write Param*****************" << std::endl;  
            break; 
        }    
        case 'x' :
        {
            std::vector<std::pair<uint16_t, uint16_t>> output_state = servo.get_output_state(1, send_wrapper);
            for (int i = 0; i < output_state.size(); i++)
            {
                std::cout << "Output state " << i << " is " << output_state[i].first << " and value is " << output_state[i].second << std::endl;
            }
            break;
        }
        default:
            break;
        } 
    } 
    return 0;
}

// Function definitions
std::vector<std::string> splitString(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}
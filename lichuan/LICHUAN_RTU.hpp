#ifndef LICHUAN_RTU_HPP
#define LICHUAN_RTU_HPP

#pragma once

#define DEBUG_SERIAL true
#if (DEBUG_SERIAL == true)
    #include <iostream>
    #define DEBUG_SERIAL_PRINTLN(x) std::cout << x << std::endl;
    #define DEBUG_SERIAL_PRINT(x)   std::cout << x;
#else
    #define DEBUG_SERIAL_PRINTLN(x)
    #define DEBUG_SERIAL_PRINT(x)
#endif

#include <cstdint>
#include <vector>
#include <iomanip>
#include <sstream>
#include <functional>

enum servomode {
    undefined,
    Speed,
    Position,
    Torque,
    TorqueSpeed,
    SpeedPosition,
    TorquePosition,
    Hybrid
};

class I_Lichuan_RTU {
protected:
    // Status
    servomode eControlMode = servomode::undefined;
    // Configuration
    uint8_t slave_id            = 1;
    bool controlOverModbus      = false;
    int64_t encoder_resolution  = 8388600;
    int64_t pulse_per_rotation  = 10000;
    bool lower16_bit_first      = true;
    std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction; //CHECK if use it this way

    // Feedback
    int64_t ActualAbsolutePosition      = 0;
    int16_t ActualSpeedRpm              = 0;
    int16_t ActualTorque                = 0;
    // Setpoints
    int64_t SetpointAbsolutePosition    = 0;
    int16_t SetpointSpeedRpm            = 0;
    int16_t SetpointTorque              = 0;

public:
    I_Lichuan_RTU(std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction) : sendFunction(sendFunction) {}
    virtual ~I_Lichuan_RTU() = default;

    inline void setMode(servomode mode) { eControlMode = mode; }
    inline servomode getMode() { return eControlMode; }

    // Configuration
    virtual std::vector<std::vector<uint8_t>> servo_config() = 0;
    virtual std::vector<std::vector<uint8_t>> read_servo_brief() = 0;

    // Commands
    std::vector<int32_t> processListoOfCommands(std::vector<std::vector<uint8_t>> &listOfCommands);
    std::string vector_to_string(std::vector<uint8_t> frame);
    void debug_print_frame(std::vector<uint8_t> frame, bool print=true);

    // Modbus functions
    std::vector<uint8_t> read_parameter (uint16_t address, uint16_t size);
    std::vector<uint8_t> write_parameter (uint16_t address, int16_t value);
    std::vector<uint8_t> write_parameter_32 (uint16_t address, int32_t value);
    uint16_t crcValueCalc(const uint8_t *data, uint16_t length);
    int32_t parseModbusResponse(const std::vector<uint8_t> &response);
    std::vector<uint8_t> send_and_receive(std::vector<uint8_t> frame);

    // Enable / Disable Servo
    virtual bool enable() = 0;
    virtual bool disable() = 0;

    // Speed
    virtual std::vector<std::vector<uint8_t>> config_for_modbus_control_speed() = 0;
    virtual int16_t get_actual_speed() = 0;
    virtual std::vector<std::vector<uint8_t>> set_speed(int16_t speed) = 0;
    virtual std::vector<std::vector<uint8_t>> move_velocity(int16_t speed) = 0;

    // Position
    virtual std::vector<std::vector<uint8_t>> config_for_modbus_control_position() = 0;
    virtual int64_t get_actual_position() = 0;
    virtual std::vector<std::vector<uint8_t>> set_position(int64_t position) = 0;
    virtual std::vector<std::vector<uint8_t>> move_relative(int64_t position) = 0;
    virtual std::vector<std::vector<uint8_t>> move_absolute(int64_t position) = 0;

    // Torque
    virtual std::vector<std::vector<uint8_t>> config_for_modbus_control_torque() = 0;
    virtual int16_t get_actual_torque() = 0;
    virtual std::vector<std::vector<uint8_t>> set_torque(int16_t torque) = 0;

    // Debug
    virtual std::vector<std::vector<uint8_t>> raw_one_rotation() = 0;
};

#endif // LICHUAN_RTU_HPP

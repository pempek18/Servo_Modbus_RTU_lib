#ifndef LCDA630P_MODBUS_RTU_HPP
#define LCDA630P_MODBUS_RTU_HPP

#define DEBUG_SERIAL false
#ifdef DEBUG_SERIAL
#include <iostream>
#define DEBUG_SERIAL_PRINTLN(x) std::cout << x << std::endl;
#define DEBUG_SERIAL_PRINT(x) std::cout << x;
#else
#define DEBUG_SERIAL_PRINTLN(x)
#define DEBUG_SERIAL_PRINT(x)
#endif

#include <cstdint>
#include <vector>
#include <iomanip>
#include <sstream>
#include <functional>
class LCDA630P_Modbus_RTU
{
    private : 
    uint16_t MotorNumber ; 
    uint16_t RatedVoltage ;
    uint16_t RatedPower ;
    uint16_t RatedCurrent ;
    uint16_t RatedTorque ;
    uint16_t MaxTorque ;
    uint16_t RatedSpeed ;
    uint16_t MaxSpeed ;
    uint32_t PositionOffsetOfAbsolutEncoder ;
    uint16_t controlOverModbus;
    bool lower16_bit_first = true ; 
    
    public :
        LCDA630P_Modbus_RTU();
        void scan_devices();
        void debug_print_frame(std::vector<uint8_t> frame, bool print);
        std::vector<uint8_t> read_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset);
        std::vector<uint8_t> read_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        std::vector<uint8_t> write_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int16_t value);
        std::vector<uint8_t> write_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int16_t value, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        std::vector<uint8_t> write_parameter_32(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int32_t value);        
        std::vector<uint8_t> write_parameter_32(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int32_t value, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);        
        std::vector<std::vector<uint8_t>>  read_servo_brief(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        std::vector<std::vector<uint8_t>>  raw_one_rotation(uint8_t slave_id);
        std::vector<std::vector<uint8_t>>  move_to_position(uint8_t slave_id, int32_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        std::vector<std::vector<uint8_t>>  speed_command(uint8_t slave_id, int32_t speed, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        std::vector<std::vector<uint8_t>>  config_for_modbus_control_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        std::vector<std::vector<uint8_t>>  config_for_modbus_control_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        std::string vector_to_string(std::vector<uint8_t>);
        uint32_t parseModbusResponse(const std::vector<uint8_t>& response) ;
        uint16_t crcValueCalc(const uint8_t *data, uint16_t length);
        bool controledOverModbus();

};

#endif // LCDA630P_MODBUS_RTU_HPP
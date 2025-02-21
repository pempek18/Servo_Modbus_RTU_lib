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
class LCDA630P_Modbus_RTU
{
    private : 
        bool lower16_bit_first = true ; 
    public :
        LCDA630P_Modbus_RTU();
        void scan_devices();
        void debug_print_frame(std::vector<uint8_t> frame, bool print);
        std::vector<uint8_t> read_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset);
        std::vector<uint8_t> write_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, uint16_t value);
        std::vector<uint8_t> write_parameter_32(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int32_t value);        
        std::vector<std::vector<uint8_t>>  read_servo_brief(uint8_t slave_id);
        std::vector<std::vector<uint8_t>>  raw_one_rotation(uint8_t slave_id);
        std::vector<std::vector<uint8_t>>  move_to_position(uint8_t slave_id, u_int32_t position);
        std::vector<std::vector<uint8_t>>  config_for_modbus_control(uint8_t slave_id);
        std::string vector_to_string(std::vector<uint8_t>);
        uint32_t parseModbusResponse(const std::vector<uint8_t>& response) ;
        uint16_t crcValueCalc(const uint8_t *data, uint16_t length);

};

#endif // LCDA630P_MODBUS_RTU_HPP
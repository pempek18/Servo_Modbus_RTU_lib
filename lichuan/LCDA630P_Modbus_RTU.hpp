#ifndef LCDA630P_MODBUS_RTU_HPP
#define LCDA630P_MODBUS_RTU_HPP

#define DEBUG_SERIAL true
#ifdef DEBUG_SERIAL
#include <iostream>
#define DEBUG_SERIAL_PRINTLN(x) std::cout << x << std::endl;
#define DEBUG_SERIAL_PRINT(x) std::cout << x;
#else
#define DEBUG_SERIAL_PRINTLN(x)
#define DEBUG_SERIAL_PRINT(x)
#endif

#include <cstdint>

class LCDA630P_Modbus_RTU
{
    public :
        LCDA630P_Modbus_RTU();
        void scan_devices();
        uint32_t read_parameter(uint8_t* frame, uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, uint16_t function_code_number);
        uint16_t crcValueCalc(const uint16_t *data, uint16_t length);
};

#endif // LCDA630P_MODBUS_RTU_HPP
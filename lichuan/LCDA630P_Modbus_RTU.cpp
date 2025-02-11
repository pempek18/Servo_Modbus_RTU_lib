#include "LCDA630P_Modbus_RTU.hpp"

LCDA630P_Modbus_RTU::LCDA630P_Modbus_RTU()
{
    DEBUG_SERIAL_PRINTLN("init");
};
void LCDA630P_Modbus_RTU::scan_devices() {
    DEBUG_SERIAL_PRINTLN("Scanning devices");
}
uint32_t LCDA630P_Modbus_RTU::read_parameter(uint8_t* frame, uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, uint16_t function_code_number)
{
    for (int i = 0; i < 7; i++) {
        frame[i] = 0;
    }
    frame[0] = slave_id; //ADR
    frame[1] = 0x03; // Read Holding Register
    frame[2] = group_number;
    frame[3] = parameter_offset;
    frame[4] = function_code_number >> 8;
    frame[5] = function_code_number & 0xFF;
    uint16_t data[2];
    uint8_t cnt = 0;
    for (int i = 2; i < 6; i=i+2) {
        data[0] = frame[i];
        data[0] <<= 8;
        data[0] |= frame[i+1];
        cnt++;
    }
    frame[6] = crcValueCalc(data, 2);
#if DEBUG_SERIAL
    for (int i = 0; i < 7; i++) {   
        std::cout << "0x" << std::hex << frame[i] << " ";
    }
    std::cout << std::endl;
#endif
    uint32_t parameter_value = 0;
    return parameter_value;
};
uint16_t LCDA630P_Modbus_RTU::crcValueCalc(const uint16_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint16_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
};
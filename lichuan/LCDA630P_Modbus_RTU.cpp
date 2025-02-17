#include "LCDA630P_Modbus_RTU.hpp"

LCDA630P_Modbus_RTU::LCDA630P_Modbus_RTU()
{
    DEBUG_SERIAL_PRINTLN("init");
};
void LCDA630P_Modbus_RTU::scan_devices()
{
    DEBUG_SERIAL_PRINTLN("Scanning devices");
}

std::vector<uint8_t> LCDA630P_Modbus_RTU::read_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, uint16_t function_code_number)
{
    std::vector<uint8_t> frame;
    frame.push_back(slave_id); // ADR
    frame.push_back(0x03);     // Read Holding Register
    frame.push_back(group_number);
    frame.push_back(parameter_offset);
    frame.push_back(function_code_number >> 8);
    frame.push_back(function_code_number & 0xFF);

    // Calculate CRC using uint8_t data
    uint16_t crc = crcValueCalc(frame.data(), frame.size());
    frame.push_back(crc & 0xFF);        // Low byte
    frame.push_back((crc >> 8) & 0xFF); // High byte

#if DEBUG_SERIAL
    for (int i = 0; i < frame.size(); i++)
    {
        std::stringstream ss;
        ss << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(frame[i]) << " ";
        DEBUG_SERIAL_PRINT(ss.str());
    }
    DEBUG_SERIAL_PRINTLN("");
#endif
    return frame;
}
std::vector<uint8_t> LCDA630P_Modbus_RTU::write_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, uint16_t value)
{
    std::vector<uint8_t> frame;
    frame.push_back(slave_id); // ADR
    frame.push_back(0x06);     // Write Holding Register
    frame.push_back(group_number);
    frame.push_back(parameter_offset);
    frame.push_back(value >> 8);
    frame.push_back(value & 0xFF);

    // Calculate CRC using uint8_t data
    uint16_t crc = crcValueCalc(frame.data(), frame.size());
    frame.push_back(crc & 0xFF);        // Low byte
    frame.push_back((crc >> 8) & 0xFF); // High byte
#if DEBUG_SERIAL
    for (int i = 0; i < frame.size(); i++)
    {
        std::stringstream ss;
        ss << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(frame[i]) << " ";
        DEBUG_SERIAL_PRINT(ss.str());
    }
    DEBUG_SERIAL_PRINTLN("");
#endif
    return frame;
};
std::vector<uint8_t> LCDA630P_Modbus_RTU::write_parameter_32(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, uint32_t value)
{
    std::vector<uint8_t> frame;
    frame.push_back(slave_id); // ADR
    frame.push_back(0x10);     // Write Holding Register
    frame.push_back(group_number);
    frame.push_back(parameter_offset);
    frame.push_back(0x00);
    frame.push_back(0x02);
    frame.push_back(value >> 24);
    frame.push_back((value >> 16) & 0xFF);
    frame.push_back((value >> 8) & 0xFF);
    frame.push_back(value & 0xFF);
    // Calculate CRC using uint8_t data
    uint16_t crc = crcValueCalc(frame.data(), frame.size());
    frame.push_back(crc & 0xFF);        // Low byte
    frame.push_back((crc >> 8) & 0xFF); // High byte
#if DEBUG_SERIAL
    for (int i = 0; i < frame.size(); i++)
    {
        std::stringstream ss;
        ss << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(frame[i]) << " ";
        DEBUG_SERIAL_PRINT(ss.str());
    }
    DEBUG_SERIAL_PRINTLN("");
#endif
    return frame;    
}
std::vector<std::vector<uint8_t>> LCDA630P_Modbus_RTU::read_servo_brief(uint8_t slave_id)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    list_of_commands.push_back(read_parameter(slave_id, 0, 0, 2));
    list_of_commands.push_back(read_parameter(slave_id, 0, 9, 2));
    list_of_commands.push_back(read_parameter(slave_id, 0, 10, 2));
    list_of_commands.push_back(read_parameter(slave_id, 0, 11, 2));
    list_of_commands.push_back(read_parameter(slave_id, 0, 12, 2));
    list_of_commands.push_back(read_parameter(slave_id, 0, 13, 2));
    list_of_commands.push_back(read_parameter(slave_id, 0, 14, 2));
    list_of_commands.push_back(read_parameter(slave_id, 0, 15, 2));
    list_of_commands.push_back(read_parameter(slave_id, 0, 28, 2));
    return list_of_commands;
}
std::pair<int, int> LCDA630P_Modbus_RTU::parseModbusResponse(const std::vector<uint8_t> &response, bool print)
{
    if (response.size() < 7) {
        throw std::runtime_error("Invalid Modbus response: too short");
    }

    // Extract first word (0x2d15)
    int word1 = (response[3] << 8) | response[4];

    // Extract second word (0x0000)
    int word2 = (response[5] << 8) | response[6];

    if (print)
        std::cout << "param number " << std::dec << response.at(3) << " word 1 : " << std::dec << word1 << " word 2 : " << std::dec << word2 << std::endl;

    return {word1, word2};
};
uint16_t LCDA630P_Modbus_RTU::crcValueCalc(const uint8_t *data, uint16_t length)
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
#include "LichuanMotion.hpp"

LichuanMotion::LichuanMotion()
{
    DEBUG_SERIAL_PRINTLN("LichuanMotion instance declared");
}

void LichuanMotion::debug_print_frame(std::vector<uint8_t> frame, bool print)
{
    if (print)
    {
        for (int i = 0; i < frame.size(); i++)
        {
            std::stringstream ss;
            ss << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(frame[i]) << " ";
            DEBUG_SERIAL_PRINT(ss.str().c_str());
        }
        DEBUG_SERIAL_PRINTLN("");
    }
}
std::vector<int32_t> LichuanMotion::processListOfCommands(std::vector<std::vector<uint8_t>> &listOfCommands, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<int32_t> values;
    for (std::vector<uint8_t> command : listOfCommands)
    {
        std::vector<uint8_t> feedback = sendFunction(command) ;
        int32_t response = parseModbusResponse(feedback) ;
        values.push_back(response);
    }
    return values ;
}

std::vector<uint8_t> LichuanMotion::read_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, uint8_t size, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)>> sendFunction)
{
    std::vector<uint8_t> frame;
    frame.push_back(slave_id); // ADR
    frame.push_back(0x03);     // Read Holding Register
    frame.push_back(group_number);
    frame.push_back(parameter_offset);
    frame.push_back(0 >> 8);
    frame.push_back(size & 0xFF);

    // Calculate CRC using uint8_t data
    uint16_t crc = crcValueCalc(frame.data(), frame.size());
    frame.push_back(crc & 0xFF);        // Low byte
    frame.push_back((crc >> 8) & 0xFF); // High byte

#if DEBUG_SERIAL
    debug_print_frame(frame, true);
#endif
    if (sendFunction.has_value()) {
        std::vector<uint8_t> feedback = sendFunction.value()(frame) ;
        parseModbusResponse(feedback) ;
    }
    return frame;
}

std::vector<uint8_t> LichuanMotion::read_parameter(uint8_t slave_id, uint16_t address, uint16_t size, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)>> sendFunction)
{
    return read_parameter(slave_id, address >> 8, address & 0xFF, size);
}

std::vector<uint8_t> LichuanMotion::write_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int16_t value, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)>> sendFunction)
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
    debug_print_frame(frame, true);
#endif

    // If sendFunction is provided, send the frame and process the response
    if (sendFunction.has_value()) {
        std::vector<uint8_t> feedback = sendFunction.value()(frame) ;
        parseModbusResponse(feedback) ;
    }

    return frame;
}
std::vector<uint8_t> LichuanMotion::write_parameter(uint8_t slave_id, uint16_t address, int16_t value, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)>> sendFunction)
{
    return write_parameter(slave_id, address >> 8, address & 0xFF, value);
}

std::vector<uint8_t> LichuanMotion::write_parameter_32(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int32_t value, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)>> sendFunction)
{
    std::vector<uint8_t> frame;
    frame.push_back(slave_id); // ADR
    frame.push_back(0x10);     // Write Holding Register
    frame.push_back(group_number);
    frame.push_back(parameter_offset);
    frame.push_back(0x00); //The high 8 bits of the function code are M(H), and the length of a 32-bit function code is 2.
    frame.push_back(0x02); //Function code number lower 8 digits M(L)
    frame.push_back(0x04); //The number of function codes corresponds ti the number of bytes M*2. For examole, if P05-07 is written alone, DATA[4] is P04
    if (lower16_bit_first)
    {
        frame.push_back((value >> 8) & 0xFF); //Write the high 8 bits of the start function code, hex
        frame.push_back(value & 0xFF); //Write the lower 8 bits of the start function code, hex
        frame.push_back((value >> 24) & 0xFF);//Write the high 8 bits of the start function code group offset + 1, hex
        frame.push_back((value >> 16) & 0xFF);//Write the low 8 bits of the start function code group offset + 1, hex
    }else
    {
        frame.push_back((value >> 24) & 0xFF);//Write the high 8 bits of the start function code group offset + 1, hex
        frame.push_back((value >> 16) & 0xFF);//Write the low 8 bits of the start function code group offset + 1, hex
        frame.push_back((value >> 8) & 0xFF); //Write the high 8 bits of the start function code, hex
        frame.push_back(value & 0xFF); //Write the lower 8 bits of the start function code, hex
    }
    // Calculate CRC using uint8_t data
    uint16_t crc = crcValueCalc(frame.data(), frame.size());
    frame.push_back(crc & 0xFF);        // Low byte
    frame.push_back((crc >> 8) & 0xFF); // High byte
#if DEBUG_SERIAL
    debug_print_frame(frame, true);
#endif

    // If sendFunction is provided, send the frame and process the response
    if (sendFunction.has_value()) {
        std::vector<uint8_t> feedback = sendFunction.value()(frame) ;
        parseModbusResponse(feedback) ;
    }

    return frame;
};

std::vector<uint8_t> LichuanMotion::write_parameter_32(uint8_t slave_id, uint16_t address, int32_t value, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)>> sendFunction)
{
    return write_parameter_32(slave_id, address >> 8, address & 0xFF, value);
}

std::string LichuanMotion::vector_to_string(std::vector<uint8_t> frame)
{
    // Convert uint16_t array to string
    std::string request_string = "";

    for (int i = 0; i < frame.size() ; i++)
    {
        request_string += static_cast<char>(frame[i]);        // Get low byte
    }
    return request_string ;
};
int32_t LichuanMotion::parseModbusResponse(const std::vector<uint8_t> &response)
{
    // Extract first word (0x2d15)
    int32_t value = 0;
    if (response.size() < 7) {
        return 0xFFFFFFFF ;
    }
    else if (lower16_bit_first && response.size() > 8 )
    {
        converter.as_uint8[1] = response[3];
        converter.as_uint8[0] = response[4];
        converter.as_uint8[3] = response[5];
        converter.as_uint8[2] = response[6];
        value = converter.as_int32[0];
#if DEBUG_SERIAL
    DEBUG_SERIAL_PRINTLN("lower16_bit_first && response.size() > 8");
        std::stringstream ss ;
        ss << std::hex << std::setfill('0') << std::setw(2) << "adr: " << static_cast<int>(response[0]) << "\tf :" <<
            static_cast<int>(response[1]) << "\tsize: " << std::dec << std::setw(2) << response[2] <<
            "\tvalue : " << value << "\t hex: " << std::hex << std::setfill('0') << std::setw(2) << "0x" <<
            static_cast<int>(value) << std::endl;
        DEBUG_SERIAL_PRINT(ss.str().c_str());
#endif
    }
    else if (!lower16_bit_first && response.size() > 8 )
    {
        converter.as_uint8[0] = response[3];
        converter.as_uint8[1] = response[4];
        converter.as_uint8[2] = response[5];
        converter.as_uint8[3] = response[6];
        value = converter.as_int32[0];
#if DEBUG_SERIAL
        DEBUG_SERIAL_PRINTLN("!lower16_bit_first && response.size() > 8 ");
        std::stringstream ss ;
        ss << std::hex << std::setfill('0') << std::setw(2) << "adr: " << static_cast<int>(response[0]) << "\tf :" <<
            static_cast<int>(response[1]) << "\tp" << static_cast<int>(response[2]) << "-" <<
            static_cast<int>(response[3]) << "\tsize: " << std::dec << std::setw(2) << response.size() <<
            "\tvalue : " << value << "\t hex: " << std::hex << std::setfill('0') << std::setw(2) << "0x" <<
            static_cast<int>(value) << std::endl;
        DEBUG_SERIAL_PRINT(ss.str().c_str());
#endif
    }else
    {
        value = static_cast<int16_t>(response[3] << 8 ) | response[4];
#if DEBUG_SERIAL
        DEBUG_SERIAL_PRINTLN("response.size() <= 8 ");
        std::stringstream ss ;
        ss << std::hex << std::setfill('0') << std::setw(2) << "adr: " << static_cast<int>(response[0]) << "\tf :" <<
            static_cast<int>(response[1]) << "\tsize: " << std::dec << std::setw(2) << static_cast<int>(response[2]) <<
            "\tvalue : " << value << "\t hex: " << std::hex << std::setfill('0') << std::setw(2) << "0x" <<
            static_cast<int>(value) << std::endl;
        DEBUG_SERIAL_PRINT(ss.str().c_str());
#endif
    }

    return value ;
};
uint16_t LichuanMotion::crcValueCalc(const uint8_t *data, uint16_t length)
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
bool LichuanMotion::controledOverModbus()
{
    return controlOverModbus == 1 ;
};
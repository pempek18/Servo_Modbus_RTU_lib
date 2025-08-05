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
    int16_t address = group_number << 8 | parameter_offset;
    MB::ModbusRequest request(slave_id, MB::utils::ReadAnalogOutputHoldingRegisters, address, size);
    
    std::vector<uint8_t> frame = request.toRaw();


    // Calculate CRC using uint8_t data
    uint16_t CRC = MB::utils::calculateCRC(frame);
    auto CRCptr  = reinterpret_cast<uint8_t *>(&CRC);
    frame.insert(frame.end(), CRCptr, CRCptr + 2);

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
    return read_parameter(slave_id, address >> 8, address & 0xFF, size, sendFunction);
}

std::vector<uint8_t> LichuanMotion::write_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int16_t value, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)>> sendFunction)
{
    int16_t address = group_number << 8 | parameter_offset;
    std::vector<MB::ModbusCell> modbusValues;
    modbusValues.emplace_back(static_cast<uint16_t>(value));
    MB::ModbusRequest request(slave_id, MB::utils::WriteSingleAnalogOutputRegister, address, 1, modbusValues);
    std::vector<uint8_t> frame = request.toRaw();

    // Calculate CRC using uint8_t data
    uint16_t CRC = MB::utils::calculateCRC(frame);
    auto CRCptr  = reinterpret_cast<uint8_t *>(&CRC);
    frame.insert(frame.end(), CRCptr, CRCptr + 2);
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
    std::vector<MB::ModbusCell> modbusValues;
    modbusValues.emplace_back(static_cast<uint16_t>(value));
    MB::ModbusRequest request(slave_id, MB::utils::WriteSingleAnalogOutputRegister, address, 1, modbusValues);
    std::vector<uint8_t> frame = request.toRaw();

    // Calculate CRC using uint8_t data
    uint16_t CRC = MB::utils::calculateCRC(frame);
    auto CRCptr  = reinterpret_cast<uint8_t *>(&CRC);
    frame.insert(frame.end(), CRCptr, CRCptr + 2);
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

std::vector<uint8_t> LichuanMotion::write_parameter_32(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int32_t value, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)>> sendFunction)
{
    std::vector<uint8_t> values ;
    if (lower16_bit_first)
    {
        values.push_back((value >> 8) & 0xFF); //Write the high 8 bits of the start function code, hex
        values.push_back(value & 0xFF); //Write the lower 8 bits of the start function code, hex
        values.push_back((value >> 24) & 0xFF);//Write the high 8 bits of the start function code group offset + 1, hex
        values.push_back((value >> 16) & 0xFF);//Write the low 8 bits of the start function code group offset + 1, hex
    }else
    {
        values.push_back((value >> 24) & 0xFF);//Write the high 8 bits of the start function code group offset + 1, hex
        values.push_back((value >> 16) & 0xFF);//Write the low 8 bits of the start function code group offset + 1, hex
        values.push_back((value >> 8) & 0xFF); //Write the high 8 bits of the start function code, hex
        values.push_back(value & 0xFF); //Write the lower 8 bits of the start function code, hex
    }
    
    // Convert uint8_t values to ModbusCell values (2 registers = 4 bytes)
    std::vector<MB::ModbusCell> modbusValues;
    for (size_t i = 0; i < values.size(); i += 2) {
        uint16_t regValue = (static_cast<uint16_t>(values[i]) << 8) | values[i + 1];
        modbusValues.emplace_back(regValue);
    }
    int16_t address = group_number << 8 | parameter_offset;
    MB::ModbusRequest request(slave_id, MB::utils::WriteMultipleAnalogOutputHoldingRegisters, address, 2, modbusValues);
    std::vector<uint8_t> frame = request.toRaw();

    // Calculate CRC using uint8_t data
    uint16_t CRC = MB::utils::calculateCRC(frame);
    auto CRCptr  = reinterpret_cast<uint8_t *>(&CRC);
    frame.insert(frame.end(), CRCptr, CRCptr + 2);


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

    MB::ModbusRequest request = MB::ModbusRequest::fromRaw(frame);
    request_string = request.toString(); 
    return request_string ;
};
int32_t LichuanMotion::parseModbusResponse(const std::vector<uint8_t> &response)
{
    MB::ModbusResponse rsp = MB::ModbusResponse::fromRaw(response);
    std::stringstream ss;
    ss << "Function Type: " << rsp.functionType() << std::endl;
    ss << "Function Code: " << rsp.functionCode() << std::endl;
    ss << "Slave ID: " << rsp.slaveID() << std::endl;
    ss << "Register Address: " << rsp.registerAddress() << std::endl;
    ss << "Register Count: " << rsp.registerValues().size() << std::endl;
    ss << "Register Values: " << rsp.registerValues()[0].reg() << std::endl;
    std::cout << ss.str() << std::endl;
    return rsp.registerValues()[0].reg();
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
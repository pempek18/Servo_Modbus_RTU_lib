#include "LCDA630P_Modbus_RTU.hpp"

LCDA630P_Modbus_RTU::LCDA630P_Modbus_RTU()
{
    DEBUG_SERIAL_PRINTLN("Class declared");
};
void LCDA630P_Modbus_RTU::scan_devices()
{
    DEBUG_SERIAL_PRINTLN("Scanning devices");
};
void LCDA630P_Modbus_RTU::debug_print_frame(std::vector<uint8_t> frame, bool print)
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
std::vector<int32_t> LCDA630P_Modbus_RTU::processListoOfCommands(std::vector<std::vector<uint8_t>> &listOfCommands, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
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
std::vector<uint8_t> LCDA630P_Modbus_RTU::read_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, uint8_t size)
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
    return frame;
}
std::vector<uint8_t> LCDA630P_Modbus_RTU::read_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction, uint8_t size)
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

    std::vector<int32_t> values ;
    std::vector<uint8_t> feedback = sendFunction(frame) ;
    int32_t response = parseModbusResponse(feedback) ; 
    values.push_back(response);
    return frame;
}
std::vector<uint8_t> LCDA630P_Modbus_RTU::write_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int16_t value)
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
    return frame;
};
std::vector<uint8_t> LCDA630P_Modbus_RTU::write_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int16_t value, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
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

    std::vector<uint32_t> values ;
    std::vector<uint8_t> feedback = sendFunction(frame) ;
    uint32_t response = parseModbusResponse(feedback) ; 
    values.push_back(response);
    return frame;
};
std::vector<uint8_t> LCDA630P_Modbus_RTU::write_parameter_32(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int32_t value)
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
    return frame;    
}
std::vector<uint8_t> LCDA630P_Modbus_RTU::write_parameter_32(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int32_t value, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
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

    std::vector<uint32_t> values ;
    std::vector<uint8_t> feedback = sendFunction(frame) ;
    uint32_t response = parseModbusResponse(feedback) ; 
    values.push_back(response);

    return frame;    
}
std::vector<std::vector<uint8_t>> LCDA630P_Modbus_RTU::read_servo_brief(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    list_of_commands.push_back(read_parameter(slave_id, 0, 0));
    list_of_commands.push_back(read_parameter(slave_id, 0, 9));
    list_of_commands.push_back(read_parameter(slave_id, 0, 10));
    list_of_commands.push_back(read_parameter(slave_id, 0, 11));
    list_of_commands.push_back(read_parameter(slave_id, 0, 12));
    list_of_commands.push_back(read_parameter(slave_id, 0, 13));
    list_of_commands.push_back(read_parameter(slave_id, 0, 14));
    list_of_commands.push_back(read_parameter(slave_id, 0, 15));
    list_of_commands.push_back(read_parameter(slave_id, 0, 28));
    list_of_commands.push_back(read_parameter(slave_id, 5, 0));
    list_of_commands.push_back(read_parameter(slave_id, 12, 26));
    std::vector<int32_t> values ;
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************")
    values = processListoOfCommands(list_of_commands, sendFunction);      
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************")
    MotorNumber = values.at(0) ; 
    RatedVoltage = values.at(1) ;
    RatedPower = values.at(2);
    RatedCurrent = values.at(3);
    RatedTorque = values.at(4);
    MaxTorque = values.at(5);
    RatedSpeed = values.at(6);
    MaxSpeed = values.at(7);
    PositionOffsetOfAbsolutEncoder = values.at(8);
    controlOverModbus = values.at(9);
    lower16_bit_first = values.at(10);

    return list_of_commands;
}
int64_t LCDA630P_Modbus_RTU::get_actual_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    list_of_commands.push_back(read_parameter(slave_id, 11, 77, 9));
    list_of_commands.push_back(read_parameter(slave_id, 11, 79, 9));
    std::vector<int32_t> values ;
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************")
    values = processListoOfCommands(list_of_commands, sendFunction);         
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************")
    ActualAbsolutePosition = (static_cast<int64_t>(static_cast<uint32_t>(values[1])) << 32) | static_cast<uint32_t>(values[0]);
    return ActualAbsolutePosition;
}
int16_t LCDA630P_Modbus_RTU::get_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    list_of_commands.push_back(read_parameter(slave_id, 11, 55));
    std::vector<int32_t> values ;
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************")
    values = processListoOfCommands(list_of_commands, sendFunction);        
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************")
    ActualSpeedRpm = values[0];
    return ActualSpeedRpm;
}
std::vector<std::vector<uint8_t>> LCDA630P_Modbus_RTU::raw_one_rotation(uint8_t slave_id)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    std::vector<uint8_t> frame;
    DEBUG_SERIAL_PRINTLN("*****************Move one rotation RAW DATA*****************");
    frame.push_back(slave_id);
    frame.push_back(0x06);
    frame.push_back(0x17);
    frame.push_back(0x00);
    frame.push_back(0x00);
    frame.push_back(0x01);
    frame.push_back(0x4D);
    frame.push_back(0xBE);
    list_of_commands.push_back(frame);
#if DEBUG_SERIAL
    debug_print_frame(frame, true);
#endif    
    frame.clear();
    frame.push_back(slave_id);
    frame.push_back(0x06);
    frame.push_back(0x17);
    frame.push_back(0x02);
    frame.push_back(0x00);
    frame.push_back(0x1C);
    frame.push_back(0x2C);
    frame.push_back(0x77);
    list_of_commands.push_back(frame);
#if DEBUG_SERIAL
    debug_print_frame(frame, true);
#endif      
    frame.clear();
    frame.push_back(slave_id);
    frame.push_back(0x06);
    frame.push_back(0x02);
    frame.push_back(0x00);
    frame.push_back(0x00);
    frame.push_back(0x01);
    frame.push_back(0x49);
    frame.push_back(0xB2);
    list_of_commands.push_back(frame);   
#if DEBUG_SERIAL
    debug_print_frame(frame, true);
#endif       
    frame.clear();
    frame.push_back(slave_id);
    frame.push_back(0x06);
    frame.push_back(0x05);
    frame.push_back(0x00);
    frame.push_back(0x00);
    frame.push_back(0x02);
    frame.push_back(0x08);
    frame.push_back(0xC7);
    list_of_commands.push_back(frame); 
#if DEBUG_SERIAL
    debug_print_frame(frame, true);
#endif      
    frame.clear();
    frame.push_back(slave_id);
    frame.push_back(0x06);
    frame.push_back(0x11);
    frame.push_back(0x00);
    frame.push_back(0x00);
    frame.push_back(0x03);
    frame.push_back(0xCC);
    frame.push_back(0xF7);
    list_of_commands.push_back(frame);  
#if DEBUG_SERIAL
    debug_print_frame(frame, true);
#endif         
    frame.clear();
    frame.push_back(slave_id);
    frame.push_back(0x10);
    frame.push_back(0x11);
    frame.push_back(0x0C);
    frame.push_back(0x00);
    frame.push_back(0x02);
    frame.push_back(0x04);
    frame.push_back(0x27);
    frame.push_back(0x10);
    frame.push_back(0x00);
    frame.push_back(0x00);
    frame.push_back(0x38);
    frame.push_back(0xDB);
    list_of_commands.push_back(frame); 
#if DEBUG_SERIAL
    debug_print_frame(frame, true);
#endif           
    frame.clear();
    frame.push_back(slave_id);
    frame.push_back(0x06);
    frame.push_back(0x31);
    frame.push_back(0x00);
    frame.push_back(0x00);
    frame.push_back(0x01);
    frame.push_back(0x46);
    frame.push_back(0xF6);
    list_of_commands.push_back(frame);  
#if DEBUG_SERIAL
    debug_print_frame(frame, true);
#endif      
    frame.clear();
    frame.push_back(slave_id);
    frame.push_back(0x06);
    frame.push_back(0x31);
    frame.push_back(0x00);
    frame.push_back(0x00);
    frame.push_back(0x03);
    frame.push_back(0xC7);
    frame.push_back(0x37);
    list_of_commands.push_back(frame);  
#if DEBUG_SERIAL
    debug_print_frame(frame, true);
#endif              
    DEBUG_SERIAL_PRINTLN("*****************Move one rotation RAW DATA*****************");
    return list_of_commands ;
}
std::vector<std::vector<uint8_t>> LCDA630P_Modbus_RTU::moveRelative(uint8_t slave_id, int32_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (!controlOverModbus)
        return list_of_commands;
    list_of_commands.push_back(write_parameter_32(1,0x11,0x0C,position));//Multi-segment location operation mode Sequential operation (P11-01 for selection of segment number)
    list_of_commands.push_back(write_parameter(1,0x31,0,1));//Communication given VDI virtual level 0～65535
    list_of_commands.push_back(write_parameter(1,0x31,0,3));//Communication given VDI virtual level 0～65535
    DEBUG_SERIAL_PRINTLN("*****************Move to pos*****************");
    processListoOfCommands(list_of_commands, sendFunction); 
    DEBUG_SERIAL_PRINTLN("*****************Move to pos*****************");
    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA630P_Modbus_RTU::moveVelocity(uint8_t slave_id, int32_t speed, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (!controlOverModbus)
        return list_of_commands;
    list_of_commands.push_back(write_parameter(1,0x2,0,0));//Control Mode Selectio 0: speed mod
    list_of_commands.push_back(write_parameter(1,0x17,0,1));//VDI1 Terminal function selection
    list_of_commands.push_back(write_parameter(1,0x31,0,1));//Communication given VDI virtual level 0～65535 16 bit input register 
    list_of_commands.push_back(write_parameter(1,0x06,0x03,speed));//Multi-segment location operation mode Sequential operation (P11-01 for selection of segment number)
    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    processListoOfCommands(list_of_commands, sendFunction); 
    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA630P_Modbus_RTU::set_torque(uint8_t slave_id, int32_t torque, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    list_of_commands.push_back(write_parameter(1,0x07,9,torque));
    list_of_commands.push_back(write_parameter(1,0x07,10,torque));
    list_of_commands.push_back(write_parameter(1,0x07,11,torque));
    list_of_commands.push_back(write_parameter(1,0x07,12,torque));
    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    processListoOfCommands(list_of_commands, sendFunction); 
    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA630P_Modbus_RTU::config_for_modbus_control_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (controlOverModbus && eControlMode == Position)
        return list_of_commands ;
    list_of_commands.push_back(write_parameter(1,0x17,0,1));//VDI1 Terminal function selection
    list_of_commands.push_back(write_parameter(1,0x17,2,28));//VDI2 Terminal function selection
    list_of_commands.push_back(write_parameter(1,0x2,0,1));//Control Mode Selectio 1: position mod
    list_of_commands.push_back(write_parameter(1,0x5,0,2));//Control Mode Selectio 1: position mod
    list_of_commands.push_back(write_parameter(1,0x5,2,10000));//Location instruction source multi-segment position instruction give
    list_of_commands.push_back(write_parameter(1,0x11,0,2));//Multi-segment location operation mode Sequential operation (P11-01 for selection of segment number)
    eControlMode = Position ; 
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    processListoOfCommands(list_of_commands, sendFunction); 
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA630P_Modbus_RTU::config_for_modbus_control_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (controlOverModbus && eControlMode == Speed)
        return list_of_commands ;
    list_of_commands.push_back(write_parameter(1,0x17,0,1));//VDI1 Terminal function selection
    list_of_commands.push_back(write_parameter(1,0x17,2,28));//VDI2 Terminal function selection
    list_of_commands.push_back(write_parameter(1,0x17,3,26));//VDI3 Terminal function selection
    list_of_commands.push_back(write_parameter(1,0x2,0,0));//Control Mode Selectio 0: speed mod
    list_of_commands.push_back(write_parameter(1,0x6,0,0));//Location instruction source multi-segment position instruction give
    list_of_commands.push_back(write_parameter(1,0x6,1,5));//Location instruction source multi-segment position instruction give
    list_of_commands.push_back(write_parameter(1,0x6,2,0));//Location instruction source multi-segment position instruction give
    list_of_commands.push_back(write_parameter(1,0x11,0,3));//Multi-segment location operation mode Sequential operation (P11-01 for selection of segment number)
    eControlMode = Speed ; 
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    processListoOfCommands(list_of_commands, sendFunction); 
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    return list_of_commands;
}
bool LCDA630P_Modbus_RTU::disable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    DEBUG_SERIAL_PRINTLN("*****************Disable*****************");
    std::vector<uint8_t> command = write_parameter(1,0x31,0,0);
    std::vector<uint8_t> response = sendFunction(command) ;
    bool value = parseModbusResponse(response);
    DEBUG_SERIAL_PRINTLN("*****************Disable*****************");
    return value;
}
std::string LCDA630P_Modbus_RTU::vector_to_string(std::vector<uint8_t> frame)
{
    // Convert uint16_t array to string
    std::string request_string = "";

    for (int i = 0; i < frame.size() ; i++)
    {
        request_string += static_cast<char>(frame[i]);        // Get low byte
    }
    return request_string ;
}
int32_t LCDA630P_Modbus_RTU::parseModbusResponse(const std::vector<uint8_t> &response)
{
    // Extract first word (0x2d15)
    int32_t value = 0;
    if (response.size() < 7) {
        throw std::runtime_error("Invalid Modbus response: too short");
    }  
    else if (lower16_bit_first && response.size() > 8 )
    {
        // value = (static_cast<int32_t>(response[5]) << 0) |
        //         (static_cast<int32_t>(response[6]) << 8) |
        //         (static_cast<int32_t>(response[7]) << 16)  |
        //         (static_cast<int32_t>(response[8]) << 24);        
        value = (static_cast<int32_t>(response[3]) << 8) |
                (static_cast<int32_t>(response[4]) << 0) |
                (static_cast<int32_t>(response[5]) << 24)  |
                (static_cast<int32_t>(response[6]) << 16);  
#if DEBUG_SERIAL
    DEBUG_SERIAL_PRINTLN("lower16_bit_first && response.size() > 8");
        std::stringstream ss ;
        ss << std::hex << std::setfill('0') << std::setw(2) << "adr: " << static_cast<int>(response[0]) << "\tf :" << 
            static_cast<int>(response[1]) << "\tp" << static_cast<int>(response[2]) << "-" << 
            static_cast<int>(response[3]) << "\tsize: " << std::dec << std::setw(2) << response.size() << 
            "\tvalue : " << value << "\t hex: " << std::hex << std::setfill('0') << std::setw(2) << "0x" << 
            static_cast<int>(value) << std::endl;
        DEBUG_SERIAL_PRINT(ss.str().c_str());
#endif
    }
    else if (!lower16_bit_first && response.size() > 8 )
    {
        value = (static_cast<int32_t>(response[7]) << 24) |
                (static_cast<int32_t>(response[8]) << 16) |
                (static_cast<int32_t>(response[9]) << 8)  |
                (static_cast<int32_t>(response[10]) << 0);        
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
}
bool LCDA630P_Modbus_RTU::controledOverModbus()
{
    return (controlOverModbus == 2);
};
#include "LCDA6_Modbus_RTU.hpp"

LCDA6_Modbus_RTU::LCDA6_Modbus_RTU()
{
    DEBUG_SERIAL_PRINTLN("Class declared");
};
void LCDA6_Modbus_RTU::scan_devices()
{
    DEBUG_SERIAL_PRINTLN("Scanning devices");
};
void LCDA6_Modbus_RTU::debug_print_frame(std::vector<uint8_t> frame, bool print)
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
std::vector<int32_t> LCDA6_Modbus_RTU::processListoOfCommands(std::vector<std::vector<uint8_t>> &listOfCommands, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
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


std::vector<uint8_t> LCDA6_Modbus_RTU::read_parameter(uint8_t slave_id, uint16_t address, uint16_t size)
{
    std::vector<uint8_t> frame;
    frame.push_back(slave_id);          // ADR
    frame.push_back(0x03);              // Read Holding Register
    frame.push_back(address >> 8);      // AddrH - Higher 8 bits
    frame.push_back(address & 0xFF);    // AddrL - Lower 8 bits
    frame.push_back(size >> 8);
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
std::vector<uint8_t> LCDA6_Modbus_RTU::read_parameter(uint8_t slave_id, uint16_t address, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction, uint16_t size)
{
    std::vector<uint8_t> frame;
    frame.push_back(slave_id);          // ADR
    frame.push_back(0x03);              // Read Holding Register
    frame.push_back(address >> 8);      // AddrH - Higher 8 bits
    frame.push_back(address & 0xFF);    // AddrL - Lower 8 bits
    frame.push_back(size >> 8);
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
std::vector<uint8_t> LCDA6_Modbus_RTU::write_parameter(uint8_t slave_id, uint16_t address, int16_t value)
{
    std::vector<uint8_t> frame;
    frame.push_back(slave_id);          // ADR
    frame.push_back(0x06);              // Read Holding Register
    frame.push_back(address >> 8);      // AddrH - Higher 8 bits
    frame.push_back(address & 0xFF);    // AddrL - Lower 8 bits
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
std::vector<uint8_t> LCDA6_Modbus_RTU::write_parameter(uint8_t slave_id, uint16_t address, int16_t value, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
{
    std::vector<uint8_t> frame;
    frame.push_back(slave_id);          // ADR
    frame.push_back(0x06);              // Read Holding Register
    frame.push_back(address >> 8);      // AddrH - Higher 8 bits
    frame.push_back(address & 0xFF);    // AddrL - Lower 8 bits
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
std::vector<uint8_t> LCDA6_Modbus_RTU::write_parameter_32(uint8_t slave_id, uint16_t address, int32_t value)
{
    std::vector<uint8_t> frame;
    frame.push_back(slave_id);          // ADR
    frame.push_back(0x10);              // Write Holding Register
    frame.push_back(address >> 8);      // AddrH - Higher 8 bits
    frame.push_back(address & 0xFF);    // AddrL - Lower 8 bits
    frame.push_back(0x00);              // The high 8 bits of the function code are M(H), and the length of a 32-bit function code is 2.
    frame.push_back(0x02);              // Function code number lower 8 digits M(L)
    frame.push_back(0x04);              // The number of function codes corresponds ti the number of bytes M*2. For examole, if P05-07 is written alone, DATA[4] is P04
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
std::vector<uint8_t> LCDA6_Modbus_RTU::write_parameter_32(uint8_t slave_id, uint16_t address, int32_t value, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
{
    std::vector<uint8_t> frame;
    frame.push_back(slave_id);          // ADR
    frame.push_back(0x10);              // Write Holding Register
    frame.push_back(address >> 8);      // AddrH - Higher 8 bits
    frame.push_back(address & 0xFF);    // AddrL - Lower 8 bits
    frame.push_back(0x00);              // The high 8 bits of the function code are M(H), and the length of a 32-bit function code is 2.
    frame.push_back(0x02);              // Function code number lower 8 digits M(L)
    frame.push_back(0x04);              // The number of function codes corresponds ti the number of bytes M*2. For examole, if P05-07 is written alone, DATA[4] is P04
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

std::vector<std::vector<uint8_t>> LCDA6_Modbus_RTU::servo_config(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;

    int16_t DI_cfg = 1 << 0;                                         // Set Enable Bit
    list_of_commands.push_back(write_parameter(slave_id, 0x1A4, DI_cfg));
    list_of_commands.push_back(write_parameter(slave_id, 0x90, 1));  // Control Mode - 1 = Communication (extended)


    std::vector<int32_t> values ;
    DEBUG_SERIAL_PRINTLN("*****************Write Config*****************")
    values = processListoOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Write Config*****************")

    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA6_Modbus_RTU::read_servo_brief(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    list_of_commands.push_back(read_parameter(slave_id, 0x00)); // Slave ID
    list_of_commands.push_back(read_parameter(slave_id, 0x05)); // Control Mode
    list_of_commands.push_back(read_parameter(slave_id, 0x0D)); // Baud Rate

    std::vector<int32_t> values;
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************")
    values = processListoOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************")

    return list_of_commands;
}
// int64_t LCDA6_Modbus_RTU::get_actual_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
// {
//     std::vector<std::vector<uint8_t>> list_of_commands;
//     list_of_commands.push_back(read_parameter(slave_id, 11, 77, 9));
//     list_of_commands.push_back(read_parameter(slave_id, 11, 79, 9));
//     std::vector<int32_t> values ;
//     DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************")
//     values = processListoOfCommands(list_of_commands, sendFunction);
//     DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************")
//     ActualAbsolutePosition = (static_cast<int64_t>(static_cast<int32_t>(values[1])) << 32) | static_cast<int32_t>(values[0]);
//     return ActualAbsolutePosition;
// }
int16_t LCDA6_Modbus_RTU::get_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    // If DI is set as INTSPD                                               //TODO - leave for now
    std::vector<uint8_t> command = read_parameter(slave_id, 0x53);          // Get Speed value (RPM) from INTSPD0
    // if not used DI as INTSPD
    // std::vector<uint8_t> command = read_parameter(slave_id, 0x140);      //FIXME // Get Commanded speed to Internal Speed Command 0
    std::vector<uint8_t> response = sendFunction(command);

    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************")
    int32_t value = parseModbusResponse(response);
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************")
    return value;
}
// std::vector<std::vector<uint8_t>> LCDA6_Modbus_RTU::raw_one_rotation(uint8_t slave_id)
// {
//     std::vector<std::vector<uint8_t>> list_of_commands;
//     std::vector<uint8_t> frame;
//     DEBUG_SERIAL_PRINTLN("*****************Move one rotation RAW DATA*****************");
//     frame.push_back(slave_id);
//     frame.push_back(0x06);
//     frame.push_back(0x17);
//     frame.push_back(0x00);
//     frame.push_back(0x00);
//     frame.push_back(0x01);
//     frame.push_back(0x4D);
//     frame.push_back(0xBE);
//     list_of_commands.push_back(frame);
// #if DEBUG_SERIAL
//     debug_print_frame(frame, true);
// #endif
//     frame.clear();
//     frame.push_back(slave_id);
//     frame.push_back(0x06);
//     frame.push_back(0x17);
//     frame.push_back(0x02);
//     frame.push_back(0x00);
//     frame.push_back(0x1C);
//     frame.push_back(0x2C);
//     frame.push_back(0x77);
//     list_of_commands.push_back(frame);
// #if DEBUG_SERIAL
//     debug_print_frame(frame, true);
// #endif
//     frame.clear();
//     frame.push_back(slave_id);
//     frame.push_back(0x06);
//     frame.push_back(0x02);
//     frame.push_back(0x00);
//     frame.push_back(0x00);
//     frame.push_back(0x01);
//     frame.push_back(0x49);
//     frame.push_back(0xB2);
//     list_of_commands.push_back(frame);
// #if DEBUG_SERIAL
//     debug_print_frame(frame, true);
// #endif
//     frame.clear();
//     frame.push_back(slave_id);
//     frame.push_back(0x06);
//     frame.push_back(0x05);
//     frame.push_back(0x00);
//     frame.push_back(0x00);
//     frame.push_back(0x02);
//     frame.push_back(0x08);
//     frame.push_back(0xC7);
//     list_of_commands.push_back(frame);
// #if DEBUG_SERIAL
//     debug_print_frame(frame, true);
// #endif
//     frame.clear();
//     frame.push_back(slave_id);
//     frame.push_back(0x06);
//     frame.push_back(0x11);
//     frame.push_back(0x00);
//     frame.push_back(0x00);
//     frame.push_back(0x03);
//     frame.push_back(0xCC);
//     frame.push_back(0xF7);
//     list_of_commands.push_back(frame);
// #if DEBUG_SERIAL
//     debug_print_frame(frame, true);
// #endif
//     frame.clear();
//     frame.push_back(slave_id);
//     frame.push_back(0x10);
//     frame.push_back(0x11);
//     frame.push_back(0x0C);
//     frame.push_back(0x00);
//     frame.push_back(0x02);
//     frame.push_back(0x04);
//     frame.push_back(0x27);
//     frame.push_back(0x10);
//     frame.push_back(0x00);
//     frame.push_back(0x00);
//     frame.push_back(0x38);
//     frame.push_back(0xDB);
//     list_of_commands.push_back(frame);
// #if DEBUG_SERIAL
//     debug_print_frame(frame, true);
// #endif
//     frame.clear();
//     frame.push_back(slave_id);
//     frame.push_back(0x06);
//     frame.push_back(0x31);
//     frame.push_back(0x00);
//     frame.push_back(0x00);
//     frame.push_back(0x01);
//     frame.push_back(0x46);
//     frame.push_back(0xF6);
//     list_of_commands.push_back(frame);
// #if DEBUG_SERIAL
//     debug_print_frame(frame, true);
// #endif
//     frame.clear();
//     frame.push_back(slave_id);
//     frame.push_back(0x06);
//     frame.push_back(0x31);
//     frame.push_back(0x00);
//     frame.push_back(0x00);
//     frame.push_back(0x03);
//     frame.push_back(0xC7);
//     frame.push_back(0x37);
//     list_of_commands.push_back(frame);
// #if DEBUG_SERIAL
//     debug_print_frame(frame, true);
// #endif
//     DEBUG_SERIAL_PRINTLN("*****************Move one rotation RAW DATA*****************");
//     return list_of_commands ;
// }
// std::vector<std::vector<uint8_t>> LCDA6_Modbus_RTU::moveRelative(uint8_t slave_id, int32_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
// {
//     std::vector<std::vector<uint8_t>> list_of_commands ;
//     if (!controlOverModbus)
//         return list_of_commands;
//     list_of_commands.push_back(write_parameter_32(1,0x11,0x0C,position));//Multi-segment location operation mode Sequential operation (P11-01 for selection of segment number)
//     list_of_commands.push_back(write_parameter(1,0x31,0,1));//Communication given VDI virtual level 0～65535
//     list_of_commands.push_back(write_parameter(1,0x31,0,3));//Communication given VDI virtual level 0～65535
//     DEBUG_SERIAL_PRINTLN("*****************Move to pos*****************");
//     processListoOfCommands(list_of_commands, sendFunction);
//     DEBUG_SERIAL_PRINTLN("*****************Move to pos*****************");
//     return list_of_commands;
// }
// int64_t LCDA6_Modbus_RTU::moveAbsolute(uint8_t slave_id, int64_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
// {
//     config_for_modbus_control_position(1, sendFunction);
//     get_actual_position(slave_id, sendFunction);
//     int64_t diffToPos = (position - ActualAbsolutePosition);
//     std::stringstream ss ;
//         ss << "Absolute Position " << std::dec << ActualAbsolutePosition << std::endl ;
//         ss << "Setpoint Position " << std::dec << position << std::endl ;
//         ss << "Difference in position: " << std::dec << diffToPos << std::endl ;
//     float fDiffToPos = ((float)diffToPos / (float)encoder_resolution) * (float)pulse_per_rotation ;
//         ss << "Difference in position float: " << fDiffToPos << std::endl ;
//     diffToPos = (int64_t)fDiffToPos ;
//         ss << "Move absolut difference: " << std::dec << diffToPos << std::endl ;
//     DEBUG_SERIAL_PRINTLN(ss.str().c_str());
//     moveRelative(slave_id, (int32_t)diffToPos, sendFunction);
//     get_actual_position(slave_id, sendFunction);
//     return ActualAbsolutePosition ;
// }
std::vector<std::vector<uint8_t>> LCDA6_Modbus_RTU::moveVelocity(uint8_t slave_id, int32_t speed, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (!controlOverModbus)
        return list_of_commands;

    // If DI is set as INTSPD                                               //TODO - leave for now
    list_of_commands.push_back(write_parameter(slave_id, 0x53, speed));     // Set Speed value (RPM) into INTSPD0
    // if not used DI as INTSPD
    // list_of_commands.push_back(write_parameter(slave_id, 0x140, speed)); //FIXME // Set Commanded speed to Internal Speed Command 0
    // list_of_commands.push_back(write_parameter(slave_id, 0x92, 0));      //FIXME // Set Commanded speed to Internal Speed Command 0

    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    processListoOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    return list_of_commands;
}
// std::vector<std::vector<uint8_t>> LCDA6_Modbus_RTU::set_torque(uint8_t slave_id, int32_t torque, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
// {
//     std::vector<std::vector<uint8_t>> list_of_commands ;
//     list_of_commands.push_back(write_parameter(1,0x07,9,torque));
//     list_of_commands.push_back(write_parameter(1,0x07,10,torque));
//     list_of_commands.push_back(write_parameter(1,0x07,11,torque));
//     list_of_commands.push_back(write_parameter(1,0x07,12,torque));
//     DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
//     processListoOfCommands(list_of_commands, sendFunction);
//     DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
//     return list_of_commands;
// }
// std::vector<std::vector<uint8_t>> LCDA6_Modbus_RTU::config_for_modbus_control_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
// {
//     std::vector<std::vector<uint8_t>> list_of_commands ;
//     if (controlOverModbus && eControlMode == Position)
//         return list_of_commands ;
//     list_of_commands.push_back(write_parameter(1,0x17,0,1));//VDI1 Terminal function selection
//     list_of_commands.push_back(write_parameter(1,0x17,2,28));//VDI2 Terminal function selection
//     list_of_commands.push_back(write_parameter(1,0x2,0,1));//Control Mode Selectio 1: position mod
//     list_of_commands.push_back(write_parameter(1,0x5,0,2));//Control Mode Selectio 1: position mod
//     list_of_commands.push_back(write_parameter(1,0x5,2,10000));//Location instruction source multi-segment position instruction give
//     list_of_commands.push_back(write_parameter(1,0x11,0,2));//Multi-segment location operation mode Sequential operation (P11-01 for selection of segment number)
//     eControlMode = Position ;
//     DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
//     processListoOfCommands(list_of_commands, sendFunction);
//     DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
//     return list_of_commands;
// }
std::vector<std::vector<uint8_t>> LCDA6_Modbus_RTU::config_for_modbus_control_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    // if (controlOverModbus && eControlMode == Speed)
    //     return list_of_commands ;

    list_of_commands.push_back(write_parameter(slave_id, 0x02, 1));         // Speed Mode       - 1 = speed
    list_of_commands.push_back(write_parameter(slave_id, 0x90, 1));         // Control Mode     - 0 - Analog \ 1 = Communication (extended)

    list_of_commands.push_back(write_parameter(slave_id, 0x05, 3));         // Analog / Internal speed selector

    eControlMode = Speed ;
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    processListoOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    return list_of_commands;
}
bool LCDA6_Modbus_RTU::enable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    // The parameter 0x80 must equal to 0 to enable
    list_of_commands.push_back(write_parameter(slave_id, 0x1A4, 1));

    DEBUG_SERIAL_PRINTLN("*****************Enable*****************");
    processListoOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Enable*****************");

    return 0;
}
bool LCDA6_Modbus_RTU::disable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    // The parameter 0x80 must equal to 0 to disable
    list_of_commands.push_back(write_parameter(slave_id, 0x1A4, 0));

    DEBUG_SERIAL_PRINTLN("*****************Disable*****************");
    processListoOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Disable*****************");
    return 0;
}
std::string LCDA6_Modbus_RTU::vector_to_string(std::vector<uint8_t> frame)
{
    // Convert uint16_t array to string
    std::string request_string = "";

    for (int i = 0; i < frame.size() ; i++)
    {
        request_string += static_cast<char>(frame[i]);        // Get low byte
    }
    return request_string ;
}
int32_t LCDA6_Modbus_RTU::parseModbusResponse(const std::vector<uint8_t> &response)
{
    #define LOWER_16_FIRST

    #ifdef LOWER_16_FIRST
        #define LSB 8
        #define MSB 0
    #else
        #define LSB 0
        #define MSB 8
    #endif

    if (response.size() < 7) {
        return 0xFFFFFFFF ;
    }
    int16_t ID = static_cast<int>(response[0]);
    int16_t fun = static_cast<int>(response[1]);

    std::stringstream ss ;
    ss << std::hex << std::setfill('0') << std::setw(2)
        << "ID: " << ID
        << "\tf :" << fun;

    switch (fun) {
        case 0x03:{
            int16_t len = static_cast<int16_t>(response[2]);
            ss << "\tbytes: " << std::dec << len;

            int16_t val;
            for(int i = 0; i < len; i+=2){
                val = (static_cast<int16_t>(response[3+2*i]) << LSB)  | (static_cast<int16_t>(response[4+2*i]) << MSB);
                ss << "\n\tvalue: "<< val
                   << "\thex: "  << std::hex << "0x" << std::setfill('0') << std::setw(2) << static_cast<int>(val);
            }
            ss << std::endl;
            DEBUG_SERIAL_PRINT(ss.str().c_str());
            return val;
        }
        case 0x06:{
            int16_t addr = (static_cast<int16_t>(response[2]) << LSB)  | (static_cast<int16_t>(response[3]) << MSB);
            ss << "\taddr: "<< std::hex << "0x" << std::setfill('0') << std::setw(2) << static_cast<int>(addr);

            int16_t val =  (static_cast<int16_t>(response[4]) << LSB)  | (static_cast<int16_t>(response[5]) << MSB);
            ss << "\tvalue: "<< std::dec << val
               << "\thex: "  << std::hex << "0x" << std::setfill('0') << std::setw(2) << static_cast<int>(val)
               << std::endl;
            DEBUG_SERIAL_PRINT(ss.str().c_str());
            return val;
        }
        case 0x10: //TODO
        default:
            return 0xFFFFFFFF;
            break;
    }
    return 0xFFFFFFFF;
};


uint16_t LCDA6_Modbus_RTU::crcValueCalc(const uint8_t *data, uint16_t length)
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
// bool LCDA6_Modbus_RTU::controledOverModbus()
// {
//     return (controlOverModbus == 2);
// };
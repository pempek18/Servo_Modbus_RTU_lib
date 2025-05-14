#include "LICHUAN_LCDA6.hpp"

LICHUAN_LCDA6::LICHUAN_LCDA6(const std::vector<uint8_t> &sendFunction) : I_Lichuan_RTU(sendFunction) {
    DEBUG_SERIAL_PRINTLN("Class declared");
    controlOverModbus = true;
}

std::vector<std::vector<uint8_t>> LICHUAN_LCDA6::servo_config() {
    // Make sure this servo will use modbus
    controlOverModbus = true;

    std::vector<std::vector<uint8_t>> list_of_commands;

    int16_t DI_cfg = 1 << 0;                                            // Set Enable Bit
    list_of_commands.push_back(write_parameter(0x1A0, DI_cfg));

    // DI configuration SPEED MODE
    list_of_commands.push_back(write_parameter(0x80, 0));
    list_of_commands.push_back(write_parameter(0x81, 1));
    list_of_commands.push_back(write_parameter(0x82, 2));
    list_of_commands.push_back(write_parameter(0x83, 3));
    list_of_commands.push_back(write_parameter(0x84, 5));
    list_of_commands.push_back(write_parameter(0x85, 7));
    list_of_commands.push_back(write_parameter(0x86, 11));
    list_of_commands.push_back(write_parameter(0x87, 12));
    // DO configuration SPEED MODE
    list_of_commands.push_back(write_parameter(0x88, 0));
    list_of_commands.push_back(write_parameter(0x89, 1));
    list_of_commands.push_back(write_parameter(0x8A, 7));
    list_of_commands.push_back(write_parameter(0x8B, 3));
    list_of_commands.push_back(write_parameter(0x8C, 4));
    list_of_commands.push_back(write_parameter(0x8D, 5));

    list_of_commands.push_back(write_parameter(0x90,  1));    // Control Mode - 1 = Communication (extended)


    std::vector<int32_t> values ;
    DEBUG_SERIAL_PRINTLN("*****************Write Config*****************")
    values = processListoOfCommands(list_of_commands);
    DEBUG_SERIAL_PRINTLN("*****************Write Config*****************")

    return list_of_commands;
}

std::vector<std::vector<uint8_t>> LICHUAN_LCDA6::read_servo_brief(){
    std::vector<std::vector<uint8_t>> list_of_commands;
    list_of_commands.push_back(read_parameter(slave_id, 0x00)); // Slave ID
    list_of_commands.push_back(read_parameter(slave_id, 0x05)); // Control Mode
    list_of_commands.push_back(read_parameter(slave_id, 0x0D)); // Baud Rate

    std::vector<int32_t> values;
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************")
    values = processListoOfCommands(list_of_commands);
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************")

    return list_of_commands;
}


bool LICHUAN_LCDA6::enable(){
    std::vector<std::vector<uint8_t>> list_of_commands;
    // The parameter 0x80 must equal to 1 to enable
    list_of_commands.push_back(write_parameter(0x1A4, 1));

    DEBUG_SERIAL_PRINTLN("*****************Enable*****************");
    processListoOfCommands(list_of_commands);
    DEBUG_SERIAL_PRINTLN("*****************Enable*****************");

    return 0;
}

bool LICHUAN_LCDA6::disable(){
    std::vector<std::vector<uint8_t>> list_of_commands;
    // The parameter 0x80 must equal to 0 to disable
    list_of_commands.push_back(write_parameter(0x1A4, 0));

    DEBUG_SERIAL_PRINTLN("*****************Disable*****************");
    processListoOfCommands(list_of_commands);
    DEBUG_SERIAL_PRINTLN("*****************Disable*****************");
    return 0;
}


std::vector<std::vector<uint8_t>> LICHUAN_LCDA6::config_for_modbus_control_speed(){
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (controlOverModbus && getMode() == Speed)
        return list_of_commands ;

    list_of_commands.push_back(write_parameter(0x02, 1));         // Speed Mode       - 1 = speed
    list_of_commands.push_back(write_parameter(0x90, 1));         // Control Mode     - 0 - Analog \ 1 = Communication (extended)

    list_of_commands.push_back(write_parameter(0x05, 3));         // Analog / Internal speed selector
    list_of_commands.push_back(write_parameter(0x1A7, 0x0801));   // Save parameters

    setMode(Speed);

    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    processListoOfCommands(list_of_commands);
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    return list_of_commands;
}

int16_t LICHUAN_LCDA6::get_actual_speed(){
    // If DI is set as INTSPD                                               //TODO - leave for now, due to errors from past
    std::vector<uint8_t> command = read_parameter(slave_id, 0x53);          // Get Speed value (RPM) from INTSPD0
    // if not used DI as INTSPD
    // std::vector<uint8_t> command = read_parameter(slave_id, 0x140);      //FIXME // Get Commanded speed to Internal Speed Command 0
    std::vector<uint8_t> response = sendFunction(command);

    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************")
    int32_t value = parseModbusResponse(response);
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************")
    return value;
}

std::vector<std::vector<uint8_t>> LICHUAN_LCDA6::set_speed(int16_t speed){
    std::vector<std::vector<uint8_t>> list_of_commands;
    // redundant method?
    return list_of_commands;
}

std::vector<std::vector<uint8_t>> LICHUAN_LCDA6::move_velocity(int16_t speed){
    std::vector<std::vector<uint8_t>> list_of_commands;
    if (!controlOverModbus)
        return list_of_commands;

    //FIXME - Leave both for now, due to errors from past
    // If DI is set as INTSPD
    list_of_commands.push_back(write_parameter(0x53, speed));     // Set Speed value (RPM) into INTSPD0
    // if not used DI as INTSPD
    list_of_commands.push_back(write_parameter(0x140, speed));    // Set Commanded speed to Internal Speed Command 0
    list_of_commands.push_back(write_parameter(0x92, 0));         // Set Commanded speed to Internal Speed Command 0

    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    processListoOfCommands(list_of_commands);
    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    return list_of_commands;
}


std::vector<std::vector<uint8_t>> LICHUAN_LCDA6::config_for_modbus_control_position(){
    //TODO
    std::vector<std::vector<uint8_t>> list_of_commands;
    return list_of_commands;
}

int64_t LICHUAN_LCDA6::get_actual_position(){
//TODO
return 0;
}

std::vector<std::vector<uint8_t>> LICHUAN_LCDA6::set_position(int64_t position){
    //TODO
    std::vector<std::vector<uint8_t>> list_of_commands;
    return list_of_commands;
}

std::vector<std::vector<uint8_t>> LICHUAN_LCDA6::move_relative(int64_t position){
    //TODO
    std::vector<std::vector<uint8_t>> list_of_commands;
    return list_of_commands;
}

std::vector<std::vector<uint8_t>> LICHUAN_LCDA6::move_absolute(int64_t position){
    //TODO
    std::vector<std::vector<uint8_t>> list_of_commands;
    return list_of_commands;
}


std::vector<std::vector<uint8_t>> LICHUAN_LCDA6::config_for_modbus_control_torque(){
    //TODO
    std::vector<std::vector<uint8_t>> list_of_commands;
    return list_of_commands;
}

int16_t LICHUAN_LCDA6::get_actual_torque(){
    //TODO
    return 0;
}

std::vector<std::vector<uint8_t>> LICHUAN_LCDA6::set_torque(int16_t speed){
    //TODO
    std::vector<std::vector<uint8_t>> list_of_commands;
    return list_of_commands;
}


std::vector<std::vector<uint8_t>> LICHUAN_LCDA6::raw_one_rotation(){
    //TODO
    std::vector<std::vector<uint8_t>> list_of_commands;
    return list_of_commands;
}
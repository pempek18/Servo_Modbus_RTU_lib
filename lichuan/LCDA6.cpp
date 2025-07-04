#include "LCDA6.hpp"

LCDA6::LCDA6()
{
    DEBUG_SERIAL_PRINTLN("Class declared");
}
LCDA6::~LCDA6()
{
    DEBUG_SERIAL_PRINTLN("Class destroyed");
};
std::vector<std::vector<uint8_t>> LCDA6::servo_config(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
{
    // Make sure this servo will use modbus
    controlOverModbus = true;

    std::vector<std::vector<uint8_t>> list_of_commands;

    // DI configuration SPEED MODE
    list_of_commands.push_back(write_parameter(slave_id, 0x80, 0));     // DI: ENABLE
    list_of_commands.push_back(write_parameter(slave_id, 0x81, 1));     // DI: ALARM RELEASE
    list_of_commands.push_back(write_parameter(slave_id, 0x82, 2));     // DI: CLOCKWISE LIMIT
    list_of_commands.push_back(write_parameter(slave_id, 0x83, 3));     // DI: COUNTERCLOCKWISE LIMIT
    list_of_commands.push_back(write_parameter(slave_id, 0x84, 5));
    list_of_commands.push_back(write_parameter(slave_id, 0x85, 7));
    list_of_commands.push_back(write_parameter(slave_id, 0x86, 11));
    list_of_commands.push_back(write_parameter(slave_id, 0x87, 12));
    // DO configuration SPEED MODE
    list_of_commands.push_back(write_parameter(slave_id, 0x88, 0));
    list_of_commands.push_back(write_parameter(slave_id, 0x89, 1));
    list_of_commands.push_back(write_parameter(slave_id, 0x8A, 7));
    list_of_commands.push_back(write_parameter(slave_id, 0x8B, 3));
    list_of_commands.push_back(write_parameter(slave_id, 0x8C, 4));
    list_of_commands.push_back(write_parameter(slave_id, 0x8D, 5));

    list_of_commands.push_back(write_parameter(slave_id, 0x90,  1));    // Control Mode - 1 = Communication (extended)

    std::vector<int32_t> values ;
    DEBUG_SERIAL_PRINTLN("*****************Write Config*****************")
    values = processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Write Config*****************")

    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA6::read_servo_brief(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    list_of_commands.push_back(read_parameter(slave_id, 0x00)); // Slave ID
    list_of_commands.push_back(read_parameter(slave_id, 0x05)); // Control Mode
    list_of_commands.push_back(read_parameter(slave_id, 0x0D)); // Baud Rate

    std::vector<int32_t> values;
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************")
    values = processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************")

    return list_of_commands;
}
int64_t LCDA6::get_actual_mechanical_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    list_of_commands.push_back(read_parameter(slave_id, 0x1BC));
    list_of_commands.push_back(read_parameter(slave_id, 0x1BD));
    std::vector<int32_t> values ;
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************");
    values = processListOfCommands(list_of_commands, sendFunction);         
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************");
    converter.as_int64 = 0 ;
    converter.as_int16[0]  = values[0] ;
    converter.as_int16[1]  = values[1] ;
    ActualAbsolutePosition = converter.as_int32[0] ;
    return ActualAbsolutePosition;
}
int64_t LCDA6::get_actual_pulse_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    return 0;
}
int16_t LCDA6::get_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    //FIXME - Leave both for now, due to errors from past
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
std::vector<std::vector<uint8_t>> LCDA6::raw_one_rotation(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    return std::vector<std::vector<uint8_t>>();
}
std::vector<std::vector<uint8_t>> LCDA6::moveRelative(uint8_t slave_id, int32_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction, int32_t speed, float torque)
{
    return std::vector<std::vector<uint8_t>>();
}
int64_t LCDA6::moveAbsolute(uint8_t slave_id, int64_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction, int32_t speed, float torque)
{
    return 0;
}
std::vector<std::vector<uint8_t>> LCDA6::moveVelocity(uint8_t slave_id, int32_t speed, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (!controlOverModbus)
        return list_of_commands;

    //FIXME - Leave both for now, due to errors from past
    // If DI is set as INTSPD
    list_of_commands.push_back(write_parameter(slave_id, 0x53, speed));     // Set Speed value (RPM) into INTSPD0
    // if not used DI as INTSPD
    list_of_commands.push_back(write_parameter(slave_id, 0x140, speed));    // Set Commanded speed to Internal Speed Command 0

    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA6::set_torque(uint8_t slave_id, float torque, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (!controlOverModbus)
        return list_of_commands;

    if (torque > 100)
        torque = 100;
    if (torque < 0)
        torque = 0;

    list_of_commands.push_back(write_parameter(slave_id, 0x5E, torque * 25));     // 1st torque limit
    list_of_commands.push_back(write_parameter(slave_id, 0x12C, torque * 25));    // Internal torque command 0
    

    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA6::config_for_modbus_control_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    return std::vector<std::vector<uint8_t>>();
}
std::vector<std::vector<uint8_t>> LCDA6::config_for_modbus_control_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (controlOverModbus && eControlMode == Speed)
        return list_of_commands ;

    list_of_commands.push_back(write_parameter(slave_id, 0x02, 1));         //(R) Speed Mode    - 1 = speed
    list_of_commands.push_back(write_parameter(slave_id, 0x90, 1));         // Control Mode     - 0 = Analog / 1 = Communication (extended)

    list_of_commands.push_back(write_parameter(slave_id, 0x05, 3));         // Analog / Internal speed selector
    list_of_commands.push_back(write_parameter(slave_id, 0x92, 0));         // Set Commanded speed to Internal Speed Command 0

    int16_t DI_cfg = (1 << 0) | (1 << 1);                                   // DI Config        - (0) enable, (1) alarm release
    list_of_commands.push_back(write_parameter(slave_id, 0x1A0, DI_cfg));   // Set DI source    - 0 = wiring / 1 = communication
    // list_of_commands.push_back(write_parameter(slave_id, 0x1A5, 0x00));  // Set DI mask      - 1 = Input OFF

    // Limit stroke
    list_of_commands.push_back(write_parameter(slave_id, 0x04, 0x00));      //(R) Enable traveling limit setting
    list_of_commands.push_back(write_parameter(slave_id, 0x06, 0x02));      // Enable Zero-speed clamp setting
    list_of_commands.push_back(write_parameter(slave_id, 0x66, 0x01));      //(R) Setting of alarm timing setting of stroke limit - DIRECTION!!!
    list_of_commands.push_back(write_parameter(slave_id, 0x8E, 0x00));      //(R) DI polarity      - 0 = NO / 1 = NC


    list_of_commands.push_back(write_parameter(slave_id, 0x1A7, 0x0801));   // Save parameters


    eControlMode = Speed ;
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    return list_of_commands;
}
bool LCDA6::enable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    // The parameter 0x80 must equal to 0 to enable
    list_of_commands.push_back(write_parameter(slave_id, 0x1A4, 0x03));     // Set communication inputs (enable, alarm release)

    DEBUG_SERIAL_PRINTLN("*****************Enable*****************");
    processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Enable*****************");

    return 0;
}
bool LCDA6::disable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    // The parameter 0x80 must equal to 0 to disable
    list_of_commands.push_back(write_parameter(slave_id, 0x1A4, 0x00));    // Reset inputs

    DEBUG_SERIAL_PRINTLN("*****************Disable*****************");
    processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Disable*****************");
    return 0;
}
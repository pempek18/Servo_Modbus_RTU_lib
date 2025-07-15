#include "LCDA6.hpp"

LCDA6::LCDA6()
{
    DEBUG_SERIAL_PRINTLN("LCDA6 instance declared");
    pulse_per_rotation = 10000;
}
LCDA6::~LCDA6()
{
    DEBUG_SERIAL_PRINTLN("Class destroyed");
};

// Configuration
std::vector<std::vector<uint8_t>> LCDA6::read_servo_brief(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    // Set control mode to communication
    list_of_commands.push_back(write_parameter(slave_id, 0x90, 1));     // Control Mode - (0) Analog / (1) Communication (extended)
    controlOverModbus = true;

    // Read parameters - Verify the config
    list_of_commands.push_back(read_parameter(slave_id, 0x00));         // Slave ID     - Default 0x01
    list_of_commands.push_back(read_parameter(slave_id, 0x0D));         // Baud Rate    - 57600 - 0x05

    std::vector<int32_t> values;
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************")
    values = processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************")

    return list_of_commands;
}
// Mode Configuration
std::vector<std::vector<uint8_t>> LCDA6::config_for_modbus_control_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    // If already in position mode, do nothing
    if (controlOverModbus && eControlMode == Position)
        return list_of_commands ;
    eControlMode = Position ;

    // Set position mode
    list_of_commands.push_back(write_parameter(slave_id, 0x02, ModeToInt(Position)));

    // Set DI source - wiring / communication
    int16_t DI_cfg = (1 << 0) | (1 << 1);                                   // DI Config     - (BIT_0) servo enable, (BIT_1) alarm release
    list_of_commands.push_back(write_parameter(slave_id, 0x1A0, DI_cfg));   // Set DI source - (0) wiring   / (1) communication
    // list_of_commands.push_back(write_parameter(slave_id, 0x1A5, 0x00));  // Set DI mask   - (0) Input ON / (1) Input OFF

    //TODO - Add position mode parameters if needed

    // DI configuration POSITION MODE
    list_of_commands.push_back(write_parameter(slave_id, 0x80, 0));         // DI: SERVO ENABLE
    list_of_commands.push_back(write_parameter(slave_id, 0x81, 1));         // DI: ALARM RELEASE
    list_of_commands.push_back(write_parameter(slave_id, 0x82, 2));         // DI: CLOCKWISE LIMIT
    list_of_commands.push_back(write_parameter(slave_id, 0x83, 3));         // DI: COUNTERCLOCKWISE LIMIT
    list_of_commands.push_back(write_parameter(slave_id, 0x84, 21));        // DI: EMERGENCY STOP
    list_of_commands.push_back(write_parameter(slave_id, 0x85, 20));        // DI: POSITION LOADING
    list_of_commands.push_back(write_parameter(slave_id, 0x86, 17));        // DI: ORIGIN SWITCH
    list_of_commands.push_back(write_parameter(slave_id, 0x87, 18));        // DI: START OF "BACK TO ZERO"

    // DO configuration POSITION MODE
    list_of_commands.push_back(write_parameter(slave_id, 0x88, 0));         // DO: SERVO READY
    list_of_commands.push_back(write_parameter(slave_id, 0x89, 1));         // DO: SERVO ALARM
    list_of_commands.push_back(write_parameter(slave_id, 0x8A, 2));         // DO: LOCATION ARRIVAL
    list_of_commands.push_back(write_parameter(slave_id, 0x8B, 3));         // DO: EXTERNAL BRAKE RELEASE
    list_of_commands.push_back(write_parameter(slave_id, 0x8C, 4));         // DO: ZERO SPEED DETECTION
    list_of_commands.push_back(write_parameter(slave_id, 0x8D, 5));         // DO: TORQUE LIMIT

    // Save parameters
    list_of_commands.push_back(write_parameter(slave_id, 0x1A7, 0x0801));

    DEBUG_SERIAL_PRINTLN("*****************Config for position mode*****************");
    processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Config for position mode*****************");
    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA6::config_for_modbus_control_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    // If already in speed mode, do nothing
    if (controlOverModbus && eControlMode == Speed)
        return list_of_commands ;
    eControlMode = Speed ;

    // Set speed mode
    list_of_commands.push_back(write_parameter(slave_id, 0x02, ModeToInt(Speed)));
    // Set speed source
    list_of_commands.push_back(write_parameter(slave_id, 0x05, 3));         // (0) Analog / (1-3) Internal speed selector
    list_of_commands.push_back(write_parameter(slave_id, 0x92, 0));         // Set Commanded speed to Internal Speed Command 0

    // Set DI source - wiring / communication
    int16_t DI_cfg = (1 << 0) | (1 << 1);                                   // DI Config     - (BIT_0) servo enable, (BIT_1) alarm release
    list_of_commands.push_back(write_parameter(slave_id, 0x1A0, DI_cfg));   // Set DI source - (0) wiring   / (1) communication
    // list_of_commands.push_back(write_parameter(slave_id, 0x1A5, 0x00));  // Set DI mask   - (0) Input ON / (1) Input OFF

    // Limit stroke
    list_of_commands.push_back(write_parameter(slave_id, 0x04, 0x00));      //(R) Enable traveling limit setting - (0) Enable & use PA_066 / (1) Disable
    list_of_commands.push_back(write_parameter(slave_id, 0x06, 0x02));      // Enable Zero-speed clamp setting   - (0) Disable / (1-2) Enable
    list_of_commands.push_back(write_parameter(slave_id, 0x66, 0x01));      //(R) Setting of alarm timing setting of stroke limit - Limit direction (Check polarity)
    list_of_commands.push_back(write_parameter(slave_id, 0x8E, 0x00));      //(R) DI polarity   - (0) NO / (1) NC

    // DI configuration SPEED MODE
    list_of_commands.push_back(write_parameter(slave_id, 0x80, 0));         // DI: SERVO ENABLE
    list_of_commands.push_back(write_parameter(slave_id, 0x81, 1));         // DI: ALARM RELEASE
    list_of_commands.push_back(write_parameter(slave_id, 0x82, 2));         // DI: CLOCKWISE LIMIT
    list_of_commands.push_back(write_parameter(slave_id, 0x83, 3));         // DI: COUNTERCLOCKWISE LIMIT
    list_of_commands.push_back(write_parameter(slave_id, 0x84, 5));         // DI: ZERO SPEED CLAMP
    list_of_commands.push_back(write_parameter(slave_id, 0x85, 7));         // DI: SPEED COMAND DIRECTION
    list_of_commands.push_back(write_parameter(slave_id, 0x86, 11));        // DI: INTSPD1
    list_of_commands.push_back(write_parameter(slave_id, 0x87, 12));        // DI: INTSPD2
    // DO configuration SPEED MODE
    list_of_commands.push_back(write_parameter(slave_id, 0x88, 0));         // DO: SERVO READY
    list_of_commands.push_back(write_parameter(slave_id, 0x89, 1));         // DO: SERVO ALARM
    list_of_commands.push_back(write_parameter(slave_id, 0x8A, 7));         // DO: SPEED ARRIVAL
    list_of_commands.push_back(write_parameter(slave_id, 0x8B, 3));         // DO: EXTERNAL BRAKE RELEASE
    list_of_commands.push_back(write_parameter(slave_id, 0x8C, 4));         // DO: ZERO SPEED DETECTION
    list_of_commands.push_back(write_parameter(slave_id, 0x8D, 5));         // DO: TORQUE LIMIT

    // Save parameters
    list_of_commands.push_back(write_parameter(slave_id, 0x1A7, 0x0801));

    DEBUG_SERIAL_PRINTLN("*****************Config for speed mode*****************");
    processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Config for speed mode*****************");
    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA6::config_for_modbus_control_torque(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    // If already in torque mode, do nothing
    if (controlOverModbus && eControlMode == Torque)
        return list_of_commands ;
    eControlMode = Torque ;

    // Set torque mode
    list_of_commands.push_back(write_parameter(slave_id, 0x02, ModeToInt(Torque)));

    // Set DI source - wiring / communication
    int16_t DI_cfg = (1 << 0) | (1 << 1);                                   // DI Config     - (BIT_0) servo enable, (BIT_1) alarm release
    list_of_commands.push_back(write_parameter(slave_id, 0x1A0, DI_cfg));   // Set DI source - (0) wiring   / (1) communication
    // list_of_commands.push_back(write_parameter(slave_id, 0x1A5, 0x00));  // Set DI mask   - (0) Input ON / (1) Input OFF

    //TODO - Add torque mode parameters if needed

    // DI configuration TORQUE MODE
    list_of_commands.push_back(write_parameter(slave_id, 0x80, 0));         // DI: SERVO ENABLE
    list_of_commands.push_back(write_parameter(slave_id, 0x81, 1));         // DI: ALARM RELEASE
    list_of_commands.push_back(write_parameter(slave_id, 0x82, 2));         // DI: CLOCKWISE LIMIT
    list_of_commands.push_back(write_parameter(slave_id, 0x83, 3));         // DI: COUNTERCLOCKWISE LIMIT
    list_of_commands.push_back(write_parameter(slave_id, 0x84, 5));         // DI: ZERO SPEED CLAMP
    list_of_commands.push_back(write_parameter(slave_id, 0x85, 15));        // DI: TORQUE LIMIT SWITCHOVER
    list_of_commands.push_back(write_parameter(slave_id, 0x86, 11));        // DI: INTSPD1
    list_of_commands.push_back(write_parameter(slave_id, 0x87, 12));        // DI: INTSPD2

    // DO configuration TORQUE MODE
    list_of_commands.push_back(write_parameter(slave_id, 0x88, 0));         // DO: SERVO READY
    list_of_commands.push_back(write_parameter(slave_id, 0x89, 1));         // DO: SERVO ALARM
    list_of_commands.push_back(write_parameter(slave_id, 0x8A, 7));         // DO: SPEED ARRIVAL
    list_of_commands.push_back(write_parameter(slave_id, 0x8B, 3));         // DO: EXTERNAL BRAKE RELEASE
    list_of_commands.push_back(write_parameter(slave_id, 0x8C, 4));         // DO: ZERO SPEED DETECTION
    list_of_commands.push_back(write_parameter(slave_id, 0x8D, 5));         // DO: TORQUE LIMIT

    // Save parameters
    list_of_commands.push_back(write_parameter(slave_id, 0x1A7, 0x0801));

    DEBUG_SERIAL_PRINTLN("*****************Config for torque mode*****************");
    processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Config for torque mode*****************");
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

// Position
int64_t LCDA6::get_actual_mechanical_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    list_of_commands.push_back(read_parameter(slave_id, 0x1BC, (uint16_t)4)); // Force to use address (2 bytes) and get 2 registers (4 bytes)
    // list_of_commands.push_back(read_parameter(slave_id, 0x1BC));
    // list_of_commands.push_back(read_parameter(slave_id, 0x1BD));
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
    //TODO
    return 0;
    std::vector<std::vector<uint8_t>> list_of_commands;

    std::vector<int32_t> values ;
    DEBUG_SERIAL_PRINTLN("*****************Read Pulse Position*****************");
    values = processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Read Pulse Position*****************");
}
std::vector<std::vector<uint8_t>> LCDA6::moveRelative(uint8_t slave_id, int32_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction, int32_t speed, float torque)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (!controlOverModbus)
        return list_of_commands;

    list_of_commands.push_back(write_parameter(slave_id, 0x94 , 0x01));         // Move relative
    list_of_commands.push_back(write_parameter_32(slave_id, 0x168, position));  // Internal Position Command 0

    DEBUG_SERIAL_PRINTLN("*****************Write Relative Position*****************");
    processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Write Relative Position*****************");
    return list_of_commands;
}
int64_t LCDA6::moveAbsolute(uint8_t slave_id, int64_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction, int32_t speed, float torque)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (!controlOverModbus)
        return 0;

    //TODO
    list_of_commands.push_back(write_parameter(slave_id, 0x94 , 0x00));         // Move absolute
    list_of_commands.push_back(write_parameter_32(slave_id, 0x168, position));  // Internal Position Command 0

    DEBUG_SERIAL_PRINTLN("*****************Write Absolute Position*****************");
    processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Write Absolute Position*****************");
}

// Speed
int16_t LCDA6::get_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<uint8_t> command = read_parameter(slave_id, 0x1C1);             // Get Feedback speed
    std::vector<uint8_t> response = sendFunction(command);

    DEBUG_SERIAL_PRINTLN("*****************Read Current Speed*****************")
    int32_t value = parseModbusResponse(response);
    DEBUG_SERIAL_PRINTLN("*****************Read Current Speed*****************")
    return value;
}
std::vector<std::vector<uint8_t>> LCDA6::moveVelocity(uint8_t slave_id, int32_t speed, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (!controlOverModbus)
        return list_of_commands;

    list_of_commands.push_back(write_parameter(slave_id, 0x53 , speed));        // 1st Internal speed
    list_of_commands.push_back(write_parameter(slave_id, 0x140, speed));        // Internal Speed Command 0

    DEBUG_SERIAL_PRINTLN("*****************Write Speed*****************");
    processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Write Speed*****************");
    return list_of_commands;
}

// Torque
int16_t LCDA6::get_torque(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction){
    std::vector<uint8_t> command = read_parameter(slave_id, 0x1C4);             // Get Feedback torque
    std::vector<uint8_t> response = sendFunction(command);

    DEBUG_SERIAL_PRINTLN("*****************Read Current Torque*****************")
    int32_t value = parseModbusResponse(response);
    DEBUG_SERIAL_PRINTLN("*****************Read Current Torque*****************")
    return value;
}
std::vector<std::vector<uint8_t>> LCDA6::set_torque(uint8_t slave_id, float torque, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (!controlOverModbus)
        return list_of_commands;

    if (torque > 100)
        torque = 100;
    else if (torque < 0)
        torque = 0;

    list_of_commands.push_back(write_parameter(slave_id, 0x5E , torque * 25));  // 1st torque limit
    list_of_commands.push_back(write_parameter(slave_id, 0x12C, torque * 25));  // Internal torque command 0


    DEBUG_SERIAL_PRINTLN("*****************Write Torque*****************");
    processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Write Torque*****************");
    return list_of_commands;
}

// Raw
std::vector<std::vector<uint8_t>> LCDA6::raw_one_rotation(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    //TODO
    return std::vector<std::vector<uint8_t>>();
}


int8_t LCDA6::ModeToInt(servomode mode){
    // Map mode to int - according to the LCDA6 manual
    switch (mode){
        case servomode::Position:
            return 0;
        case servomode::Speed:
            return 1;
        case servomode::Torque:
            return 2;
        case servomode::SpeedPosition:
            return 3;
        case servomode::TorquePosition:
            return 4;
        case servomode::TorqueSpeed:
            return 5;
        default:
            return 1; // Default to speed
    }
}
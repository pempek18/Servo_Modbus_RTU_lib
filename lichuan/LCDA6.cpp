#include "LCDA6.hpp"

LCDA6::LCDA6()
{
    DEBUG_SERIAL_PRINTLN("LCDA6 instance declared");
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
    list_of_commands.push_back(write_parameter(slave_id, 0x8F, 0));     // Manual Servo Enable - (0) Manual / (1) Power-On Automatic
    list_of_commands.push_back(write_parameter(slave_id, 0x90, 1));     // Control Mode - (0) Analog / (1) Communication (extended)
    controlOverModbus = true;

    // Read parameters - Verify the config
    list_of_commands.push_back(read_parameter(slave_id, 0x00));         // Slave ID     - Default 0x01
    //TODO write slave ID per device defined
    list_of_commands.push_back(read_parameter(slave_id, 0x0D));         // Baud Rate    - 57600 - 0x05
    list_of_commands.push_back(read_parameter(slave_id, 0x4A));         // Pulses per revolution
    // list_of_commands.push_back(read_parameter(slave_id, 0x45));         // Feedback pulse division factor (not working)
    std::vector<int32_t> values;
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************")
    values = processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************")

    pulse_per_rotation = values[4]; // parameter 0x4A
    // encoder_resolution = float(pulse_per_rotation / 4) * values[5]; // parameter 0x45 (not working)
    encoder_resolution = 0x1FFFF; // 17 bits - driver version Cxy
    return list_of_commands;
}
// Mode Configuration
std::vector<std::vector<uint8_t>> LCDA6::config_for_modbus_control_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    // If already in position mode, do nothing
    if (controlOverModbus && eControlMode == servomode::Position)
        return list_of_commands ;
    eControlMode = servomode::Position ;

    // Set position mode
    list_of_commands.push_back(write_parameter(slave_id, 0x02, ModeToInt(servomode::Position)));

    // Set DI source - wiring / communication
    int16_t DI_cfg = (1 << 0) | // DI Config - (BIT_0) servo enable
                     (1 << 1) | //           - (BIT_1) alarm release
                     (1 << 5);  //           - (BIT_5) position loading
    list_of_commands.push_back(write_parameter(slave_id, 0x1A0, DI_cfg));   // Set DI source - (0) wiring   / (1) communication
    // list_of_commands.push_back(write_parameter(slave_id, 0x1A5, 0x00));  // Set DI mask   - (0) Input ON / (1) Input OFF

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
    if (controlOverModbus && eControlMode == servomode::Speed)
        return list_of_commands ;
    eControlMode = servomode::Speed ;

    // Set speed mode
    list_of_commands.push_back(write_parameter(slave_id, 0x02, ModeToInt(servomode::Speed)));
    // Set speed source
    list_of_commands.push_back(write_parameter(slave_id, 0x05, 3));         // (0) Analog / (1-3) Internal speed selector
    list_of_commands.push_back(write_parameter(slave_id, 0x92, 0));         // Set Commanded speed to Internal Speed Command 0

    // Set DI source - wiring / communication
    int16_t DI_cfg = (1 << 0) | // DI Config - (BIT_0) servo enable
                     (1 << 1);  //           - (BIT_1) alarm release
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
    if (controlOverModbus && eControlMode == servomode::Torque)
        return list_of_commands ;
    eControlMode = servomode::Torque;

    // Set torque mode
    list_of_commands.push_back(write_parameter(slave_id, 0x02, ModeToInt(servomode::Torque)));

    // Set DI source - wiring / communication
    int16_t DI_cfg = (1 << 0) | // DI Config - (BIT_0) servo enable
                     (1 << 1);  //           - (BIT_1) alarm release
    list_of_commands.push_back(write_parameter(slave_id, 0x1A0, DI_cfg));   // Set DI source - (0) wiring   / (1) communication
    // list_of_commands.push_back(write_parameter(slave_id, 0x1A5, 0x00));  // Set DI mask   - (0) Input ON / (1) Input OFF

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
std::vector<std::vector<uint8_t>> LCDA6::config_for_modbus_control_position_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    // If already in speed mode, do nothing
    std::vector<uint8_t> command = read_parameter(slave_id, 0x02);
    std::vector<uint8_t> response = sendFunction(command);
    int32_t value = parseModbusResponse(response);
    if (value == ModeToInt(servomode::PositionSpeed))
    {
        DEBUG_SERIAL_PRINTLN("Config for speed position mode already in");
        eControlMode = servomode::PositionSpeed;
        controlOverModbus = true;
        return list_of_commands;
    }
    DEBUG_SERIAL_PRINTLN("*****************Config for speed position mode*****************");
    // Set speed mode
    list_of_commands.push_back(write_parameter(slave_id, 0x02, ModeToInt(servomode::PositionSpeed)));
    // Set speed & position source
    list_of_commands.push_back(write_parameter(slave_id, 0x04, 0x00));      //(R) Enable traveling limit setting - (0) Enable & use PA_066 / (1) Disable
    list_of_commands.push_back(write_parameter(slave_id, 0x05, 0));         // (0) Analog / (1-3) Internal speed selector
    list_of_commands.push_back(write_parameter(slave_id, 0x58, 200));       // Acceleration time
    list_of_commands.push_back(write_parameter(slave_id, 0x59, 200));       // Deceleration time
    list_of_commands.push_back(write_parameter(slave_id, 0x90, 1));         // Extended function mode (using communication control).
    list_of_commands.push_back(write_parameter(slave_id, 0x92, 0));         // Set Commanded speed to Internal Speed Command 0
    list_of_commands.push_back(write_parameter(slave_id, 0x94 , 0x00));     // Position control - (0) Absolute / (1) Relative
    list_of_commands.push_back(write_parameter(slave_id, 0x96 , 0x02));     // Nastawa pozycji narastająca
//FIXME//FIXME//FIXME//FIXME//FIXME
    // Set DI source - wiring / communication
    int16_t DI_cfg = (1 << 0) | // DI Config - (BIT_0) servo enable
                     (1 << 1) | //           - (BIT_1) alarm release
                     (1 << 4) | //           - (BIT_4) C_MODE change
                     (1 << 5);  //           - (BIT_5) position loading

    list_of_commands.push_back(write_parameter(slave_id, 0x1A0, DI_cfg));   // Set DI source - (0) wiring   / (1) communication
    list_of_commands.push_back(write_parameter(slave_id, 0x1A4, 0x00));     // Communication inputs (enable, alarm release, c_mode change to position)
    // list_of_commands.push_back(write_parameter(slave_id, 0x1A5, 0x00));  // Set DI mask   - (0) Input ON / (1) Input OFF

    // Limit stroke
    list_of_commands.push_back(write_parameter(slave_id, 0x04, 0x00));      //(R) Enable traveling limit setting - (0) Enable & use PA_066 / (1) Disable
    list_of_commands.push_back(write_parameter(slave_id, 0x06, 0x02));      // Enable Zero-speed clamp setting   - (0) Disable / (1-2) Enable
    list_of_commands.push_back(write_parameter(slave_id, 0x66, 0x01));      //(R) Setting of alarm timing setting of stroke limit - Limit direction (Check polarity)
    list_of_commands.push_back(write_parameter(slave_id, 0x8E, 0x00));      //(R) DI polarity - (0) NO / (1) NC

    // DI configuration SPEED MODE //FIXME decide upon the DI configuration
    list_of_commands.push_back(write_parameter(slave_id, 0x80, 0));         // DI: SERVO ENABLE
    list_of_commands.push_back(write_parameter(slave_id, 0x81, 1));         // DI: ALARM RELEASE
    list_of_commands.push_back(write_parameter(slave_id, 0x82, 2));         // DI: CLOCKWISE LIMIT
    list_of_commands.push_back(write_parameter(slave_id, 0x83, 3));         // DI: COUNTERCLOCKWISE LIMIT
    list_of_commands.push_back(write_parameter(slave_id, 0x84, 4));         // DI: C_MODE change Control mode switching
    list_of_commands.push_back(write_parameter(slave_id, 0x85, 20));        // DI: POSITION LOADING
    list_of_commands.push_back(write_parameter(slave_id, 0x86, 11));        // DI: INTSPD1
    list_of_commands.push_back(write_parameter(slave_id, 0x87, 12));        // DI: INTSPD2

    // DO configuration SPEED MODE //FIXME decide upon the DO configuration
    list_of_commands.push_back(write_parameter(slave_id, 0x88, 0));         // DO: SERVO READY
    list_of_commands.push_back(write_parameter(slave_id, 0x89, 1));         // DO: SERVO ALARM
    list_of_commands.push_back(write_parameter(slave_id, 0x8A, 7));         // DO: SPEED ARRIVAL
    list_of_commands.push_back(write_parameter(slave_id, 0x8B, 2));         // DO: LOCATION ARRIVAL
    list_of_commands.push_back(write_parameter(slave_id, 0x8C, 4));         // DO: ZERO SPEED DETECTION
    list_of_commands.push_back(write_parameter(slave_id, 0x8D, 5));         // DO: TORQUE LIMIT

    // Save parameters
    list_of_commands.push_back(write_parameter(slave_id, 0x1A7, 0x0801));

    processListOfCommands(list_of_commands, sendFunction);   
    eControlMode = servomode::PositionSpeed;
    DEBUG_SERIAL_PRINTLN("*****************Config for speed position mode*****************");
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
    list_of_commands.push_back(read_parameter(slave_id, 0x1BC));
    list_of_commands.push_back(read_parameter(slave_id, 0x1BD));
    std::vector<int32_t> values ;
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************");
    values = processListOfCommands(list_of_commands, sendFunction);
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************");
    // converter.as_int64 = 0 ;
    // converter.as_int16[0]  = values[0] ;
    // converter.as_int16[1]  = values[1] ;
    // ActualAbsolutePosition = converter.as_int32[0] ;

    // Combine the two 16-bit words into a signed 32-bit integer
    uint16_t low  = static_cast<uint16_t>(values[0] & 0xFFFF);
    uint16_t high = static_cast<uint16_t>(values[1] & 0xFFFF);

    uint32_t combined = (static_cast<uint32_t>(high) << 16) | low;
    int32_t signed32 = static_cast<int32_t>(combined);

    return static_cast<int64_t>(signed32);

    return ActualAbsolutePosition;
}
int64_t LCDA6::get_actual_pulse_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    ActualAbsolutePosition = get_actual_mechanical_position(slave_id, sendFunction);
    float position =  (float)ActualAbsolutePosition / (float)encoder_resolution  * (float)pulse_per_rotation;
    ActualPulseCounterPosition = (int64_t)position;
    return ActualPulseCounterPosition;
}
std::vector<std::vector<uint8_t>> LCDA6::moveRelative(uint8_t slave_id, int32_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction, int32_t speed, float torque)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (!controlOverModbus)
        return list_of_commands;
    get_actual_pulse_position(slave_id, sendFunction);
    int64_t position_delta = ActualPulseCounterPosition + position;
    moveAbsolute(slave_id, position_delta, sendFunction, speed, torque);
    
    return list_of_commands;
}
int64_t LCDA6::moveAbsolute(uint8_t slave_id, int64_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction, int32_t speed, float torque)
{
    if (!controlOverModbus)
        return 0;
    std::vector<std::vector<uint8_t>> list_of_commands;
    converter.as_int64 = position;
    list_of_commands.push_back(write_parameter(slave_id, 0x168, converter.as_int16[0]));  // Internal Position Command 0
    list_of_commands.push_back(write_parameter(slave_id, 0x169, converter.as_int16[1]));  // Internal Position Command 0
    list_of_commands.push_back(write_parameter(slave_id, 0x190, abs(speed)));   // Internal Position Speed Command 0
    list_of_commands.push_back(write_parameter(slave_id, 0x91, 0));             // Position mode index
    int16_t DI_cfg = (1 << 0) | // DI Config - (BIT_0) servo enable
                     (1 << 1) | //           - (BIT_1) alarm release
                     (0 << 4) | //           - (BIT_4) C_MODE change
                     (1 << 5);  //           - (BIT_5) position loading

    list_of_commands.push_back(write_parameter(slave_id, 0x1A4, DI_cfg));   // Set DI source - (0) wiring   / (1) communication     
    DEBUG_SERIAL_PRINTLN("*****************Write Absolute Position*****************");
    processListOfCommands(list_of_commands, sendFunction, false);
    DEBUG_SERIAL_PRINTLN("*****************Write Absolute Position*****************");
            DI_cfg = (1 << 0) | // DI Config - (BIT_0) servo enable
                     (1 << 1) | //           - (BIT_1) alarm release
                     (0 << 4) | //           - (BIT_4) C_MODE change
                     (0 << 5);  //           - (BIT_5) position loading
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    list_of_commands.clear();
    list_of_commands.push_back(write_parameter(slave_id, 0x1A4, DI_cfg));   // Set DI source - (0) wiring   / (1) communication 
    processListOfCommands(list_of_commands, sendFunction, false);
    return position; // Return the target position
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
    {
        DEBUG_SERIAL_PRINTLN("Not in control over modbus");
        return list_of_commands;
    }
    
    if (eControlMode == servomode::PositionSpeed)
    {
        int16_t DI_cfg =    (1 << 0) | // DI Config - (BIT_0) servo enable
                            (1 << 1) | //           - (BIT_1) alarm release
                            (1 << 4) | //           - (BIT_4) C_MODE change
                            (0 << 5);  //           - (BIT_5) position loading

        list_of_commands.push_back(write_parameter(slave_id, 0x1A4, DI_cfg));   // Set DI source - (0) wiring   / (1) communication 
        list_of_commands.push_back(write_parameter(slave_id, 0x140 , speed));        // 1st Internal speed - unused - leave for any issues in future
    }else if (eControlMode == servomode::Speed)
    {
        list_of_commands.push_back(write_parameter(slave_id, 0x140, speed));        // Internal Speed Command 0
    }
    // list_of_commands.push_back(write_parameter(slave_id, 0x190, abs(speed)));   // Internal Position Speed Command 0 - speed when setting position if 0x90 = 0

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

bool LCDA6::inTargetPosition(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::pair<uint16_t, uint16_t>> output_state = get_output_state(slave_id, sendFunction);
    for (int i = 0; i < output_state.size(); i++)
    {
        if (output_state[i].first == 2 && output_state[i].second == 1)
            return true;
    }
    return false;
}

bool LCDA6::inTargetSpeed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<uint8_t> command = read_parameter(slave_id, 0x8A);         // DO: SPEED ARRIVAL
    std::vector<uint8_t> response = sendFunction(command);
    int32_t value = parseModbusResponse(response);
    if (value != 7)
    {
        DEBUG_SERIAL_PRINTLN("Unable to read speed arrival, no speed arrival output");
        return false;
    }
    command = read_parameter(slave_id, 0x1D3);             // Get Feedback speed
    response = sendFunction(command);

    DEBUG_SERIAL_PRINTLN("*****************Read Current Speed*****************")
    value = parseModbusResponse(response);
    DEBUG_SERIAL_PRINTLN("*****************Read Current Speed*****************")
    if (value == 0)
        return true;
    else
        return false;
    return false;
}

std::vector<std::pair<uint16_t, uint16_t>> LCDA6::get_output_state(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    DEBUG_SERIAL_PRINTLN("*****************Get Output State*****************");
    MB::ModbusRequest request = MB::ModbusRequest(slave_id, MB::utils::ReadAnalogOutputHoldingRegisters, 0x88, 6);
    std::vector<uint8_t> frame = request.toRaw();

    // Calculate CRC using uint8_t data
    uint16_t CRC = MB::utils::calculateCRC(frame);
    auto CRCptr  = reinterpret_cast<uint8_t *>(&CRC);
    frame.insert(frame.end(), CRCptr, CRCptr + 2);

    std::vector<uint8_t> response = sendFunction(frame);
    MB::ModbusResponse response_settings = MB::ModbusResponse::fromRaw(response);
    
    request = MB::ModbusRequest(slave_id, MB::utils::ReadAnalogOutputHoldingRegisters, 0x1D3, 1);
    frame.clear();
    frame = request.toRaw();
    CRC = MB::utils::calculateCRC(frame);
    CRCptr  = reinterpret_cast<uint8_t *>(&CRC);
    frame.insert(frame.end(), CRCptr, CRCptr + 2);
    response.clear();
    response = sendFunction(frame);
    MB::ModbusResponse response_values = MB::ModbusResponse::fromRaw(response);

    std::vector<std::pair<uint16_t, uint16_t>> output_state_vector;
    for (int i = 0; i < 6; i++)
    {
        uint16_t setting = response_settings.registerValues()[i].reg();
        uint16_t value = (response_values.registerValues()[0].reg() & (1 << i)) ? 1 : 0;
        output_state_vector.push_back(std::make_pair(setting, value));
        DEBUG_SERIAL_PRINT("Output state setting ");
        DEBUG_SERIAL_PRINT(i);
        DEBUG_SERIAL_PRINT(" is ");
        DEBUG_SERIAL_PRINT(setting);
        DEBUG_SERIAL_PRINT(" and value is ");
        DEBUG_SERIAL_PRINTLN(value);
    }
    DEBUG_SERIAL_PRINTLN("*****************Get Output State*****************");
    return output_state_vector;
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
        case servomode::PositionSpeed:
            return 3;
        case servomode::PositionTorque:
            return 4;
        case servomode::SpeedTorque:
            return 5;
        case servomode::CanOpen:
            return 10;
        default:
            return 1; // Default to speed
    }
}
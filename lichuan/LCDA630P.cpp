#include "LCDA630P.hpp"

LCDA630P::LCDA630P()
{
    DEBUG_SERIAL_PRINTLN("Class declared");
};

LCDA630P::~LCDA630P()
{
    DEBUG_SERIAL_PRINTLN("Class destroyed");    
};

std::vector<std::vector<uint8_t>> LCDA630P::read_servo_brief(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)0, (uint8_t)0));
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)0, (uint8_t)9));
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)0, (uint8_t)10));
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)0, (uint8_t)11));
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)0, (uint8_t)12));
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)0, (uint8_t)13));
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)0, (uint8_t)14));
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)0, (uint8_t)15));
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)0, (uint8_t)28));
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)5, (uint8_t)0));
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)12, (uint8_t)26));
    std::vector<int32_t> values ;
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************");
    values = processListOfCommands(list_of_commands, sendFunction);      
    DEBUG_SERIAL_PRINTLN("*****************Read Brief*****************");
    MotorNumber =  values.at(0) ; 
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

    std::stringstream ss ;
        ss << "MotorNumber: " << std::dec << MotorNumber << std::endl ;
        ss << "RatedVoltage: " << std::dec << RatedVoltage << std::endl ;
        ss << "RatedPower: " << std::dec << RatedPower << std::endl ;
        ss << "RatedCurrent: " << std::dec << RatedCurrent << std::endl ;
        ss << "RatedTorque: " << std::dec << RatedTorque << std::endl ;
        ss << "MaxTorque: " << std::dec << MaxTorque << std::endl ;
        ss << "RatedSpeed: " << std::dec << RatedSpeed << std::endl ;
        ss << "MaxSpeed: " << std::dec << MaxSpeed << std::endl ;
        ss << "PositionOffsetOfAbsolutEncoder: " << std::dec << PositionOffsetOfAbsolutEncoder << std::endl ;
        ss << "controlOverModbus: " << std::dec << controlOverModbus << std::endl ;
        ss << "lower16_bit_first: " << std::dec << lower16_bit_first << std::endl ;
        ss << "encoder_resolution: " << std::dec << encoder_resolution << std::endl ;
        ss << "pulse_per_rotation: " << std::dec << pulse_per_rotation << std::endl ;
        ss << "ActualAbsolutePosition: " << std::dec << ActualAbsolutePosition << std::endl ;
        ss << "ActualPulseCounterPosition: " << std::dec << ActualPulseCounterPosition << std::endl ;
        ss << "ActualSpeedRpm: " << std::dec << ActualSpeedRpm << std::endl ;
        ss << "eControlMode: " << std::dec << eControlMode << std::endl ;

    DEBUG_SERIAL_PRINTLN(ss.str().c_str());

    return list_of_commands;
}
int64_t LCDA630P::get_actual_mechanical_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)11, (uint8_t)77, (uint8_t)9));
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)11, (uint8_t)79, (uint8_t)9));
    std::vector<int32_t> values ;
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************");
    values = processListOfCommands(list_of_commands, sendFunction);         
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************");
    converter.as_int32[0]  = values[0] ;
    converter.as_int32[1]  = values[1] ;
    ActualAbsolutePosition = converter.as_int64 ;
    return ActualAbsolutePosition;
}
int64_t LCDA630P::get_actual_pulse_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    ActualAbsolutePosition = get_actual_mechanical_position(slave_id, sendFunction);
    float position =  (float)ActualAbsolutePosition / (float)encoder_resolution  * (float)pulse_per_rotation;
    ActualPulseCounterPosition = (int64_t)position;
    return ActualPulseCounterPosition;
}
int16_t LCDA630P::get_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands;
    list_of_commands.push_back(read_parameter(slave_id, (uint8_t)11, (uint8_t)55));
    std::vector<int32_t> values ;
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************");
    values = processListOfCommands(list_of_commands, sendFunction);        
    DEBUG_SERIAL_PRINTLN("*****************Read Absolute Position*****************");   
    ActualSpeedRpm = values[0];
    return ActualSpeedRpm;
}
std::vector<std::vector<uint8_t>> LCDA630P::raw_one_rotation(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
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
    processListOfCommands(list_of_commands, sendFunction); 
#if DEBUG_SERIAL
    debug_print_frame(frame, true);
#endif              
    DEBUG_SERIAL_PRINTLN("*****************Move one rotation RAW DATA*****************");
    return list_of_commands ;
}
std::vector<std::vector<uint8_t>> LCDA630P::moveRelative(uint8_t slave_id, int32_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (!controlOverModbus)
        return list_of_commands;
    list_of_commands.push_back(write_parameter_32(slave_id, (uint8_t)0x11, (uint8_t)0x0C, position));//Multi-segment location operation mode Sequential operation (P11-01 for selection of segment number)
    list_of_commands.push_back(write_parameter(slave_id,    (uint8_t)0x31, (uint8_t)0, (uint8_t)1));//Communication given VDI virtual level 0～65535
    list_of_commands.push_back(write_parameter(slave_id,    (uint8_t)0x31, (uint8_t)0, (uint8_t)3));//Communication given VDI virtual level 0～65535
    DEBUG_SERIAL_PRINTLN("*****************Move to pos*****************");
    processListOfCommands(list_of_commands, sendFunction); 
    DEBUG_SERIAL_PRINTLN("*****************Move to pos*****************");
    return list_of_commands;
}
int64_t LCDA630P::moveAbsolute(uint8_t slave_id, int64_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    config_for_modbus_control_position(1, sendFunction);
    ActualPulseCounterPosition = get_actual_pulse_position(slave_id, sendFunction);
    int64_t diffToPos = (position - ActualPulseCounterPosition);
    std::stringstream ss ;
        ss << "Absolute Position " << std::dec << ActualPulseCounterPosition << std::endl ;
        ss << "Setpoint Position " << std::dec << position << std::endl ;
        ss << "Difference in position: " << std::dec << diffToPos << std::endl ; 
    DEBUG_SERIAL_PRINTLN(ss.str().c_str());
    moveRelative(slave_id, (int32_t)diffToPos, sendFunction);
    ActualPulseCounterPosition = get_actual_pulse_position(slave_id, sendFunction);
    return ActualPulseCounterPosition ;
}
std::vector<std::vector<uint8_t>> LCDA630P::moveVelocity(uint8_t slave_id, int32_t speed, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (!controlOverModbus)
        return list_of_commands;
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x2, (uint8_t)0, (uint8_t)0));//Control Mode Selectio 0: speed mod
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x17, (uint8_t)0, (uint8_t)1));//VDI1 Terminal function selection
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x31, (uint8_t)0, (uint8_t)1));//Communication given VDI virtual level 0～65535 16 bit input register 
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x06, (uint8_t)0x03, speed));//Multi-segment location operation mode Sequential operation (P11-01 for selection of segment number)
    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    processListOfCommands(list_of_commands, sendFunction); 
    DEBUG_SERIAL_PRINTLN("*****************Rotate with speed*****************");
    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA630P::set_torque(uint8_t slave_id, float torque, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    // Convert torque percentage to int16_t with bounds checking
    float scaled_torque = torque * 10.0f;
    int16_t value;
    
    if (scaled_torque > 32767.0f) {
        value = 32767;  // Max value for int16_t
    } else if (scaled_torque < -32768.0f) {
        value = -32768; // Min value for int16_t
    } else {
        value = static_cast<int16_t>(std::round(scaled_torque));
    }
    std::vector<std::vector<uint8_t>> list_of_commands ;
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x07, (uint8_t)9, value ));
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x07, (uint8_t)10, value ));
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x07, (uint8_t)11, value ));
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x07, (uint8_t)12, value ));
    DEBUG_SERIAL_PRINTLN("*****************Set Torque*****************");
    processListOfCommands(list_of_commands, sendFunction); 
    DEBUG_SERIAL_PRINTLN("*****************Set Torque*****************");
    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA630P::config_for_modbus_control_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (controlOverModbus && eControlMode == Position)
        return list_of_commands ;
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x17, (uint8_t)0, (uint8_t)1));//VDI1 Terminal function selection
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x17, (uint8_t)2, (uint8_t)28));//VDI2 Terminal function selection
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x2, (uint8_t)0, (uint8_t)1));//Control Mode Selectio 1: position mod
    // list_of_commands.push_back(write_parameter(1,0x5,0,2));//Control Mode Selectio 1: position mod
    // list_of_commands.push_back(write_parameter(1,0x5,2,10000));//Location instruction source multi-segment position instruction give
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x11, (uint8_t)0, (uint8_t)2));//Multi-segment location operation mode Sequential operation (P11-01 for selection of segment number)
    eControlMode = Position ; 
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    processListOfCommands(list_of_commands, sendFunction); 
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    return list_of_commands;
}
std::vector<std::vector<uint8_t>> LCDA630P::config_for_modbus_control_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    std::vector<std::vector<uint8_t>> list_of_commands ;
    if (controlOverModbus && eControlMode == Speed)
        return list_of_commands ;
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x17, (uint8_t)0, (uint8_t)1));//VDI1 Terminal function selection
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x17, (uint8_t)2, (uint8_t)28));//VDI2 Terminal function selection
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x17, (uint8_t)3, (uint8_t)26));//VDI3 Terminal function selection
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x2, (uint8_t)0, (uint8_t)0));//Control Mode Selectio 0: speed mod
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x6, (uint8_t)0, (uint8_t)0));//Location instruction source multi-segment position instruction give
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x6, (uint8_t)1, (uint8_t)5));//Location instruction source multi-segment position instruction give
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x6, (uint8_t)2, (uint8_t)0));//Location instruction source multi-segment position instruction give
    list_of_commands.push_back(write_parameter(1, (uint8_t)0x11, (uint8_t)0, (uint8_t)3));//Multi-segment location operation mode Sequential operation (P11-01 for selection of segment number)
    eControlMode = Speed ; 
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    processListOfCommands(list_of_commands, sendFunction); 
    DEBUG_SERIAL_PRINTLN("*****************Config for modbus control*****************");
    return list_of_commands;
}
bool LCDA630P::enable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    DEBUG_SERIAL_PRINTLN("*****************Disable*****************");
    std::vector<uint8_t> command = write_parameter(slave_id, (uint8_t)0x31, (uint8_t)0, (uint8_t)1);
    std::vector<uint8_t> response = sendFunction(command) ;
    bool value = parseModbusResponse(response);
    DEBUG_SERIAL_PRINTLN("*****************Disable*****************");
    return value;
}
bool LCDA630P::disable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction)
{
    DEBUG_SERIAL_PRINTLN("*****************Disable*****************");
    std::vector<uint8_t> command = write_parameter(slave_id, (uint8_t)0x31, (uint8_t)0, (uint8_t)0);
    std::vector<uint8_t> response = sendFunction(command) ;
    bool value = parseModbusResponse(response);
    DEBUG_SERIAL_PRINTLN("*****************Disable*****************");
    return value;
}
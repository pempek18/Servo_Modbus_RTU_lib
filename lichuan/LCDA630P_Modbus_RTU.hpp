#ifndef LCDA630P_MODBUS_RTU_HPP
#define LCDA630P_MODBUS_RTU_HPP

#define DEBUG_SERIAL false
#ifdef DEBUG_SERIAL
#include <iostream>
#define DEBUG_SERIAL_PRINTLN(x) std::cout << x << std::endl;
#define DEBUG_SERIAL_PRINT(x) std::cout << x;
#else
#define DEBUG_SERIAL_PRINTLN(x)
#define DEBUG_SERIAL_PRINT(x)
#endif

#include <cstdint>
#include <vector>
#include <iomanip>
#include <sstream>
#include <functional>

enum servomode 
{
    Speed,
    Position,
    Torque,
    TorqueSpeed,
    SpeedPosition,
    TorquePosition,
    Hybrid
};

class LCDA630P_Modbus_RTU
{
    private : 
        //inital parameters
        uint16_t MotorNumber ; 
        uint16_t RatedVoltage ;
        uint16_t RatedPower ;
        uint16_t RatedCurrent ;
        uint16_t RatedTorque ;
        uint16_t MaxTorque ;
        uint16_t RatedSpeed ;
        uint16_t MaxSpeed ;
        uint32_t PositionOffsetOfAbsolutEncoder ;
        uint16_t controlOverModbus;
        bool lower16_bit_first = true ; 
        //working parameters
        int64_t ActualAbsolutePosition;
        int16_t ActualSpeedRpm ; 
        servomode eControlMode ; 
    
    public :
        LCDA630P_Modbus_RTU();
        void scan_devices();
        /// @brief Debug function to show the frame
        /// @param frame frame itself
        /// @param print print enable
        void debug_print_frame(std::vector<uint8_t> frame, bool print);
        /// @brief Read specific parameter form servo 
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param size size of frame, at least 8 byte
        /// @return void
        std::vector<uint8_t> read_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, uint8_t size=2 );
        /// @brief Read specific parameter form servo with given send function
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param sendFunction provide function for sending RS-485
        /// @param size size of frame, at least 8 byte
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> read_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction, uint8_t size=2);
        /// @brief Write specific parameter to servo
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param value 
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> write_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int16_t value);
        /// @brief Write specific parameter to servo with given send function
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param value value to write
        /// @param sendFunction provide function for sending RS-485
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> write_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int16_t value, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Write specific parameter 32 bit to servo
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param value 
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> write_parameter_32(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int32_t value);        
        /// @brief  Write specific parameter 32 bit to servo with given send function
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param value value to write
        /// @param sendFunction provide function for sending RS-485
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> write_parameter_32(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int32_t value, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);        
        /// @brief Read all important parameters from servo and store in the instance of class variables
        /// @param slave_id Servo addres
        /// @param sendFunction provide function for sending RS-485
        /// @return multiple vector of byte
        std::vector<std::vector<uint8_t>>  read_servo_brief(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Read actual absoulte positon from servo encoder
        /// @param slave_id Servo addres
        /// @param sendFunction provide function for sending RS-485
        /// @return actual position
        int64_t get_actual_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Get actual speed of servo rotating
        /// @param slave_id Servo addres
        /// @param sendFunction  provide function for sending RS-485
        /// @return actual position
        int16_t get_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Sample from manual how to make one rotation without any functions 
        /// @param slave_id Servo addres
        /// @return vector of mupliple frames
        std::vector<std::vector<uint8_t>>  raw_one_rotation(uint8_t slave_id);
        /// @brief Control servo in speed mode
        /// @param slave_id Servo addres
        /// @param position number of increments to move
        /// @param sendFunction provide function for sending RS-485
        /// @return  vector of mupliple frames
        std::vector<std::vector<uint8_t>>  moveRelative(uint8_t slave_id, int32_t position,  std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Control servo in speed mode
        /// @param slave_id Servo addres
        /// @param speed setpoint speed in RPM, max 6000 set > 0 to go direction incrementic encoder and < 0 decrementig
        /// @param sendFunction provide function for sending RS-485
        /// @return vector of mupliple frames
        std::vector<std::vector<uint8_t>>  moveVelocity(uint8_t slave_id, int32_t speed, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Control servo in speed mode
        /// @param slave_id Servo addres
        /// @param torque setpoint torque in Nm
        /// @param sendFunction provide function for sending RS-485
        /// @return  vector of mupliple frames
        std::vector<std::vector<uint8_t>>  set_torque(uint8_t slave_id, int32_t torque, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Setup servodrive to working in position mode 
        /// @param slave_id Servo addres
        /// @param sendFunction provide function for sending RS-485
        /// @return vector of mupliple frames
        std::vector<std::vector<uint8_t>>  config_for_modbus_control_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Setup servodrive to working in speed mode 
        /// @param slave_id Servo addres
        /// @param sendFunction provide function for sending RS-485
        /// @return  vector of mupliple frames      
        std::vector<std::vector<uint8_t>>  config_for_modbus_control_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Send Disable to servo to take off power form motor 
        /// @param slave_id Servo addres
        /// @param sendFunction provide function for sending RS-485
        /// @return confirmation of servo beeing disable
        bool disable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Convert a vector of bytes that make up one frame to string 
        /// @param frame to send
        /// @return string 
        std::string vector_to_string(std::vector<uint8_t> frame);
        /// @brief parse response and print as readable on command line, return response value
        /// @param frame given in response of send function 
        /// @return 
        uint32_t parseModbusResponse(const std::vector<uint8_t>& response) ;
        /// @brief Calculatr crc for send request
        /// @param data frame without data
        /// @param length size of data
        /// @return crc
        uint16_t crcValueCalc(const uint8_t *data, uint16_t length);
        /// @brief check is servo setup to be controlled over modbus
        /// @return true or false
        bool controledOverModbus();

};

#endif // LCDA630P_MODBUS_RTU_HPP
#ifndef LCDA6_MODBUS_RTU_HPP
#define LCDA6_MODBUS_RTU_HPP

#define DEBUG_SERIAL true
#if (DEBUG_SERIAL == true)
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

class LCDA6_Modbus_RTU
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
        int64_t encoder_resolution = 8388600 ;
        int64_t pulse_per_rotation = 10000 ;
        bool lower16_bit_first = true ;
        //Absolute position stored in instance
        int64_t ActualAbsolutePosition;
        int16_t ActualSpeedRpm ;
        servomode eControlMode ;

    public :
        LCDA6_Modbus_RTU();
        void scan_devices();
        /// @brief Debug function to show the frame
        /// @param frame frame itself
        /// @param print print enable
        void debug_print_frame(std::vector<uint8_t> frame, bool print=true);
        /// @brief poin address of list of command, to process them
        /// @param listOfCommands Addres of sequence of frames
        /// @param sendFunction provide function for sending RS-485
        /// @return values in response to list of command
        std::vector<int32_t> processListoOfCommands(std::vector<std::vector<uint8_t>>& listOfCommands,  std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Read specific parameter form servo
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param size size of frame, at least 8 byte
        /// @return void
        std::vector<uint8_t> read_parameter(uint8_t slave_id, uint16_t address, uint16_t size=1 );
        /// @brief Read specific parameter form servo with given send function
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param sendFunction provide function for sending RS-485
        /// @param size size of frame, at least 8 byte
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> read_parameter(uint8_t slave_id, uint16_t address, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction, uint16_t size=1);
        /// @brief Write specific parameter to servo
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param value
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> write_parameter(uint8_t slave_id, uint16_t address, int16_t value);
        /// @brief Write specific parameter to servo with given send function
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param value value to write
        /// @param sendFunction provide function for sending RS-485
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> write_parameter(uint8_t slave_id, uint16_t address, int16_t value, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Write specific parameter 32 bit to servo
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param value
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> write_parameter_32(uint8_t slave_id, uint16_t address, int32_t value);
        /// @brief  Write specific parameter 32 bit to servo with given send function
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param value value to write
        /// @param sendFunction provide function for sending RS-485
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> write_parameter_32(uint8_t slave_id, uint16_t address, int32_t value, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);


        std::vector<std::vector<uint8_t>> LCDA6_Modbus_RTU::servo_config(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
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
        /// @brief Move absolut to position based on encoder position
        /// @param slave_id Servo addres
        /// @param position number of increments to move
        /// @param sendFunction provide function for sending RS-485
        /// @return vector of mupliple frames
        int64_t moveAbsolute(uint8_t slave_id, int64_t position,  std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
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
        int32_t parseModbusResponse(const std::vector<uint8_t>& response) ;
        /// @brief Calculatr crc for send request
        /// @param data frame without data
        /// @param length size of data
        /// @return crc
        uint16_t crcValueCalc(const uint8_t *data, uint16_t length);
        /// @brief check is servo setup to be controlled over modbus
        /// @return true or false
        // bool controledOverModbus();

};

#endif // LCDA6_MODBUS_RTU_HPP
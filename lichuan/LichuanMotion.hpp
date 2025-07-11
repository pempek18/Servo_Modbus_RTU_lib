#ifndef LICHUAN_MOTION
#define LICHUAN_MOTION

#define DEBUG_SERIAL true
#if (DEBUG_SERIAL == true)
    #include <iostream>
    #define DEBUG_SERIAL_PRINTLN(x) std::cout << x << std::endl;
    #define DEBUG_SERIAL_PRINT(x) std::cout << x;
    #define DEBUG_SERIAL_PRINTF(x, ...) std::cout << x << __VA_ARGS__ << std::endl;
#else
    #define DEBUG_SERIAL_PRINTLN(x)
    #define DEBUG_SERIAL_PRINT(x)
    #define DEBUG_SERIAL_PRINTF(x, ...)
#endif

#include <cstdint>
#include <vector>
#include <iomanip>
#include <sstream>
#include <functional>
#include <cmath>
#include <optional>
#include <chrono>
#include <thread>

union Converter64 {
    int64_t as_int64;
    int32_t as_int32[2];
    int16_t as_int16[4];
    uint8_t as_uint8[8];
};

enum servomode
{
    undefined,
    Speed,
    Position,
    Torque,
    TorqueSpeed,
    SpeedPosition,
    TorquePosition,
    Hybrid
};

class LichuanMotion
{
    public:
    // Assign send function
    std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction = NULL;
    void onSend(std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> callable){
        sendFunction = callable;
    }

    protected :
        uint8_t slave_id;
        Converter64 converter;
    public :
        LichuanMotion();
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
        int64_t encoder_resolution = 8388608 ;
        int64_t pulse_per_rotation = 10000 ;
        bool lower16_bit_first = true ;
        //Absolute position stored in instance
        int64_t ActualAbsolutePosition;
        int64_t ActualPulseCounterPosition ;
        int16_t ActualSpeedRpm ;
        servomode eControlMode ;
        void scan_devices();
        /// @brief Debug function to show the frame
        /// @param frame frame itself
        /// @param print print enable
        void debug_print_frame(std::vector<uint8_t> frame, bool print=true);
        /// @brief poin address of list of command, to process them
        /// @param listOfCommands Addres of sequence of frames
        /// @param sendFunction provide function for sending RS-485
        /// @return values in response to list of command
        std::vector<int32_t> processListOfCommands(std::vector<std::vector<uint8_t>>& listOfCommands,  std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        /// @brief Read specific parameter form servo with given send function
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param sendFunction provide function for sending RS-485
        /// @param size size of frame, at least 8 byte
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> read_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, uint8_t size=2, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)>> sendFunction = std::nullopt);
        /// @brief Read specific parameter form servo with given send function
        /// @param slave_id Servo addres
        /// @param address address of parameter
        /// @param size size of frame, at least 8 byte
        /// @param sendFunction provide function for sending RS-485
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> read_parameter(uint8_t slave_id, uint16_t address, uint16_t size=2, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)>> sendFunction = std::nullopt);
        /// @brief Write specific parameter to servo with given send function
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param value value to write
        /// @param sendFunction provide function for sending RS-485
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> write_parameter(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int16_t value, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)>> sendFunction = std::nullopt);
        /// @brief Write specific parameter to servo with given send function
        /// @param slave_id Servo addres
        /// @param address address of parameter
        /// @param value value to write
        /// @param sendFunction provide function for sending RS-485
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> write_parameter(uint8_t slave_id, uint16_t address, int16_t value, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)>> sendFunction = std::nullopt);
        /// @brief  Write specific parameter 32 bit to servo with given send function
        /// @param slave_id Servo addres
        /// @param group_number PXX-YY - XX - group number
        /// @param parameter_offset PXX-YY - YY - parameter offset
        /// @param value value to write
        /// @param sendFunction provide function for sending RS-485
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> write_parameter_32(uint8_t slave_id, uint8_t group_number, uint8_t parameter_offset, int32_t value, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)>> sendFunction = std::nullopt);
        /// @brief  Write specific parameter 32 bit to servo with given send function
        /// @param slave_id Servo addres
        /// @param address address of parameter
        /// @param value value to write
        /// @param sendFunction provide function for sending RS-485
        /// @return a vector of bytes that make up one frame
        std::vector<uint8_t> write_parameter_32(uint8_t slave_id, uint16_t address, int32_t value, std::optional<std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)>> sendFunction = std::nullopt);
        /// @brief Read all important parameters from servo and store in the instance of class variables
        /// @param slave_id Servo addres
        /// @param sendFunction provide function for sending RS-485
        /// @return multiple vector of byte
        virtual std::vector<std::vector<uint8_t>>  read_servo_brief(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction) = 0;
        /// @brief Read actual absoulte positon from servo encoder
        /// @param slave_id Servo addres
        /// @param sendFunction provide function for sending RS-485
        /// @return actual position
        virtual int64_t get_actual_mechanical_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction) = 0;
        /// @brief Read actual absoulte positon from Input instruction pulse counter
        /// @param slave_id Servo addres
        /// @param sendFunction provide function for sending RS-485
        /// @return actual position
        virtual int64_t get_actual_pulse_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction) = 0;
        /// @brief Get actual speed of servo rotating
        /// @param slave_id Servo addres
        /// @param sendFunction  provide function for sending RS-485
        /// @return actual position
        virtual int16_t get_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction) = 0;
        /// @brief Sample from manual how to make one rotation without any functions
        /// @param slave_id Servo addres
        /// @return vector of mupliple frames
        virtual std::vector<std::vector<uint8_t>>  raw_one_rotation(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction) = 0;
        /// @brief Control servo in speed mode
        /// @param slave_id Servo addres
        /// @param position number of increments to move
        /// @param sendFunction provide function for sending RS-485
        /// @return  vector of mupliple frames
        virtual std::vector<std::vector<uint8_t>>  moveRelative(uint8_t slave_id, int32_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction, int32_t speed = 1000, float torque = 10.0) = 0;
        /// @brief Move absolut to position based on encoder position
        /// @param slave_id Servo addres
        /// @param position number of increments to move
        /// @param sendFunction provide function for sending RS-485
        /// @return vector of mupliple frames
        virtual int64_t moveAbsolute(uint8_t slave_id, int64_t position,  std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction, int32_t speed = 1000, float torque = 10.0) = 0;
        /// @brief Control servo in speed mode
        /// @param slave_id Servo addres
        /// @param speed setpoint speed in RPM, max 6000 set > 0 to go direction incrementic encoder and < 0 decrementig
        /// @param sendFunction provide function for sending RS-485
        /// @return vector of mupliple frames
        virtual std::vector<std::vector<uint8_t>>  moveVelocity(uint8_t slave_id, int32_t speed, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction) = 0;
        /// @brief Control servo in speed mode
        /// @param slave_id Servo addres
        /// @param torque setpoint torque in Nm
        /// @param sendFunction provide function for sending RS-485
        /// @return  vector of mupliple frames
        virtual std::vector<std::vector<uint8_t>>  set_torque(uint8_t slave_id, float torque, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction) = 0;
        /// @brief Setup servodrive to working in position mode
        /// @param slave_id Servo addres
        /// @param sendFunction provide function for sending RS-485
        /// @return vector of mupliple frames
        virtual std::vector<std::vector<uint8_t>>  config_for_modbus_control_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction) = 0;
        /// @brief Setup servodrive to working in speed mode
        /// @param slave_id Servo addres
        /// @param sendFunction provide function for sending RS-485
        /// @return  vector of mupliple frames
        virtual std::vector<std::vector<uint8_t>>  config_for_modbus_control_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction) = 0;
        /// @brief Send Enable to servo to take on power form motor
        /// @param slave_id Servo addres
        /// @param sendFunction provide function for sending RS-485
        /// @return confirmation of servo beeing enable
        virtual bool enable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction) = 0;
        /// @brief Send Disable to servo to take off power form motor
        /// @param slave_id Servo addres
        /// @param sendFunction provide function for sending RS-485
        /// @return confirmation of servo beeing disable
        virtual bool disable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction) = 0;
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
        bool controledOverModbus();

};

#endif // LICHUAN_MOTION
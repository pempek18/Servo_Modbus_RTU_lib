#ifndef LCDA6_HPP
#define LCDA6_HPP

#include "LichuanMotion.hpp"

enum class servomode : uint8_t
{
    Position = 0,
    Speed = 1,
    Torque = 2,
    PositionSpeed = 3,
    PositionTorque = 4,
    SpeedTorque = 5,
    CanOpen = 10
};

class LCDA6 : public LichuanMotion
{
    private:
        servomode eControlMode = servomode::Position;
        int8_t ModeToInt(servomode mode);
    public :
        LCDA6();
        ~LCDA6();
        std::vector<std::vector<uint8_t>>  read_servo_brief(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);

        std::vector<std::vector<uint8_t>>  config_for_modbus_control_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        std::vector<std::vector<uint8_t>>  config_for_modbus_control_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        std::vector<std::vector<uint8_t>>  config_for_modbus_control_torque(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        std::vector<std::vector<uint8_t>>  config_for_modbus_control_position_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);

        bool enable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        bool disable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);

        int64_t get_actual_mechanical_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction);
        int64_t get_actual_pulse_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction);
        std::vector<std::vector<uint8_t>>  moveRelative(uint8_t slave_id, int32_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction, int32_t speed = 1000, float torque = 10.0);
        int64_t moveAbsolute(uint8_t slave_id, int64_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction, int32_t speed = 1000, float torque = 10.0);

        std::vector<std::vector<uint8_t>> set_speed(uint8_t slave_id, int32_t speed, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction);
        int16_t get_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction);
        std::vector<std::vector<uint8_t>>  moveVelocity(uint8_t slave_id, int32_t speed, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction);

        int16_t get_torque(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction);
        std::vector<std::vector<uint8_t>>  set_torque(uint8_t slave_id, float torque, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);

        std::vector<std::vector<uint8_t>>  raw_one_rotation(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);

        bool inTargetPosition(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        bool inTargetSpeed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);

        std::vector<std::pair<uint16_t, uint16_t>> get_output_state(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);

        int64_t encoder_resolution = 131072 ;
        int64_t pulse_per_rotation = 10000 ;
};

#endif // LCDA6_HPP
#ifndef LCDA6_HPP
#define LCDA6_HPP

#include "LichuanMotion.hpp"

class LCDA6 : public LichuanMotion
{
    private:
        int8_t ModeToInt(servomode mode);
    public :
        LCDA6();
        ~LCDA6();
        std::vector<std::vector<uint8_t>>  read_servo_brief(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);

        std::vector<std::vector<uint8_t>>  config_for_modbus_control_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        std::vector<std::vector<uint8_t>>  config_for_modbus_control_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        std::vector<std::vector<uint8_t>>  config_for_modbus_control_torque(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);

        bool enable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);
        bool disable(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);

        int64_t get_actual_mechanical_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction);
        int64_t get_actual_pulse_position(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction);
        std::vector<std::vector<uint8_t>>  moveRelative(uint8_t slave_id, int32_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction, int32_t speed = 1000, float torque = 10.0);
        int64_t moveAbsolute(uint8_t slave_id, int64_t position, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction, int32_t speed = 1000, float torque = 10.0);

        int16_t get_speed(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction);
        std::vector<std::vector<uint8_t>>  moveVelocity(uint8_t slave_id, int32_t speed, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction);

        int16_t get_torque(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction);
        std::vector<std::vector<uint8_t>>  set_torque(uint8_t slave_id, float torque, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);

        std::vector<std::vector<uint8_t>>  raw_one_rotation(uint8_t slave_id, std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)> sendFunction);

        int32_t parseModbusResponse(const std::vector<uint8_t>& response) override;
};

#endif // LCDA6_HPP
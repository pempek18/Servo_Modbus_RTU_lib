#ifndef LICHUAN_LCDA630P_HPP
#define LICHUAN_LCDA630P_HPP

#pragma once

#include "../LICHUAN_RTU.hpp"

class LICHUAN_LCDA630P : public I_Lichuan_RTU {
    private :
        /* Additional parameters */
    public :
        LICHUAN_LCDA630P(std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> sendFunction);
        inline uint16_t Param2Address(uint8_t group, uint8_t param) { return (group << 8) | param; }

        // Configuration
        std::vector<std::vector<uint8_t>> servo_config() override;
        std::vector<std::vector<uint8_t>> read_servo_brief() override;

        // Enable / Disable Servo
        bool enable() override;
        bool disable() override;

        // Speed
        std::vector<std::vector<uint8_t>> config_for_modbus_control_speed() override;
        int64_t get_actual_speed() override;
        std::vector<std::vector<uint8_t>> set_speed(int64_t speed) override;
        std::vector<std::vector<uint8_t>> move_velocity(int64_t speed) override;

        // Position
        std::vector<std::vector<uint8_t>> config_for_modbus_control_position() override;
        int64_t get_actual_position() override;
        std::vector<std::vector<uint8_t>> set_position(int64_t position) override;
        std::vector<std::vector<uint8_t>> move_relative(int64_t position) override;
        std::vector<std::vector<uint8_t>> move_absolute(int64_t position) override;

        // Torque
        std::vector<std::vector<uint8_t>> config_for_modbus_control_torque() override;
        int64_t get_actual_torque() override;
        std::vector<std::vector<uint8_t>> set_torque(int64_t torque) override;

        // Debug
        std::vector<std::vector<uint8_t>> raw_one_rotation() override;
};

#endif // LICHUAN_LCDA6_HPP
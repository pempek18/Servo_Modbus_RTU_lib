#ifndef LCDA630P_MODBUS_RTU_HPP
#define LCDA630P_MODBUS_RTU_HPP

#include <cstdint>

class LCDA630P_Modbus_RTU
{
    public :
        LCDA630P_Modbus_RTU();
        void scan_devices();
        uint16_t crcValueCalc(const uint16_t *data, uint16_t length);
};

#endif // LCDA630P_MODBUS_RTU_HPP
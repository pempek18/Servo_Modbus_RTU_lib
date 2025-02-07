#include <iostream>

#include "MB/modbusException.hpp"
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
// #include "lichuan/LCDA630P_Modbus_RTU.hpp"

int main() {
    MB::ModbusRequest request(1, MB::utils::ReadDiscreteOutputCoils, 100, 10);
    // LCDA630P_Modbus_RTU servo;

    std::cout << "Start Lichuan servo app" << std::endl;
    return 0;
}
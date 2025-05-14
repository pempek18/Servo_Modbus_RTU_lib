#include "LICHUAN_RTU.hpp"

std::vector<int32_t> I_Lichuan_RTU::processListoOfCommands(std::vector<std::vector<uint8_t>> &listOfCommands){
    std::vector<int32_t> values;
    for (std::vector<uint8_t> command : listOfCommands) {
        std::vector<uint8_t> feedback = this->sendFunction(command) ;
        int32_t response = this->parseModbusResponse(feedback) ;
        values.push_back(response);
    }
    return values ;
}

std::string I_Lichuan_RTU::vector_to_string(std::vector<uint8_t> frame){
    // Convert uint16_t array to string
    std::string request_string = "";

    for (int i = 0; i < frame.size() ; i++){
        request_string += static_cast<char>(frame[i]);        // Get low byte
    }
    return request_string ;
}

void I_Lichuan_RTU::debug_print_frame(std::vector<uint8_t> frame, bool print){
    if (!print)
        return;

    for (int i = 0; i < frame.size(); i++){
        std::stringstream ss;
        ss << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(frame[i]) << " ";
        DEBUG_SERIAL_PRINT(ss.str().c_str());
    }
    DEBUG_SERIAL_PRINTLN("");
}

std::vector<uint8_t> I_Lichuan_RTU::read_parameter(uint16_t address, uint16_t size){
    std::vector<uint8_t> frame;
    frame.push_back(this->slave_id);            // Slave ID
    frame.push_back(0x03);                      // Read Holding Register
    frame.push_back(address >> 8);              // AddrH - Higher 8 bits
    frame.push_back(address & 0xFF);            // AddrL - Lower 8 bits
    frame.push_back(size >> 8);
    frame.push_back(size & 0xFF);

    // Calculate CRC using uint8_t data
    uint16_t crc = crcValueCalc(frame.data(), frame.size());
    frame.push_back( crc       & 0xFF);         // Low byte
    frame.push_back((crc >> 8) & 0xFF);         // High byte

    #if DEBUG_SERIAL
        debug_print_frame(frame, true);
    #endif
    return frame;
}

std::vector<uint8_t> I_Lichuan_RTU::write_parameter(uint16_t address, int16_t value){
    std::vector<uint8_t> frame;
    frame.push_back(this->slave_id);            // Slave ID
    frame.push_back(0x06);                      // Write Holding Register
    frame.push_back(address >> 8);              // AddrH - Higher 8 bits
    frame.push_back(address & 0xFF);            // AddrL - Lower 8 bits
    frame.push_back(value >> 8);
    frame.push_back(value & 0xFF);

    // Calculate CRC using uint8_t data
    uint16_t crc = crcValueCalc(frame.data(), frame.size());
    frame.push_back( crc       & 0xFF);         // Low byte
    frame.push_back((crc >> 8) & 0xFF);         // High byte

    #if DEBUG_SERIAL
        debug_print_frame(frame, true);
    #endif
    return frame;
}

std::vector<uint8_t> I_Lichuan_RTU::write_parameter_32(uint16_t address, int32_t value){
    std::vector<uint8_t> frame;
    frame.push_back(this->slave_id);            // Slave ID
    frame.push_back(0x10);                      // Write Holding Register
    frame.push_back(address >> 8);              // AddrH - Higher 8 bits
    frame.push_back(address & 0xFF);            // AddrL - Lower 8 bits
    frame.push_back(0x00);                      // The high 8 bits of the function code are M(H), and the length of a 32-bit function code is 2.
    frame.push_back(0x02);                      // Function code number lower 8 digits M(L)
    frame.push_back(0x04);                      // The number of function codes corresponds ti the number of bytes M*2. For examole, if P05-07 is written alone, DATA[4] is P04
    if (lower16_bit_first)
    {
        frame.push_back((value >> 8 ) & 0xFF);  // Write the high  8 bits of the start function code, hex
        frame.push_back( value        & 0xFF);  // Write the lower 8 bits of the start function code, hex
        frame.push_back((value >> 24) & 0xFF);  // Write the high  8 bits of the start function code group offset + 1, hex
        frame.push_back((value >> 16) & 0xFF);  // Write the low   8 bits of the start function code group offset + 1, hex
    }else
    {
        frame.push_back((value >> 24) & 0xFF);  // Write the high  8 bits of the start function code group offset + 1, hex
        frame.push_back((value >> 16) & 0xFF);  // Write the low   8 bits of the start function code group offset + 1, hex
        frame.push_back((value >> 8 ) & 0xFF);  // Write the high  8 bits of the start function code, hex
        frame.push_back( value        & 0xFF);  // Write the lower 8 bits of the start function code, hex
    }

    // Calculate CRC using uint8_t data
    uint16_t crc = crcValueCalc(frame.data(), frame.size());
    frame.push_back( crc       & 0xFF);         // Low byte
    frame.push_back((crc >> 8) & 0xFF);         // High byte

    #if DEBUG_SERIAL
        debug_print_frame(frame, true);
    #endif
    return frame;
}

uint16_t I_Lichuan_RTU::crcValueCalc(const uint8_t *data, uint16_t length){
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++){
        crc ^= data[i];
        for (uint16_t j = 0; j < 8; j++){
            if (crc & 0x0001){
                crc >>= 1;
                crc ^= 0xA001;
            }
            else{
                crc >>= 1;
            }
        }
    }
    return crc;
}

int32_t I_Lichuan_RTU::parseModbusResponse(const std::vector<uint8_t> &response){
    int16_t LSB, MSB;

    (lower16_bit_first) ? LSB = 8, MSB = 0 : LSB = 0, MSB = 8;

    // if(lower16_bit_first){ LSB = 8; MSB = 0; }
    // else{                  LSB = 0; MSB = 8; }

    if (response.size() < 7) {
        return 0xFFFFFFFF ;
    }
    int16_t ID  = static_cast<int>(response[0]);
    int16_t fun = static_cast<int>(response[1]);

    std::stringstream ss ;
    ss << std::hex << std::setfill('0') << std::setw(2)
        << "ID: " << ID
        << "\tf :" << fun;

    switch (fun) {
        case 0x03:{ // Read Holding Register
            int16_t len = static_cast<int16_t>(response[2]);
            ss << "\tbytes: " << std::dec << len;

            int16_t val;
            for(int i = 0; i < len; i+=2){
                val = (static_cast<int16_t>(response[3+2*i]) << LSB)  | (static_cast<int16_t>(response[4+2*i]) << MSB);
                ss << "\n\tvalue: "<< val
                   << "\thex: "  << std::hex << "0x" << std::setfill('0') << std::setw(2) << static_cast<int>(val);
            }
            ss << std::endl;
            DEBUG_SERIAL_PRINT(ss.str().c_str());
            return val;
        }
        case 0x06:{ // Write Holding Register
            int16_t addr = (static_cast<int16_t>(response[2]) << LSB)  | (static_cast<int16_t>(response[3]) << MSB);
            ss << "\taddr: "<< std::hex << "0x" << std::setfill('0') << std::setw(2) << static_cast<int>(addr);

            int16_t val =  (static_cast<int16_t>(response[4]) << LSB)  | (static_cast<int16_t>(response[5]) << MSB);
            ss << "\tvalue: "<< std::dec << val
               << "\thex: "  << std::hex << "0x" << std::setfill('0') << std::setw(2) << static_cast<int>(val)
               << std::endl;
            DEBUG_SERIAL_PRINT(ss.str().c_str());
            return val;
        }
        case 0x10: //TODO: Write Multiple Holding Registers
        default:
            return 0xFFFFFFFF;
            break;
    }
    return 0xFFFFFFFF;
}

std::vector<uint8_t> I_Lichuan_RTU::send_and_receive(std::vector<uint8_t> frame){
    //TODO Check if this should return frame as in the original class
    //TODO or should it return the response

    std::vector<int32_t> values;
    std::vector<uint8_t> feedback = sendFunction(frame) ;
    int32_t response = parseModbusResponse(feedback) ;
    values.push_back(response);
    return frame;
}


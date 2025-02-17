#include <iostream>
#include <iomanip>
#include <boost/asio.hpp>

#include "MB/modbusException.hpp"
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "lichuan/LCDA630P_Modbus_RTU.hpp"

std::vector<uint8_t> show_modbus_frame();
void send_request_over_serial(std::string request);
std::vector<uint8_t> send(std::string request);
std::string vector_to_string(std::vector<uint8_t>);
int main()
{
    LCDA630P_Modbus_RTU servo;
    servo.scan_devices();
    std::vector<uint8_t> p02_2 = servo.read_parameter(1, 2, 2, 2);
    std::vector<uint8_t> frame2 = servo.write_parameter(1, 2, 2, 2);
    std::vector<uint8_t> frame3 = servo.write_parameter_32(1, 5, 7, 123);
    std::cout << "*****************For testing*****************" << std::endl;
    std::vector<uint8_t> request = show_modbus_frame();
    // Convert request.toRaw() to string
    std::string request_string_from_modbus = vector_to_string(request);
    std::cout << "*****************For testing*****************" << std::endl;
    std::cout << "*****************Read & Write*****************" << std::endl;
    send_request_over_serial(request_string_from_modbus);
    std::cout << "Read P02.04, write 50 and read again" << std::endl;
    std::vector<uint8_t> p02_4 = servo.read_parameter(1, 2, 4, 2);
    std::string p02_4_s = vector_to_string(p02_4);
    send_request_over_serial(p02_4_s);
    p02_4 = servo.write_parameter(1, 2, 4, 50); // manually cannot write to 
    p02_4_s = vector_to_string(p02_4);
    send_request_over_serial(p02_4_s);
    std::cout << "*****************Read & Write*****************" << std::endl;
    std::cout << "*****************Read Brief*****************" << std::endl;
    std::vector<std::vector<uint8_t>> params = servo.read_servo_brief(1);
    for (std::vector<uint8_t> param : params)
    {
        std::string param_s = vector_to_string(param);
        std::vector<uint8_t> feedback =  send(param_s);
        std::pair<int,int> response = servo.parseModbusResponse(feedback, true) ;
    }
    std::cout << "*****************Read Brief*****************" << std::endl;
    return 0;
}
std::string vector_to_string(std::vector<uint8_t> frame)
{
    // Convert uint16_t array to string
    std::string request_string = "";
    for (int i = 0; i < 8; i++)
    {
        request_string += static_cast<char>(frame[i]);        // Get low byte
    }
    return request_string ;
}
std::vector<uint8_t> show_modbus_frame()
{
    // Create simple request
    MB::ModbusRequest request(1, MB::utils::ReadAnalogOutputHoldingRegisters, 0x0202, 2);

    std::cout << "Stringed Request: " << request.toString() << std::endl;

    std::cout << "Raw request:" << std::endl;

    // Get raw represenatation for request
    std::vector<uint8_t> rawed = request.toRaw();

    // Method for showing byte
    auto showByte = [](const uint8_t &byte)
    {
        std::cout << " 0x" << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(byte);
    };

    // Show all bytes
    std::for_each(rawed.begin(), rawed.end(), showByte);
    std::cout << std::endl;

    // Create CRC and pointer to its bytes
    uint16_t CRC = MB::utils::calculateCRC(rawed);
    auto CRCptr = reinterpret_cast<uint8_t *>(&CRC);

    // Show byted CRC for request
    std::cout << "CRC for the above code: ";
    std::for_each(CRCptr, CRCptr + 2, showByte);
    std::cout << std::endl;

    rawed.insert(rawed.end(), CRCptr, CRCptr + 2);
    return rawed;
}
std::vector<uint8_t> send(std::string request)
{
    std::vector<uint8_t> response_vector ; 
    try
    {
        boost::asio::io_service io;
        boost::asio::serial_port serial(io, "/dev/ttyUSB0"); // Change to your port

        // Set serial port parameters
        serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

        boost::asio::write(serial, boost::asio::buffer(request));

        for (char c : request)
        {
            response_vector.push_back(c);
        }
        size_t n = boost::asio::read(serial, boost::asio::buffer(response_vector, 8));

        return response_vector ;
    }
    catch (std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return response_vector ;
    }
}
void send_request_over_serial(std::string request)
{
    std::cout << "Request bytes: ";
    for (char c : request) {
        std::cout << " 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(static_cast<unsigned char>(c));
    }
    std::cout << std::endl;
    std::cout << "Sending request: " << request << std::endl;
    try
    {
        boost::asio::io_service io;
        boost::asio::serial_port serial(io, "/dev/ttyUSB0"); // Change to your port

        // Set serial port parameters
        serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

        boost::asio::write(serial, boost::asio::buffer(request));

        // Example: Read response from the serial port
        char response[8];
        size_t n = boost::asio::read(serial, boost::asio::buffer(response, sizeof(response)));
        std::cout << "Response: " << std::string(response, n) << std::endl;
        for (char c : response) {
            std::cout << " 0x" << std::hex << std::setw(2) << std::setfill('0') 
            << static_cast<int>(static_cast<unsigned char>(c));
        }
        std::cout << std::endl;
    }
    catch (std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}
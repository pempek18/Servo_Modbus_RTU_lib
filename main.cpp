#include <iostream>
#include <iomanip>
#include <boost/asio.hpp>
#include "lichuan/LCDA630P_Modbus_RTU.hpp"

void send_request_over_serial(std::string request);
std::vector<uint8_t> send(std::string request);
int main()
{
    LCDA630P_Modbus_RTU servo;
    servo.scan_devices();
    std::cout << "*****************Read Brief*****************" << std::endl;
    std::vector<std::vector<uint8_t>> params = servo.read_servo_brief(1);
    for (std::vector<uint8_t> param : params)
    {
        std::string param_s = servo.vector_to_string(param);
        std::vector<uint8_t> feedback =  send(param_s);
        std::pair<int,int> response = servo.parseModbusResponse(feedback) ;
    }
    std::cout << "*****************Read Brief*****************" << std::endl;
    std::cout << "*****************Read P0C-26*****************" << std::endl;
    std::vector<uint8_t> p0c_26 = servo.read_parameter(1,12,26,2);
    std::string p0c_26_s = servo.vector_to_string(p0c_26);
    std::vector<uint8_t> p0c_26_r = send(p0c_26_s);
    std::pair<int,int> p0c_26_f = servo.parseModbusResponse(p0c_26_r);
    std::cout << "*****************Read P0C-26*****************" << std::endl;
    std::cout << "*****************Move one rotation*****************" << std::endl;
    std::vector<std::vector<uint8_t>> one_rot = servo.test_one_rotation(1);
    for (std::vector<uint8_t> param : one_rot)
    {
        std::string param_s = servo.vector_to_string(param);
        send(param_s);
    }
    std::cout << "*****************Move one rotation*****************" << std::endl;
    return 0;
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
        std::cout << "request send: " ;

        for (char c : request)
        {
            response_vector.push_back(c);
            std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(c) << " ";
        }
        std::cout << std::endl;

        int size_of_frame = 8;
        if (request[1] == 0x10)
            size_of_frame = 8;
        else 
            size_of_frame = 8;

        size_t n = boost::asio::read(serial, boost::asio::buffer(response_vector, size_of_frame));

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
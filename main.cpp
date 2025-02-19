#include <iostream>
#include <iomanip>
#include <boost/asio.hpp>
#include "lichuan/LCDA630P_Modbus_RTU.hpp"

void send_request_over_serial(std::string request);
std::vector<uint8_t> send(std::string request);
std::string vector_to_string(std::vector<uint8_t>);
int main()
{
    LCDA630P_Modbus_RTU servo;
    servo.scan_devices();
    std::cout << "*****************Read Brief*****************" << std::endl;
    std::vector<std::vector<uint8_t>> params = servo.read_servo_brief(1);
    for (std::vector<uint8_t> param : params)
    {
        std::string param_s = vector_to_string(param);
        std::vector<uint8_t> feedback =  send(param_s);
        std::pair<int,int> response = servo.parseModbusResponse(feedback) ;
    }
    std::cout << "*****************Read Brief*****************" << std::endl;
    std::cout << "*****************Move one rotation*****************" << std::endl;
    std::vector<std::vector<uint8_t>> one_rot = servo.test_one_rotation(1);
    for (std::vector<uint8_t> param : one_rot)
    {
        std::string param_s = vector_to_string(param);
        send(param_s);
    }
    std::cout << "*****************Move one rotation*****************" << std::endl;
    return 0;
}
std::string vector_to_string(std::vector<uint8_t> frame)
{
    // Convert uint16_t array to string
    std::string request_string = "";
    int size_of_frame = 8;
    if (frame[1] == 0x06)
        size_of_frame = 8;
    else if (frame[1] == 0x10)
        size_of_frame = 13;

    for (int i = 0; i < size_of_frame ; i++)
    {
        request_string += static_cast<char>(frame[i]);        // Get low byte
    }
    return request_string ;
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
        if (request[1] == 0x06)
            size_of_frame = 8;
        else if (request[1] == 0x10)
            size_of_frame = 13;

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
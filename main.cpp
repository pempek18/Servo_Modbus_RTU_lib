#include <iostream>
#include <iomanip>
#include <boost/asio.hpp>
#include "lichuan/LCDA630P_Modbus_RTU.hpp"

void send_request_over_serial(std::string request);
std::vector<uint8_t> send(std::string request);
void read_with_timeout(boost::asio::io_context &io, boost::asio::serial_port &serial, std::vector<uint8_t> &response_vector, int timeout_ms) ;
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
        uint32_t response = servo.parseModbusResponse(feedback) ;
    }
    std::cout << "*****************Read Brief*****************" << std::endl;
    std::cout << "*****************Read P0C-26*****************" << std::endl;
    std::vector<uint8_t> p0c_26 = servo.read_parameter(1,0x0C,0x26);
    std::string p0c_26_s = servo.vector_to_string(p0c_26);
    std::vector<uint8_t> p0c_26_r = send(p0c_26_s);
    uint32_t p0c_26_f = servo.parseModbusResponse(p0c_26_r);
    std::cout << "*****************Read P0C-26*****************" << std::endl;
    std::cout << "*****************Move one rotation RAW DATA*****************" << std::endl;
    std::vector<std::vector<uint8_t>> one_rot_raw = servo.raw_one_rotation(1);
    for (std::vector<uint8_t> param : one_rot_raw)
        servo.debug_print_frame(param, true);
    std::cout << "*****************Move one rotation RAW DATA*****************" << std::endl;
    std::cout << "*****************Read P05-00*****************" << std::endl;
    std::vector<uint8_t> p05_00 = servo.read_parameter(1,5,0);
    std::string p05_00_S = servo.vector_to_string(p05_00);
    std::vector<uint8_t> p05_00_v = send(p05_00_S);
    uint32_t p05_00_value = servo.parseModbusResponse(p05_00_v);
    std::cout << "*****************Read P05-00*****************" << std::endl;    
    if (p05_00_value != 2 )
    {
        std::cout << "*****************Config for modbus control*****************" << std::endl;
        std::vector<std::vector<uint8_t>> config = servo.config_for_modbus_control(1);
        for (std::vector<uint8_t> param : config)
        {
            std::string param_s = servo.vector_to_string(param);
            servo.parseModbusResponse(param);
            send(param_s);
        }  
        std::cout << "*****************Config for modbus control*****************" << std::endl;        
    }
    std::cout << "*****************Move to pos*****************" << std::endl;
    std::vector<std::vector<uint8_t>> one_rot = servo.move_to_position(1, -10000);
    for (std::vector<uint8_t> param : one_rot)
    {
        std::string param_s = servo.vector_to_string(param);
        std::vector<uint8_t> recive = send(param_s);
        servo.parseModbusResponse(recive);
    }    
    std::cout << "*****************Move to pos*****************" << std::endl;
    std::cout << "*****************Disable*****************" << std::endl;
    std::cout << "press any key to disable" << std::endl;
    std::cin.get() ;
    std::vector<uint8_t> disable = servo.write_parameter(1,0x31,0,0);
    std::string disable_s = servo.vector_to_string(disable);
    send(disable_s);
    std::cout << "*****************Disable*****************" << std::endl;
    return 0;
}
std::vector<uint8_t> send(std::string request)
{
    std::vector<uint8_t> response_vector(request.size()) ; 
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
        std::cout << "send: " ;

        for (char c : request)
        {
            std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(c) << " ";
        }
        std::cout << std::endl;

        int size_of_frame = 8;
        if (request[1] == 0x10)
            size_of_frame = 8;
        else 
            size_of_frame = 8;

        read_with_timeout(io, serial, response_vector, 100);

        std::cout << "recive: " ;

        for (char c : response_vector)
        {
            std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(c) << " ";
        }
        std::cout << std::endl;        

        return response_vector ;
    }
    catch (std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return response_vector ;
    }
}
void read_with_timeout(boost::asio::io_context &io, boost::asio::serial_port &serial, std::vector<uint8_t> &response_vector, int timeout_ms) {
    boost::asio::deadline_timer timer(io, boost::posix_time::milliseconds(timeout_ms));
    bool data_received = false;

    timer.async_wait([&](const boost::system::error_code &ec) {
        if (!ec) serial.cancel();  // Cancel read operation if timeout occurs
    });

    boost::asio::async_read(serial, boost::asio::buffer(response_vector),
        [&](const boost::system::error_code &ec, std::size_t length) {
            data_received = !ec;
            timer.cancel();  // Cancel timer if data is received
        });

    io.run(); // Run the I/O context

    if (!data_received) {
        std::cerr << "Timeout reached, no data received." << std::endl;
    }
};
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
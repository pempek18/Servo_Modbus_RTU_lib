#include <iostream>
#include <iomanip>
#include <boost/asio.hpp>
#include "LCDA630P_Modbus_RTU.hpp"

void send_request_over_serial(std::string request);
std::vector<uint8_t> send(std::string request, bool print=false, uint8_t frameSize = 8);
std::vector<uint8_t> send_wrapper(const std::vector<uint8_t>& request) ;
void read_with_timeout(boost::asio::io_context &io, boost::asio::serial_port &serial, std::vector<uint8_t> &response_vector, int timeout_ms) ;
std::vector<std::string> splitString(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

int main()
{
    LCDA630P_Modbus_RTU servo;
    char mode ;
    std::string s ; 
    while (true)
    {
        std::cout << "choose what what to do: r[read], w[write], p[acutal position], t[torque], m[move], s[speed], d[disable], q[quit]" << std::endl ; 
        std::cin >> mode ; 
        switch (mode)
        {
        case 'b':
        {
            std::vector<std::vector<uint8_t>> params = servo.read_servo_brief(1, send_wrapper);
            break;
        }
        case 'r':
        {
            std::cout << "*****************Read Param*****************" << std::endl;
            int paramGroup, paramOffset ; 
            std::cout << "Type Group Parameter and Offset in format GROUP,OFFSET, SIZE[OPTIONAL]" << std::endl ;
            std::cin >> s ;
            std::vector<std::string> params = splitString(s, ',');
            paramGroup = std::stoi(params[0]);
            paramOffset = std::stoi(params[1]);
            if (params.size() < 3)
                servo.read_parameter(1, paramGroup, paramOffset, send_wrapper);
            else 
            {
                uint8_t size = std::stoi(params[2]);
                servo.read_parameter(1, paramGroup, paramOffset, send_wrapper, size);
            }
            std::cout << "*****************Read Param*****************" << std::endl;  
            break; 
        }
        case 'w':
        {
            std::cout << "*****************Write Param*****************" << std::endl;
            int paramGroup, paramOffset ;
            int32_t value ;  
            std::cout << "Type Group Parameter, Offset and value in format GROUP,OFFSET,VALUE" << std::endl ;
            std::cin >> s ;
            std::vector<std::string> params = splitString(s, ',');
            if (params.size() < 3 )
            {
                std::cerr << "type at least 3 values, typed: " << params.size() << std::endl ;
                break;
            }
            paramGroup = std::stoi(params[0]);
            paramOffset = std::stoi(params[1]);
            value = std::stoi(params[2]);
            if (abs(value) > 65535)
                servo.write_parameter_32(1, paramGroup, paramOffset, value, send_wrapper);
            else
                servo.write_parameter(1, paramGroup, paramOffset, value, send_wrapper);
            std::cout << "*****************Write Param*****************" << std::endl;  
            break; 
        }    
        case 'p' :
        {
            uint64_t pos = servo.get_actual_position(1, send_wrapper);
            std::cout << "Actual Absolut position is : " << std::dec <<  pos << " hex : 0x" << std::hex << pos << std::endl ;
            break;
        }   
        case 'v' :
        {
            uint64_t pos = servo.get_actual_position(1, send_wrapper);
            std::cout << "Actual Absolut position is : " << std::dec <<  pos << " hex : 0x" << std::hex << pos << std::endl ;
            break;
        }           
        case 't' :
        {
            std::cout << "Type Torque[%]" << std::endl ;
            std::cin >> s ;      
            int32_t torque = std::stoi(s);  
            std::vector<std::vector<uint8_t>> config = servo.set_torque(1, torque, send_wrapper);
            break;
        }
        case 'm' :
        {
            std::cout << "Type position to move or q to quit" << std::endl ;
            std::cin >> s ;      
            int32_t position = std::stoi(s);  
            std::vector<std::vector<uint8_t>> config = servo.config_for_modbus_control_position(1, send_wrapper);
            std::vector<std::vector<uint8_t>> one_rot = servo.moveRelative(1, position, send_wrapper);  
            break;
        }                          
        case 's' :
        {
            std::cout << "Type speed value or q to quit" << std::endl ;
            std::cin >> s ;      
            int32_t position = std::stoi(s);  
            std::vector<std::vector<uint8_t>> config = servo.config_for_modbus_control_speed(1, send_wrapper);
            std::vector<std::vector<uint8_t>> one_rot = servo.moveVelocity(1, position, send_wrapper);  
            break;
        }         
        case 'd' :
        {
            bool response = servo.disable(1, send_wrapper);
            break;
        }
        case 'q' :        
           return 0 ;
        default:
            break;
        } 
    } 
    return 0;
}

std::vector<uint8_t> send(std::string request, bool print, uint8_t frameSize)
{
    std::vector<uint8_t> response_vector(frameSize) ; 
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

        if (print)
        {
            std::cout << "send: " ;

            for (char c : request)
            {
                std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(c) << " ";
            }
            std::cout << std::endl;
        }

        read_with_timeout(io, serial, response_vector, 100);
        
        if (print)
        {
            std::cout << "recive: " ;

            for (char c : response_vector)
            {
                std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(c) << " ";
            }
            std::cout << std::endl;  
        }      

        return response_vector ;
    }
    catch (std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return response_vector ;
    }
}
std::vector<uint8_t> send_wrapper(const std::vector<uint8_t> &request)
{
    std::string request_string = "";

    for (int i = 0; i < request.size() ; i++)
    {
        request_string += static_cast<char>(request[i]);        // Get low byte
    }
    uint8_t frameSize = 8 ; 
    if (  ((uint8_t)request[5]) < frameSize)
        frameSize = 8 ; 
    else
        frameSize = ((uint8_t)request[5]) ; 
    std::cout << "frame size : " << std::dec << +frameSize << std::endl ; //+ - promotion to a printable type
    std::vector<uint8_t> response = send(request_string, true, frameSize);
    return response ;
}
void read_with_timeout(boost::asio::io_context &io, boost::asio::serial_port &serial, std::vector<uint8_t> &response_vector, int timeout_ms)
{
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
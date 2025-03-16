#include <serial.hpp>

Serial::Serial(const std::string &port, int32_t baudrate) : serial_port(io_service), port(port), baudrate(baudrate){
    
}

Serial::~Serial(){
    close();
}

void Serial::open()
{
    try{
        serial_port.open(port);
        serial_port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
    }
    catch (const boost::system::system_error &e){
        std::cerr << "Error opening serial port: " << e.what() << std::endl;
    }
}

void Serial::close()
{
    if (serial_port.is_open())
    {
        serial_port.close();
    }
}

bool Serial::isOpen(){
    return serial_port.is_open();
}


/**
 * sends a string over serial to the connected port. 
 * 
 */
bool Serial::write(const std::string &message) {

    if(isOpen()){
        boost::asio::write(serial_port, boost::asio::buffer(message));
        return true;
    }

    return false;

}

void Serial::read(std::string &buffer){

    last_string = "";
    boost::asio::async_read_until(serial_port, boost::asio::dynamic_buffer(last_string), '\n', 
    boost::bind(&Serial::read_handler, this, boost::placeholders::_1, boost::placeholders::_2));
}

void Serial::read_handler(const boost::system::error_code &error, std::size_t bytes_transferred) {
    if (!error) {
        std::cout << "Received: " << last_string << std::endl;
    } else {
        std::cerr << "Error during read: " << error.message() << std::endl;
    }
}

// This will block and process async reads/writes
void Serial::run() {
    io_service.run();
}
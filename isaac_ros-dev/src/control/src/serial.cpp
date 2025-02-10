

#include <serial.hpp>



/**
 * sends a string over serial to the connected port. 
 * 
 */
void Serial::send(const std::string &message) {

    boost::asio::write(serial_, boost::asio::buffer(message));


}

char Serial::read_byte() {

    char c;
    boost::system::error_code ec;
    boost::asio::read(serial_, boost::asio::buffer(&c, 1), ec);

    if (ec) {
        return '\0'; 
    }

    return c;

}

void Serial::read_string(std::string &buffer){

    boost::asio::streambuf buf;
    boost::system::error_code ec;

    std::bind(readHandler, std::placeholders::_1, std::placeholders::_2, std::ref(serial_), std::ref(buffer));

    boost::asio::async_read_until(serial_, boost::asio::dynamic_buffer(buffer), '\n',std::move(Serial::readHandler)); 


}

void Serial::readHandler(
    const boost::system::error_code& error,
    std::size_t bytes_transferred,
    boost::asio::serial_port& serial,
    std::string& buffer
) {
    if (!error) {
        std::cout << "Received: " << buffer.substr(0, bytes_transferred) << std::endl;
        buffer.clear();
    } else {
        std::cerr << "Error: " << error.message() << std::endl;
    }
}





#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <boost/asio.hpp>
#include <cstdint>
#include <string>
#include <iostream>


class Serial {

    public: 
    
    Serial(const std::string &port, int32_t baudrate) : baudrate_(baudrate) {

        serial_.open(port);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
        
        io_.run();


    }

    void send(const std::string &message);

    char read_byte();

    void read_string(std::string &buffer);


    private:


    boost::asio::io_service io_;
    boost::asio::serial_port serial_{io_};

    int32_t baudrate_;
    
        
    void readHandler(
        const boost::system::error_code& error,
        std::size_t bytes_transferred,
        boost::asio::serial_port& serial,
        std::string& buffer
    );


};










#endif // SERIAL_HPP
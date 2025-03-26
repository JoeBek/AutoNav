#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <boost/asio.hpp>
#include <cstdint>
#include <string>
#include <iostream>


class Serial {

    public: 
    
    // constructor and desctructor
    Serial();
    ~Serial();

    // send/receive motor commands
    bool write(const std::string &message);
    void read(const std::string &buffer);
    void run();

    // check connection
    void close();
    bool isOpen();
    void open();

    //last read string
    std::string last_string;

    private:

    // connection details
    int32_t baudrate;
    std::string port;

    // idk what this is
    boost::asio::io_service io_service;
    boost::asio::serial_port serial_port;


    void read_handler(const boost::system::error_code &error, std::size_t bytes_transferred); 
};

#endif // SERIAL_HPP
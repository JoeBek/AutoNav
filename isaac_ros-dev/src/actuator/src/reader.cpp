#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <string>

class SerialReaderNode : public rclcpp::Node
{
public:
    SerialReaderNode() : Node("serial_reader_node"), io_(), serial_(io_, "/dev/THS0")
    {
        serial_.set_option(boost::asio::serial_port_base::baud_rate(9600));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&SerialReaderNode::read_serial, this));
    }

private:
    void read_serial()
    {
        boost::asio::streambuf buf;
        boost::asio::read_until(serial_, buf, "\n");
        std::string data = boost::asio::buffer_cast<const char*>(buf.data());
        RCLCPP_INFO(this->get_logger(), "Received: %s", data.c_str());
    }

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


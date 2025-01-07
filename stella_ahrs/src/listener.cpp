#include "mw_ahrs.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    char *port = "/dev/ttyUSB0";

    rclcpp::spin(std::make_shared<ntrex::MwAhrsRosDriver>(port,460800, 500));
    rclcpp::shutdown();
    return 0;
}
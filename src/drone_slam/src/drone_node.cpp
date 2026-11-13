#include "rclcpp/rclcpp.hpp"

class DroneNode : public rclcpp::Node
{
public:
    DroneNode() : Node("drone_node")
    {
        RCLCPP_INFO(this->get_logger(), "Drone SLAM node started!");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&DroneNode::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Drone system is running...");
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneNode>());
    rclcpp::shutdown();
    return 0;
}

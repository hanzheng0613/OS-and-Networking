#include <rclcpp/rclcpp.hpp>



#include <std_msgs/msg/string.hpp>


void __attribute__((noinline, visibility("default"))) handle_message(const std::shared_ptr<std_msgs::msg::String> msg,
                                                                      rclcpp::Node * node)
{
    RCLCPP_INFO(node->get_logger(),"Received message: '%s'", msg->data.c_str());
}

class ShmSubscriber : public rclcpp::Node
{

public:
    ShmSubscriber() : Node("shm_subscriber")
    {
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "chatter",
            10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                handle_message(msg, this);
            });
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShmSubscriber>();
    
    
    rclcpp::spin(node);
    
    
    rclcpp::shutdown();
    return 0;
}


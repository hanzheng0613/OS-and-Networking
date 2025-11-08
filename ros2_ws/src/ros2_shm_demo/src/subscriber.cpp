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
    [](std_msgs::msg::String::SharedPtr) {
        // Empty callback – we'll call take() manually anyway
    });
    }

    void poll_messages()
    {
        std::shared_ptr<std_msgs::msg::String> msg = std::make_shared<std_msgs::msg::String>();
        rclcpp::MessageInfo info;

        // Manually take message
        bool taken = sub_->take(*msg, info);

        if (taken) {
            handle_message(msg, this);
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShmSubscriber>();

    rclcpp::Rate rate(10);  // 10 Hz polling
    while (rclcpp::ok()) {
    
        node->poll_messages();  // call take() manually
        rclcpp::spin_some(node); 
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}


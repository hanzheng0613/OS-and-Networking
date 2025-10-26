#include <rclcpp/rclcpp.hpp>


#include <std_msgs/msg/string.hpp>

#include <chrono>


#include <memory>


using namespace std::chrono_literals;

class ShmPublisher : public rclcpp::Node
{
public:
  ShmPublisher() : Node("shm_publisher")
  {

    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(500ms, [this]() {
      publish_message();
    });
  }
  
  
  void __attribute__((noinline))publish_message() {
  
    auto msg = std_msgs::msg::String();
    msg.data = std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(),"Publishing: '%s'", msg.data.c_str());
    
    
    pub_->publish(msg);
}

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  
  
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShmPublisher>();
  
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



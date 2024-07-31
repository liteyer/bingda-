#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

// 定义一个MinimalSubscriber类，继承自rclcpp::Node
class MinimalSubscriber : public rclcpp::Node
{
  public:
    // MinimalSubscriber的构造函数
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      // 创建一个订阅者，订阅名为"topic"的主题，队列大小为10，并绑定topic_callback函数
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    // topic_callback函数，用于处理接收到的消息
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      // 使用RCLCPP_INFO宏和节点日志记录器输出接收到的消息内容
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    // 定义一个订阅者指针
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

// main函数，程序的入口点
int main(int argc, char * argv[])
{
  // 初始化ROS 2
  rclcpp::init(argc, argv);
  // 创建MinimalSubscriber类的共享指针，并使用rclcpp::spin函数保持节点运行
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  // 关闭ROS 2
  rclcpp::shutdown();
  // 返回0，表示程序成功执行
  return 0;
}
//
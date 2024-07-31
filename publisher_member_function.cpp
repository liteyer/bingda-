#include <chrono>  // 导入用于处理时间的头文件
#include <memory>  // 导入用于智能指针的头文件

#include "rclcpp/rclcpp.hpp"  // 导入ROS 2的C++客户端库
#include "std_msgs/msg/string.hpp"  // 导入ROS 2的标准消息类型String

using namespace std::chrono_literals;  // 使用chrono命名空间的时间单位

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

// 定义一个MinimalPublisher类，继承自rclcpp::Node
class MinimalPublisher : public rclcpp::Node
{
  public:
    // MinimalPublisher的构造函数
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)  // 创建一个名为"minimal_publisher"的节点，并初始化计数器
    {
      // 创建一个发布者，发布到名为"topic"的主题，队列大小为10
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      // 创建一个定时器，每隔500毫秒调用timer_callback函数
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    // timer_callback函数，用于定时发布消息
    void timer_callback()
    {
      // 创建一个std_msgs::msg::String类型的消息
      auto message = std_msgs::msg::String();
      // 将消息数据设置为"Hello, world! "加上当前计数器的值
      message.data = "Hello, world! " + std::to_string(count_++);
      // 使用RCLCPP_INFO宏和节点日志记录器输出即将发布的消息
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      // 调用发布者的publish函数来发布消息
      publisher_->publish(message);
    }
    // 定义一个发布者指针
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // 定义一个计数器
    size_t count_;
};

// main函数，程序的入口点
int main(int argc, char * argv[])
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    // 创建MinimalPublisher类的共享指针，并使用rclcpp::spin函数保持节点运行
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    // 关闭ROS 2
    rclcpp::shutdown();
    // 返回0，表示程序成功执行
    return 0;
}

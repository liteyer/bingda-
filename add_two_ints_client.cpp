#include "rclcpp/rclcpp.hpp" // 包含ROS 2 C++客户端库的头文件
#include "example_interfaces/srv/add_two_ints.hpp" // 包含示例服务定义的头文件

#include <chrono> // 用于处理时间相关的功能
#include <cstdlib> // 用于标准库的C风格函数，比如字符串转长整型
#include <memory> // 用于智能指针

using namespace std::chrono_literals; // 允许使用字面量后缀，如1s代表1秒

int main(int argc, char **argv) // 程序入口点
{
  rclcpp::init(argc, argv); // 初始化ROS 2

  // 检查用户是否输入了两个参数
  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1; // 如果参数数量不正确，则打印使用方法并退出
  }

  // 创建一个名为"add_two_ints_client"的ROS 2节点
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  
  // 创建一个客户端，用于调用名为"add_two_ints"的服务
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  // 创建一个服务请求，并从命令行参数填充两个整数
  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]); // 将第一个参数转换为长整型并赋值给请求的a
  request->b = atoll(argv[2]); // 将第二个参数转换为长整型并赋值给请求的b

  // 循环等待直到服务可用
  while (!client->wait_for_service(1s)) {
    // 如果ROS 2不再运行，则打印错误信息并退出
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    // 如果服务不可用，则打印信息并继续等待
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // 异步发送请求，并获取一个未来对象来跟踪结果
  auto result = client->async_send_request(request);
  
  // 等待直到请求完成
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS) // 检查请求是否成功
  {
    // 如果成功，打印出两个整数的和
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    // 如果失败，打印错误信息
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  // 清理并关闭ROS 2
  rclcpp::shutdown();
  return 0; // 程序正常退出
}

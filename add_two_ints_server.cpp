#include "rclcpp/rclcpp.hpp" // 包含ROS 2 C++客户端库的头文件
#include "example_interfaces/srv/add_two_ints.hpp" // 包含示例服务定义的头文件

#include <memory> // 用于智能指针

// 定义服务处理函数，它将被调用来处理客户端的请求
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  // 计算请求中两个整数的和，并将结果存储在响应中
  response->sum = request->a + request->b;
  
  // 打印接收到的请求信息
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  
  // 打印将要发送回客户端的响应信息
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv) // 程序入口点
{
  rclcpp::init(argc, argv); // 初始化ROS 2

  // 创建一个名为"add_two_ints_server"的ROS 2节点
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  // 在该节点上创建一个名为"add_two_ints"的服务，并将处理函数指针传递给服务
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  // 打印信息，表示服务器已准备好接收请求
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  // 进入循环，等待并处理客户端请求
  rclcpp::spin(node);
  
  // 清理并关闭ROS 2
  rclcpp::shutdown();
}

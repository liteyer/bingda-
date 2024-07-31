import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# 定义一个名为MinimalPublisher的类，继承自Node
class MinimalPublisher(Node):

    def __init__(self):
        # 调用父类Node的构造函数，并设置节点名称为'minimal_publisher'
        super().__init__('minimal_publisher')
        
        # 创建一个发布者，发布的话题类型为String，话题名称为'topic'，队列大小为10
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # 设置定时器周期为0.5秒
        timer_period = 0.5  # seconds
        # 创建一个定时器，每隔timer_period秒调用一次self.timer_callback函数
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 初始化一个计数器
        self.i = 0

    # 定义定时器回调函数
    def timer_callback(self):
        # 创建一个String类型的消息
        msg = String()
        # 设置消息内容，包含计数器self.i的值
        msg.data = 'Hello World: %d' % self.i
        # 使用发布者发布消息
        self.publisher_.publish(msg)
        # 在日志中打印发布的信息
        self.get_logger().info('Publishing: "%s"' % msg.data)
        # 增加计数器
        self.i += 1


# 主函数
def main(args=None):
    # 初始化rclpy，这是ROS2客户端库的Python接口
    rclpy.init(args=args)

    # 创建MinimalPublisher类的实例
    minimal_publisher = MinimalPublisher()

    # 进入事件循环，等待回调函数被调用
    rclpy.spin(minimal_publisher)

    # 显式销毁节点，这是一个可选步骤，因为当节点对象被垃圾回收时，节点会自动销毁
    minimal_publisher.destroy_node()
    # 关闭rclpy，清理资源
    rclpy.shutdown()


# 当脚本作为主程序运行时，调用main函数
if __name__ == '__main__':
    main()

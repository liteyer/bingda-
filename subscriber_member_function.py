import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# 定义一个名为MinimalSubscriber的类，继承自Node
class MinimalSubscriber(Node):

    def __init__(self):
        # 调用父类Node的构造函数，并设置节点名称为'minimal_subscriber'
        super().__init__('minimal_subscriber')
        
        # 创建一个订阅者，订阅的话题类型为String，话题名称为'topic'，队列大小为10
        # 并设置回调函数为self.listener_callback
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        
        # 防止订阅者变量未使用而引发警告
        self.subscription

    # 定义订阅者回调函数
    def listener_callback(self, msg):
        # 在日志中打印接收到的消息内容
        self.get_logger().info('I heard: "%s"' % msg.data)


# 主函数
def main(args=None):
    # 初始化rclpy，这是ROS2客户端库的Python接口
    rclpy.init(args=args)

    # 创建MinimalSubscriber类的实例
    minimal_subscriber = MinimalSubscriber()

    # 进入事件循环，等待回调函数被调用
    rclpy.spin(minimal_subscriber)

    # 显式销毁节点，这是一个可选步骤，因为当节点对象被垃圾回收时，节点会自动销毁
    minimal_subscriber.destroy_node()
    
    # 关闭rclpy，清理资源
    rclpy.shutdown()


# 当脚本作为主程序运行时，调用main函数
if __name__ == '__main__':
    main()

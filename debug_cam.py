import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import sys

class CameraDebugger(Node):
    def __init__(self):
        super().__init__('camera_debugger')
        self.bridge = CvBridge()
        
        # 定义两种 QoS 配置：可靠 vs 尽力而为
        self.qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.qos_best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 定义我们要尝试的所有话题 (根据你提供的 list)
        target_topics = [
            '/camera/color/image_raw',
            '/image_raw',
            '/camera/depth/image_raw'
        ]

        print("="*40)
        print("正在暴力扫描所有图像话题...")
        print("只要有一个能通，就说明环境没问题！")
        print("="*40)

        self.subs = []
        for topic in target_topics:
            # 对每个话题，同时尝试 Reliable 和 Best Effort 两种模式
            # 这是一个“笨办法”，但最管用
            self.create_sniffer(topic, self.qos_reliable, "RELIABLE")
            self.create_sniffer(topic, self.qos_best_effort, "BEST_EFFORT")

    def create_sniffer(self, topic_name, qos, qos_name):
        # 创建订阅者
        sub = self.create_subscription(
            Image,
            topic_name,
            lambda msg, t=topic_name, q=qos_name: self.callback(msg, t, q),
            qos
        )
        self.subs.append(sub)

    def callback(self, msg, topic, qos_type):
        print(f"\n[成功!] 收到图像了！")
        print(f"  -> 话题: {topic}")
        print(f"  -> QoS模式: {qos_type}")
        print(f"  -> 尺寸: {msg.width} x {msg.height}")
        
        # 尝试显示图像
        try:
            if "depth" in topic:
                img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                # 深度图归一化以便显示
                cv2.imshow("Debug Depth", cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX))
            else:
                img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imshow("Debug Color", img)
            
            cv2.waitKey(1)
        except Exception as e:
            print(f"  -> 显示失败: {e}")

def main():
    rclpy.init()
    debugger = CameraDebugger()
    try:
        print("等待数据中 (如果卡在这里超过10秒，请检查 Gazebo 是否暂停)...")
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        pass
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
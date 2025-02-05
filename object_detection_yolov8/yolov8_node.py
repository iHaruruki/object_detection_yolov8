#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLOv8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')
        # 画像トピックのサブスクライバー
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # 画像がパブリッシュされているトピック名に合わせる
            self.image_callback,
            10)
        self.subscription  # 参照を保持

        self.bridge = CvBridge()

        # YOLOv8モデルの読み込み（例：軽量モデル yolov8n.pt を使用）
        self.get_logger().info("YOLOv8モデルを読み込み中...")
        self.model = YOLO("yolov8n.pt")
        self.get_logger().info("YOLOv8モデルの読み込みが完了しました。")

    def image_callback(self, msg):
        # ROS2のImageメッセージをOpenCV画像に変換
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"画像変換エラー: {e}")
            return

        # YOLOv8による物体検出
        try:
            # YOLOv8は画像を直接渡すと内部で推論を行います
            results = self.model(cv_image)
        except Exception as e:
            self.get_logger().error(f"YOLOv8推論エラー: {e}")
            return

        # 推論結果の1枚目の画像に描画（resultsはリストで返される）
        # 結果描画には results[0].plot() を利用可能
        annotated_frame = results[0].plot()

        # ウィンドウに表示（必要に応じて別トピックにパブリッシュも可能）
        cv2.imshow("YOLOv8 Detection", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

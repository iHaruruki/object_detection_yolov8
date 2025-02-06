#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        # 画像トピックのサブスクライバー（例：/image_raw）
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # 実際の環境に合わせてトピック名を変更
            self.image_callback,
            10)
        self.subscription  # 参照保持

        self.bridge = CvBridge()

        # YOLOv8モデルの読み込み
        self.get_logger().info("YOLOv8モデルを読み込み中...")
        try:
            # モデルファイル（例：yolov8n.pt）のパスを指定
            self.model = YOLO("yolov8n.pt")
            self.get_logger().info("YOLOv8モデルの読み込みが完了しました。")
        except Exception as e:
            self.get_logger().error(f"モデル読み込みエラー: {e}")

    def image_callback(self, msg):
        # ROS2のImageメッセージをOpenCV画像に変換
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"画像変換エラー: {e}")
            return

        # YOLOv8による物体検出（推論）
        try:
            results = self.model(cv_image)
        except Exception as e:
            self.get_logger().error(f"YOLOv8推論エラー: {e}")
            return

        # 推論結果の1枚目の画像に検出結果を描画（ultralyticsのresultsはリスト）
        annotated_image = results[0].plot()

        # 結果画像をウィンドウ表示（必要に応じてROSトピックにパブリッシュするなどの拡張も可能）
        cv2.imshow("Object Detection (YOLOv8)", annotated_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

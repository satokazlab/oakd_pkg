import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2
from ultralytics import YOLO
from visualization_msgs.msg import Marker  # Marker メッセージをインポート
from geometry_msgs.msg import Point

class YoloOakdNode(Node):
    def __init__(self):
        super().__init__('yolo_oakd_node')
        
        # YOLOv8のモデルをロード
        self.model = YOLO('/home/yoheiyanagi/YOLO/ultralytics/runs/detect/train2/weights/best.pt')

        # カメラの設定
        self.pipeline = dai.Pipeline()

        # RGBカメラの設定
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setFps(30)

        # モノカメラの設定
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

        mono_right = self.pipeline.create(dai.node.MonoCamera)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

        # 深度計算
        stereo = self.pipeline.create(dai.node.StereoDepth)
        stereo.setConfidenceThreshold(200)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # 出力設定
        self.xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        self.xout_rgb.setStreamName("video")
        cam_rgb.video.link(self.xout_rgb.input)

        self.xout_depth = self.pipeline.create(dai.node.XLinkOut)
        self.xout_depth.setStreamName("depth")
        stereo.depth.link(self.xout_depth.input)

        # ROS2のパブリッシャ
        self.publisher_rgb = self.create_publisher(Image, 'camera/image_raw', 10)
        self.publisher_depth = self.create_publisher(Image, 'camera/depth_raw', 10)
        self.publisher_marker = self.create_publisher(Marker, 'camera/bounding_boxes', 10)  # Marker パブリッシャを作成

        # DepthAIデバイスの初期化
        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)
        self.q_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # RGBと深度のフレームを取得
        frame_rgb = self.q_rgb.get().getCvFrame()
        frame_depth = self.q_depth.get().getCvFrame()

        # YOLOv8で物体検出
        results = self.model(frame_rgb)

        if isinstance(results,list):
             results=results[0]

        # Markerを作成してバウンディングボックスをRVizに表示
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "yolo_bounding_boxes"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        if hasattr(results,'boxes'):
            
            
         for box in results.boxes:
             x1, y1, x2, y2 = map(int, box.xyxy[0])  # バウンディングボックス座標
             conf = float(box.conf[0])  # 信頼度を取得

            # RGB画像にバウンディングボックスと信頼度を描画
             cv2.rectangle(frame_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
             cv2.putText(frame_rgb, f"{conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0),5 )

            # RViz用のMarkerを作成
             marker.points.append(self.create_point(x1, y1))
             marker.points.append(self.create_point(x2, y1))
             marker.points.append(self.create_point(x2, y1))
             marker.points.append(self.create_point(x2, y2))
             marker.points.append(self.create_point(x2, y2))
             marker.points.append(self.create_point(x1, y2))
             marker.points.append(self.create_point(x1, y2))
             marker.points.append(self.create_point(x1, y1))

        # バウンディングボックスをパブリッシュ
        self.publisher_marker.publish(marker)

        # RGB画像と深度画像をROS2メッセージに変換してパブリッシュ
        msg_rgb = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="bgr8")
        msg_depth = self.bridge.cv2_to_imgmsg(frame_depth, encoding="mono16")

        self.publisher_rgb.publish(msg_rgb)
        self.publisher_depth.publish(msg_depth)

    def create_point(self, x, y):
        """RVizのMarker用に座標を設定"""
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = 0.0
        return point

def main(args=None):
    rclpy.init(args=args)
    node = YoloOakdNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

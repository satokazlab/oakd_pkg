#!/usr/bin/env python3

import sys
import cv2
import depthai as dai
import numpy as np
import time
import json
from pathlib import Path
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        # ROS 2パラメータの宣言
        self.declare_parameter('model_path', '')
        self.declare_parameter('config_path', '')

        # camera topic add 
        self.bridge=CvBridge()
        self.image_publisher=self.create_publisher(Image,'detection_image',10)

        

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        if not Path(model_path).exists():
            self.get_logger().error(f"Model path does not exist: {model_path}")
            sys.exit(1)
        if not Path(config_path).exists():
            self.get_logger().error(f"Config path does not exist: {config_path}")
            sys.exit(1)

        # JSON設定の読み込み
        with open(config_path, 'r') as f:
            config = json.load(f)

        nnConfig = config.get("nn_config", {})
        W, H = map(int, nnConfig.get("input_size", "640*480").split('x'))
        metadata = nnConfig.get("NN_specific_metadata", {})
        self.labels = config.get("mappings", {}).get("labels", {})

        anchors = metadata.get("anchors", [])
        anchorMasks = metadata.get("anchor_masks", {})
        iouThreshold = metadata.get("iou_threshold", 0.5)
        confidenceThreshold = metadata.get("confidence_threshold", 0.5)

        # パイプラインの構築
        pipeline = dai.Pipeline()
        camRgb = pipeline.create(dai.node.ColorCamera)
        detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
        nnOut = pipeline.create(dai.node.XLinkOut)
        nnOut.setStreamName("nn")
        camRgb.setPreviewSize(W, H)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        detectionNetwork.setConfidenceThreshold(confidenceThreshold)
        detectionNetwork.setNumClasses(len(self.labels))
        detectionNetwork.setCoordinateSize(4)
        detectionNetwork.setAnchors(anchors)
        detectionNetwork.setAnchorMasks(anchorMasks)
        detectionNetwork.setIouThreshold(iouThreshold)
        detectionNetwork.setBlobPath(model_path)

        camRgb.preview.link(detectionNetwork.input)
        detectionNetwork.out.link(nnOut.input)

        # カメラ映像の出力を設定
        camRgbVideoOut = pipeline.create(dai.node.XLinkOut)
        camRgbVideoOut.setStreamName("rgb")  # カメラ映像のストリーム名を設定
        camRgb.video.link(camRgbVideoOut.input)


        

        # デバイス接続とパイプライン開始

        self.device = dai.Device(pipeline)
        self.qDet = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)
        # camera topic add
        self.qVideo = self.device.getOutputQueue(name='rgb',maxSize=4,blocking=False)
        #camRgb.video.link(self.qVideo.input)

        self.detection_publisher = self.create_publisher(Detection2DArray, 'detections', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        inDet = self.qDet.tryGet()
        if inDet is None:
            return

        detection_msg = Detection2DArray()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_frame"
        detection_msg.header = header
        # camara topic add 
        frame=None
        inFrame=self.qVideo.tryGet()

        if inFrame is not None:
            frame=inFrame.getCvFrame()
        
        if frame is not None:
            
            for detection in inDet.detections:
                
                detection2d = Detection2D()
                label_id = detection.label
                label_name = self.labels[label_id] if label_id < len(self.labels) else str(label_id)

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = label_name
                hypothesis.hypothesis.score = detection.confidence
                detection2d.results.append(hypothesis)

                bbox = BoundingBox2D()
                bbox.center.position.x = (detection.xmin + detection.xmax) / 2
                bbox.center.position.y = (detection.ymin + detection.ymax) / 2
                bbox.size_x = detection.xmax - detection.xmin
                bbox.size_y = detection.ymax - detection.ymin
                detection2d.bbox = bbox

                detection_msg.detections.append(detection2d)

                xmin=int(detection.xmin * frame.shape[1])
                ymin=int(detection.ymin * frame.shape[0])
                xmax=int(detection.xmax * frame.shape[1])
                ymax=int(detection.ymax * frame.shape[0])

                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(frame, label_name, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4)


                self.detection_publisher.publish(detection_msg)

            image_msg=self.bridge.cv2_to_imgmsg(frame,encoding="bgr8")
            self.image_publisher.publish(image_msg)

        self.detection_publisher.publish(detection_msg)    

    def destroy_node(self):
        super().destroy_node()
        self.device.close()

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

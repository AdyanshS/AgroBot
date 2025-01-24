#!/usr/bin/env python3


from rclpy.node import Node
import rclpy
import cv_bridge
import cv2
import numpy as np

from ultralytics import YOLO

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from agrobot_interfaces.msg import Xywh, XyXy, YoloResults


class ObjectDetectionYOLO(Node):
    def __init__(self):
        super().__init__('object_detection_YOLO')

        self.declare_parameter('use_compressed_image', True)
        self.declare_parameter('show_image', True)
        self.declare_parameter('model_name', 'cotton_plant_model.pt')
        self.declare_parameter('iou', 0.3)
        self.declare_parameter('conf', 0.3)

        self.use_compressed_image = self.get_parameter(
            'use_compressed_image').value
        self.show_image = self.get_parameter(
            'show_image').value
        self.model_name = self.get_parameter('model_name').value

        if self.use_compressed_image:
            self.compressed_subscription = self.create_subscription(
                CompressedImage,
                '/gemini_e/color/image_raw/compressed',
                self.image_callback,
                10)

        if not self.use_compressed_image:
            self.subscription = self.create_subscription(
                Image,
                'image_raw',
                self.image_callback,
                10
            )

        # Publish the annotated image
        self.publisher = self.create_publisher(
            CompressedImage,
            'annotated_compressed_image',
            10
        )
        # Publish the YOLO results
        self.yolo_result_publisher = self.create_publisher(
            YoloResults,
            'yolo_results',
            10
        )

        self.model = YOLO(
            f'/home/sr09/agrobot_ws/src/agrobot_vision/agrobot_vision/model/{self.model_name}')

        # Set the IOU and Confidence threshold
        self.iou = self.get_parameter('iou').value
        self.conf = self.get_parameter('conf').value

    def image_callback(self, msg):
        self.get_logger().info('Receiving image', throttle_duration_sec=3.0)

        # Convert ROS Image message to OpenCV image
        bridge = cv_bridge.CvBridge()
        if self.use_compressed_image:
            # Convert compressed image to OpenCV image
            img = bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        else:
            # Convert image to OpenCV image
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.yolo_object_detect(img)

    def yolo_object_detect(self, img: np.ndarray):

        # Perform object detection and stores the results
        results = self.model.track(
            img, persist=True, conf=self.conf, iou=self.iou)

        yolo_result_msg = YoloResults()  # YoloResults message to store the results
        contour_areas = []  # List to store contour areas
        differences = []  # List to store differences
        z_differences = []  # List to store z_differences
        center_y_to_track_points = []  # List to store center_y_to_track_points

        # Calculate the  line's x-coordinate
        line_x = img.shape[1] // 2

        # Camera Center
        camera_center = (img.shape[1] // 2, img.shape[0] // 2)

        # Loop through the results and store the bounding box and stores the results in the YoloResults message
        for result in results:

            xywh_list = result.boxes.xywh.tolist()  # center_x, center_y, width, height

            for xywh in xywh_list:
                xywh_msg = Xywh()
                xywh_msg.center_x = xywh[0]
                xywh_msg.center_y = xywh[1]
                xywh_msg.width = xywh[2]
                xywh_msg.height = xywh[3]

                # Calculate the center_y_to_track and store it in the list
                # Top Line middle point
                # center_y_to_track = xywh[1] - (xywh[3] // 2)
                center_y_to_track = xywh[1] - 0

                center_y_to_track_points.append(center_y_to_track)

                yolo_result_msg.xywh.append(xywh_msg)

                # Calculate contour area
                contour_area = xywh[2] ** 2
                contour_areas.append(contour_area)

                # Calculate  x - difference
                difference = (xywh[0] - line_x)
                differences.append(difference)

                # Calculate z - difference (height difference)
                z_difference = (camera_center[1] - center_y_to_track)
                z_differences.append(z_difference)

            # top_left_x, top_left_y, bottom_right_x, bottom_right_y
            xyxy_list = result.boxes.xyxy.tolist()

            for xyxy in xyxy_list:
                xyxy_msg = XyXy()
                xyxy_msg.tl_x = xyxy[0]
                xyxy_msg.tl_y = xyxy[1]
                xyxy_msg.br_x = xyxy[2]
                xyxy_msg.br_y = xyxy[3]

                yolo_result_msg.xyxy.append(xyxy_msg)

        # Class ids:
        cls_list = [int(cls) for cls in result.boxes.cls.tolist()]
        yolo_result_msg.class_ids.extend(cls_list)

        # Confidence values
        conf_list = result.boxes.conf.tolist()
        yolo_result_msg.confidence.extend(conf_list)

        # Tracking ids
        if result.boxes.id is not None:
            ids_list = result.boxes.id.tolist()
            yolo_result_msg.tracking_id.extend(ids_list)

        # Plot the annotated image
        annotated_frame = results[0].plot()

        # Draw the center points, contour areas, and differences
        for i, xywh in enumerate(yolo_result_msg.xywh):
            center_x, center_y, width, height = xywh.center_x, xywh.center_y, xywh.width, xywh.height
            contour_area = contour_areas[i]
            difference = differences[i]
            z_difference = z_differences[i]
            center_y_to_track = center_y_to_track_points[i]

            # Draw the center point
            cv2.circle(annotated_frame, (int(center_x),
                       int(center_y)), 5, (0, 0, 255), -1)

            # Draw center_x_to_track point
            cv2.circle(annotated_frame, (int(center_x), int(
                center_y_to_track)), 5, (255, 255, 0), -1)

            # Put the contour area text
            cv2.putText(annotated_frame, f'Area: {int(contour_area)}',
                        (int(center_x), int(center_y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2)

            # Draw the difference line
            cv2.line(annotated_frame, (int(center_x), int(center_y_to_track)),
                     (line_x, int(center_y_to_track)), (255, 0, 0), 2)

            # Put the difference text
            cv2.putText(annotated_frame, f'x_Diff: {int(difference)}',
                        (int(center_x), int(center_y) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        2)

            # Put the z-difference text
            cv2.putText(annotated_frame, f'z_Diff: {int(z_difference)}',
                        (int(center_x), int(center_y) + 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        2)

        # Draw camera center point
        cv2.circle(annotated_frame, camera_center, 5, (0, 255, 255), -1)
        cv2.line(annotated_frame, (line_x, 0),
                 (line_x, img.shape[0]), (0, 255, 0), 2)

        # Add contour areas and differences to the YoloResults message
        yolo_result_msg.contour_area.extend(contour_areas)
        yolo_result_msg.x_differences.extend(differences)
        yolo_result_msg.center_y_to_track.extend(center_y_to_track_points)
        yolo_result_msg.z_differences.extend(z_differences)

        cv2.imshow('YOLOv11 Tracking', annotated_frame)
        cv2.waitKey(1)

        # Convert the annotated image to a ROS message
        bridge = cv_bridge.CvBridge()
        annotated_frame_msg = bridge.cv2_to_compressed_imgmsg(
            annotated_frame, dst_format='png')

        # Publish the results
        self.yolo_result_publisher.publish(yolo_result_msg)
        self.publisher.publish(annotated_frame_msg)


def main(args=None):
    rclpy.init(args=args)
    object_detection = ObjectDetectionYOLO()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

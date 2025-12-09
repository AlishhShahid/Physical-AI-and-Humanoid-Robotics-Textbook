import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import threading
import time


class IsaacObjectDetectionNode(Node):
    """
    Isaac ROS Object Detection node for humanoid robot
    Performs real-time object detection using computer vision techniques
    """
    def __init__(self):
        super().__init__('isaac_object_detection')

        # Parameters
        self.declare_parameter('detection_model', 'yolo')  # yolo, ssd, haar, color
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('max_detections', 10)
        self.declare_parameter('detection_frequency', 5.0)  # Hz
        self.declare_parameter('use_depth', True)  # Use depth for 3D detection

        self.detection_model = self.get_parameter('detection_model').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.max_detections = self.get_parameter('max_detections').value
        self.detection_frequency = self.get_parameter('detection_frequency').value
        self.use_depth = self.get_parameter('use_depth').value

        # Initialize detection models
        self.initialize_detection_model()

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Detection results
        self.detections = Detection2DArray()
        self.detection_lock = threading.Lock()

        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.image_sub = Subscriber(
            self, Image, '/humanoid_robot/camera/image_raw', qos_profile=qos_profile)
        self.depth_sub = Subscriber(
            self, Image, '/humanoid_robot/depth/image_raw', qos_profile=qos_profile)
        self.info_sub = self.create_subscription(
            CameraInfo, '/humanoid_robot/camera/camera_info', self.camera_info_callback, 10)

        # Synchronize image and depth for 3D detection
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_depth_callback)

        # Create publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/humanoid_robot/detections', 10)
        self.debug_image_pub = self.create_publisher(Image, '/humanoid_robot/detection_debug', 10)

        # Create timer for publishing detections
        self.detection_timer = self.create_timer(1.0 / self.detection_frequency, self.publish_detections)

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

        self.get_logger().info(f'Isaac Object Detection node initialized with {self.detection_model} model')

    def initialize_detection_model(self):
        """Initialize the object detection model based on parameter"""
        if self.detection_model == 'haar':
            # Initialize Haar cascade classifier
            self.haar_cascade = cv2.CascadeClassifier()
            # For demonstration, we'll use a pre-trained face cascade
            # In a real implementation, you'd train your own cascades for robot-relevant objects
            self.haar_cascade.load(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        elif self.detection_model == 'color':
            # Initialize color-based detection parameters
            # Define color ranges for objects of interest
            self.color_ranges = {
                'red': (np.array([0, 50, 50]), np.array([10, 255, 255])),
                'blue': (np.array([100, 50, 50]), np.array([130, 255, 255])),
                'green': (np.array([40, 50, 50]), np.array([80, 255, 255])),
                'yellow': (np.array([20, 50, 50]), np.array([40, 255, 255]))
            }
        else:
            # Default to simple shape-based detection
            self.get_logger().info('Using simple shape-based detection')

    def camera_info_callback(self, msg):
        """Receive camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def image_depth_callback(self, image_msg, depth_msg):
        """Process synchronized image and depth for object detection"""
        try:
            # Convert ROS images to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, "32FC1")

            # Perform object detection
            detections = self.detect_objects(cv_image, cv_depth)

            # Update detections with thread safety
            with self.detection_lock:
                self.detections = detections

            # Create debug image with detections
            debug_image = self.draw_detections(cv_image, detections)
            debug_image_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_image_msg.header = image_msg.header
            self.debug_image_pub.publish(debug_image_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image/depth: {e}')

    def detect_objects(self, image, depth_image):
        """Perform object detection on the image"""
        detections = Detection2DArray()
        detections.header.stamp = self.get_clock().now().to_msg()
        detections.header.frame_id = 'camera_link'  # Assuming camera frame

        # Convert image to appropriate format for detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply different detection methods based on model
        if self.detection_model == 'haar':
            # Haar cascade detection
            objects = self.haar_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30),
                flags=cv2.CASCADE_SCALE_IMAGE
            )

            for (x, y, w, h) in objects:
                detection = self.create_detection(x, y, w, h, "object", 0.8, depth_image)
                if detection is not None:
                    detections.detections.append(detection)

        elif self.detection_model == 'color':
            # Color-based detection
            for color_name, (lower, upper) in self.color_ranges.items():
                # Create mask for color range
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower, upper)

                # Find contours in the mask
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for contour in contours:
                    # Filter by area
                    area = cv2.contourArea(contour)
                    if area > 500:  # Minimum area threshold
                        # Get bounding rectangle
                        x, y, w, h = cv2.boundingRect(contour)
                        detection = self.create_detection(x, y, w, h, color_name, 0.7, depth_image)
                        if detection is not None:
                            detections.detections.append(detection)

        else:
            # Default: simple shape detection
            # Find contours in the grayscale image
            _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                # Filter by area
                area = cv2.contourArea(contour)
                if area > 1000:  # Minimum area threshold
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(contour)

                    # Determine object type based on aspect ratio
                    aspect_ratio = float(w) / h
                    if 0.7 <= aspect_ratio <= 1.3:
                        obj_type = "square"
                    elif aspect_ratio > 1.3:
                        obj_type = "rectangle_horizontal"
                    else:
                        obj_type = "rectangle_vertical"

                    detection = self.create_detection(x, y, w, h, obj_type, 0.6, depth_image)
                    if detection is not None:
                        detections.detections.append(detection)

        return detections

    def create_detection(self, x, y, w, h, label, confidence, depth_image):
        """Create a Detection2D message from bounding box coordinates"""
        detection = Detection2D()

        # Set bounding box
        detection.bbox.center.x = x + w / 2.0
        detection.bbox.center.y = y + h / 2.0
        detection.bbox.size_x = w
        detection.bbox.size_y = h

        # Set ID and confidence
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = label
        hypothesis.score = confidence
        detection.results.append(hypothesis)

        # If depth is available, add 3D position
        if self.use_depth and depth_image is not None:
            # Get depth at the center of the bounding box
            center_x = int(detection.bbox.center.x)
            center_y = int(detection.bbox.center.y)

            if 0 <= center_x < depth_image.shape[1] and 0 <= center_y < depth_image.shape[0]:
                depth = depth_image[center_y, center_x]

                # Only add 3D info if depth is valid
                if not np.isnan(depth) and depth > 0:
                    # Convert pixel coordinates to 3D world coordinates
                    if self.camera_matrix is not None:
                        # Use camera parameters to convert to 3D
                        fx = self.camera_matrix[0, 0]
                        fy = self.camera_matrix[1, 1]
                        cx = self.camera_matrix[0, 2]
                        cy = self.camera_matrix[1, 2]

                        # Calculate 3D position
                        pos_x = (center_x - cx) * depth / fx
                        pos_y = (center_y - cy) * depth / fy
                        pos_z = depth

                        # Set pose (simplified)
                        detection.id = f"{label}_{pos_x:.2f}_{pos_y:.2f}_{pos_z:.2f}"

        return detection

    def draw_detections(self, image, detections):
        """Draw detection results on the image"""
        output_image = image.copy()

        for detection in detections.detections:
            # Get bounding box coordinates
            center_x = int(detection.bbox.center.x)
            center_y = int(detection.bbox.center.y)
            size_x = int(detection.bbox.size_x)
            size_y = int(detection.bbox.size_y)

            # Calculate top-left corner
            x = center_x - size_x // 2
            y = center_y - size_y // 2

            # Draw bounding box
            cv2.rectangle(output_image, (x, y), (x + size_x, y + size_y), (0, 255, 0), 2)

            # Draw label and confidence
            label = detection.results[0].id if len(detection.results) > 0 else "unknown"
            confidence = detection.results[0].score if len(detection.results) > 0 else 0.0

            label_text = f"{label}: {confidence:.2f}"
            cv2.putText(output_image, label_text, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return output_image

    def publish_detections(self):
        """Publish the latest detections"""
        with self.detection_lock:
            if len(self.detections.detections) > 0:
                self.detections.header.stamp = self.get_clock().now().to_msg()
                self.detection_pub.publish(self.detections)


class IsaacSegmentationNode(Node):
    """
    Isaac ROS Segmentation node for humanoid robot
    Performs semantic and instance segmentation
    """
    def __init__(self):
        super().__init__('isaac_segmentation')

        # Parameters
        self.declare_parameter('segmentation_model', 'simple')  # simple, color-based
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('publish_frequency', 2.0)  # Hz

        self.segmentation_model = self.get_parameter('segmentation_model').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.publish_frequency = self.get_parameter('publish_frequency').value

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image, '/humanoid_robot/camera/image_raw', self.image_callback, 10)

        # Create publishers
        self.segmentation_pub = self.create_publisher(Image, '/humanoid_robot/segmentation', 10)

        # Create timer for processing
        self.process_timer = self.create_timer(1.0 / self.publish_frequency, self.process_segmentation)

        self.latest_image = None
        self.latest_segmentation = None

        self.get_logger().info(f'Isaac Segmentation node initialized with {self.segmentation_model} model')

    def image_callback(self, msg):
        """Receive image for segmentation"""
        try:
            # Store the latest image
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def process_segmentation(self):
        """Process segmentation on the latest image"""
        if self.latest_image is None:
            return

        # Perform segmentation based on model
        if self.segmentation_model == 'color':
            segmented_image = self.color_based_segmentation(self.latest_image)
        else:
            # Default to simple edge-based segmentation
            segmented_image = self.edge_based_segmentation(self.latest_image)

        # Publish segmentation result
        try:
            seg_msg = self.cv_bridge.cv2_to_imgmsg(segmented_image, encoding="bgr8")
            seg_msg.header.stamp = self.get_clock().now().to_msg()
            seg_msg.header.frame_id = 'camera_link'
            self.segmentation_pub.publish(seg_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing segmentation: {e}')

    def color_based_segmentation(self, image):
        """Perform color-based segmentation"""
        # Convert to HSV for better color separation
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges for different object types
        color_ranges = [
            (np.array([0, 50, 50]), np.array([10, 255, 255])),    # Red
            (np.array([100, 50, 50]), np.array([130, 255, 255])), # Blue
            (np.array([40, 50, 50]), np.array([80, 255, 255])),   # Green
            (np.array([20, 50, 50]), np.array([40, 255, 255])),   # Yellow
        ]

        # Create segmented image
        segmented = np.zeros_like(image)

        for i, (lower, upper) in enumerate(color_ranges):
            # Create mask for color range
            mask = cv2.inRange(hsv, lower, upper)

            # Apply color to segmented image
            color = np.random.randint(0, 255, 3)  # Random color for this segment
            segmented[mask != 0] = color

        return segmented

    def edge_based_segmentation(self, image):
        """Perform simple edge-based segmentation"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Create segmented image
        segmented = np.zeros_like(image)

        # Draw filled contours with random colors
        for i, contour in enumerate(contours):
            if cv2.contourArea(contour) > 500:  # Filter small contours
                color = np.random.randint(0, 255, 3).tolist()
                cv2.fillPoly(segmented, [contour], color)

        return segmented


def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    detection_node = IsaacObjectDetectionNode()
    segmentation_node = IsaacSegmentationNode()

    # Use multi-threaded executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(detection_node)
    executor.add_node(segmentation_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        detection_node.destroy_node()
        segmentation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
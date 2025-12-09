import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Header
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import threading
import time
from sklearn.cluster import KMeans


class IsaacFeatureExtractionNode(Node):
    """
    Isaac ROS Feature Extraction node for humanoid robot
    Extracts visual and geometric features for perception and recognition
    """
    def __init__(self):
        super().__init__('isaac_feature_extraction')

        # Parameters
        self.declare_parameter('feature_type', 'orb')  # orb, sift, surf, color, shape
        self.declare_parameter('max_features', 500)
        self.declare_parameter('matching_threshold', 0.7)
        self.declare_parameter('descriptor_size', 32)  # For custom descriptors
        self.declare_parameter('publish_frequency', 5.0)  # Hz

        self.feature_type = self.get_parameter('feature_type').value
        self.max_features = self.get_parameter('max_features').value
        self.matching_threshold = self.get_parameter('matching_threshold').value
        self.descriptor_size = self.get_parameter('descriptor_size').value
        self.publish_frequency = self.get_parameter('publish_frequency').value

        # Initialize feature detectors
        self.initialize_feature_detectors()

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
        self.features_pub = self.create_publisher(Float32MultiArray, '/humanoid_robot/features', 10)
        self.feature_image_pub = self.create_publisher(Image, '/humanoid_robot/features_debug', 10)

        # Create timer for processing
        self.process_timer = self.create_timer(1.0 / self.publish_frequency, self.process_features)

        self.latest_image = None
        self.latest_features = None
        self.feature_lock = threading.Lock()

        self.get_logger().info(f'Isaac Feature Extraction node initialized with {self.feature_type} features')

    def initialize_feature_detectors(self):
        """Initialize feature detection algorithms"""
        if self.feature_type == 'orb':
            self.detector = cv2.ORB_create(nfeatures=self.max_features)
        elif self.feature_type == 'sift':
            # Note: SIFT requires opencv-contrib-python
            try:
                self.detector = cv2.SIFT_create()
            except:
                self.get_logger().warn('SIFT not available, falling back to ORB')
                self.detector = cv2.ORB_create(nfeatures=self.max_features)
        elif self.feature_type == 'surf':
            # Note: SURF requires opencv-contrib-python and may not be available
            try:
                self.detector = cv2.xfeatures2d.SURF_create()
            except:
                self.get_logger().warn('SURF not available, falling back to ORB')
                self.detector = cv2.ORB_create(nfeatures=self.max_features)
        else:
            # Default to ORB
            self.detector = cv2.ORB_create(nfeatures=self.max_features)

    def image_callback(self, msg):
        """Receive image for feature extraction"""
        try:
            # Store the latest image
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def process_features(self):
        """Process features on the latest image"""
        if self.latest_image is None:
            return

        try:
            # Extract features
            features_msg, debug_image = self.extract_features(self.latest_image)

            # Update features with thread safety
            with self.feature_lock:
                self.latest_features = features_msg

            # Publish features
            if features_msg is not None:
                features_msg.header.stamp = self.get_clock().now().to_msg()
                features_msg.header.frame_id = 'camera_link'
                self.features_pub.publish(features_msg)

            # Publish debug image
            if debug_image is not None:
                debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
                debug_msg.header.stamp = self.get_clock().now().to_msg()
                debug_msg.header.frame_id = 'camera_link'
                self.feature_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing features: {e}')

    def extract_features(self, image):
        """Extract features from the image"""
        # Convert to grayscale for feature detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect keypoints and compute descriptors
        keypoints, descriptors = self.detector.detectAndCompute(gray, None)

        if descriptors is not None:
            # Create features message
            features_msg = Float32MultiArray()
            # Flatten descriptors and add to data array
            features_msg.data = descriptors.flatten().tolist()
        else:
            # If no features detected, create empty message
            features_msg = Float32MultiArray()
            features_msg.data = []

        # Create debug image with keypoints drawn
        debug_image = cv2.drawKeypoints(
            image, keypoints, None,
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )

        return features_msg, debug_image

    def extract_color_features(self, image):
        """Extract color-based features"""
        # Convert to different color spaces
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

        # Calculate color histograms
        hist_h = cv2.calcHist([hsv], [0], None, [50], [0, 180])
        hist_s = cv2.calcHist([hsv], [1], None, [50], [0, 256])
        hist_v = cv2.calcHist([hsv], [2], None, [50], [0, 256])

        # Normalize histograms
        hist_h = cv2.normalize(hist_h, hist_h).flatten()
        hist_s = cv2.normalize(hist_s, hist_s).flatten()
        hist_v = cv2.normalize(hist_v, hist_v).flatten()

        # Combine all color features
        color_features = np.concatenate([hist_h, hist_s, hist_v])

        return color_features

    def extract_shape_features(self, image):
        """Extract shape-based features"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply threshold
        _, thresh = cv2.threshold(gray, 127, 255, 0)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        shape_features = []
        for contour in contours:
            # Calculate shape descriptors
            area = cv2.contourArea(contour)
            if area > 100:  # Filter small contours
                perimeter = cv2.arcLength(contour, True)
                circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0

                # Bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h if h != 0 else 0

                # Extent (ratio of contour area to bounding rectangle area)
                extent = float(area) / (w * h) if w * h != 0 else 0

                # Add shape features
                shape_features.extend([area, perimeter, circularity, aspect_ratio, extent])

        return np.array(shape_features)


class IsaacPerceptionPipelineNode(Node):
    """
    Isaac ROS Perception Pipeline node for humanoid robot
    Combines multiple perception modules for comprehensive understanding
    """
    def __init__(self):
        super().__init__('isaac_perception_pipeline')

        # Parameters
        self.declare_parameter('enable_object_detection', True)
        self.declare_parameter('enable_segmentation', True)
        self.declare_parameter('enable_feature_extraction', True)
        self.declare_parameter('fusion_frequency', 2.0)  # Hz

        self.enable_object_detection = self.get_parameter('enable_object_detection').value
        self.enable_segmentation = self.get_parameter('enable_segmentation').value
        self.enable_feature_extraction = self.get_parameter('enable_feature_extraction').value
        self.fusion_frequency = self.get_parameter('fusion_frequency').value

        # Initialize perception components
        self.object_detection_results = None
        self.segmentation_results = None
        self.feature_results = None

        # Create subscribers for perception results
        if self.enable_object_detection:
            self.detection_sub = self.create_subscription(
                # In a real implementation, this would subscribe to detection results
            )

        if self.enable_segmentation:
            self.segmentation_sub = self.create_subscription(
                # In a real implementation, this would subscribe to segmentation results
            )

        if self.enable_feature_extraction:
            self.feature_sub = self.create_subscription(
                # In a real implementation, this would subscribe to feature results
            )

        # Create publishers
        self.scene_description_pub = self.create_publisher(
            # In a real implementation, this would publish a comprehensive scene description
        )

        # Create timer for perception fusion
        self.fusion_timer = self.create_timer(1.0 / self.fusion_frequency, self.fuse_perception)

        self.get_logger().info('Isaac Perception Pipeline node initialized')

    def fuse_perception(self):
        """Fuse results from different perception modules"""
        # This is where perception results would be combined
        # to create a comprehensive understanding of the scene
        pass


def main(args=None):
    rclpy.init(args=args)

    # Create feature extraction node
    feature_node = IsaacFeatureExtractionNode()
    # Create perception pipeline node
    pipeline_node = IsaacPerceptionPipelineNode()

    # Use multi-threaded executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(feature_node)
    executor.add_node(pipeline_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        feature_node.destroy_node()
        pipeline_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
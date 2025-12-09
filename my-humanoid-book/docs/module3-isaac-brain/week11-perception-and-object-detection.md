---
sidebar_position: 3
---

# Week 11: Perception and Object Detection

## Learning Objectives

By the end of this week, you will be able to:
- Implement computer vision techniques for object detection and recognition
- Extract and match visual features using ORB, SIFT, and other algorithms
- Perform semantic and instance segmentation for scene understanding
- Integrate perception with navigation and mapping systems
- Test perception accuracy in various lighting and environmental conditions

## Perception in Robotics

Perception is the robot's ability to understand and interpret its environment through sensors. For humanoid robots, perception systems typically include:

- **Visual perception**: Camera-based object detection and recognition
- **Depth perception**: 3D scene understanding using stereo vision or depth sensors
- **Feature extraction**: Identifying distinctive elements in the environment
- **Scene segmentation**: Understanding spatial relationships between objects

## Feature Extraction and Matching

Feature extraction is fundamental to visual perception, allowing robots to identify and match distinctive elements in their environment.

### ORB (Oriented FAST and Rotated BRIEF)

ORB is an efficient feature detection algorithm that combines FAST corner detection with BRIEF descriptors:

```python
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

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

def main(args=None):
    rclpy.init(args=args)
    feature_node = IsaacFeatureExtractionNode()

    try:
        rclpy.spin(feature_node)
    except KeyboardInterrupt:
        pass
    finally:
        feature_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Object Detection with Multiple Approaches

Our perception system implements multiple object detection approaches to handle various scenarios:

### Haar Cascade Detection

```python
class IsaacObjectDetectionNode(Node):
    """
    Isaac ROS Object Detection node for humanoid robot
    Performs real-time object detection using computer vision techniques
    """
    def __init__(self):
        super().__init__('isaac_object_detection')

        # Parameters
        self.declare_parameter('detection_model', 'haar')  # yolo, ssd, haar, color
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

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image, '/humanoid_robot/camera/image_raw', self.image_callback, 10)

        # Create publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/humanoid_robot/detections', 10)
        self.debug_image_pub = self.create_publisher(Image, '/humanoid_robot/detection_debug', 10)

        # Create timer for publishing detections
        self.detection_timer = self.create_timer(1.0 / self.detection_frequency, self.publish_detections)

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

    def image_callback(self, msg):
        """Process image for object detection"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform object detection
            detections = self.detect_objects(cv_image)

            # Create debug image with detections
            debug_image = self.draw_detections(cv_image, detections)
            debug_image_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_image_msg.header.stamp = msg.header.stamp
            debug_image_msg.header.frame_id = msg.header.frame_id
            self.debug_image_pub.publish(debug_image_msg)

            # Publish detections
            detections.header.stamp = self.get_clock().now().to_msg()
            detections.header.frame_id = 'camera_link'
            self.detection_pub.publish(detections)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_objects(self, image, depth_image=None):
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
                detection = self.create_detection(x, y, w, h, "face", 0.8, depth_image)
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

    def create_detection(self, x, y, w, h, label, confidence, depth_image=None):
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
                    # Calculate 3D position using camera parameters
                    # (simplified - would need actual camera calibration in practice)
                    pos_x = (center_x - depth_image.shape[1]/2) * depth / 500.0  # focal length approximation
                    pos_y = (center_y - depth_image.shape[0]/2) * depth / 500.0
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
        pass  # Handled in image_callback
```

## Semantic Segmentation

Semantic segmentation provides pixel-level understanding of scenes:

```python
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

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image, '/humanoid_robot/camera/image_raw', self.image_callback, 10)

        # Create publishers
        self.segmentation_pub = self.create_publisher(Image, '/humanoid_robot/segmentation', 10)

        # Create timer for processing
        self.process_timer = self.create_timer(1.0 / self.publish_frequency, self.process_segmentation)

        self.latest_image = None

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
```

## Perception Pipeline Integration

The perception system is integrated into a comprehensive pipeline that combines multiple modalities:

```python
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
                Detection2DArray, '/humanoid_robot/detections', self.detection_callback, 10)

        if self.enable_segmentation:
            self.segmentation_sub = self.create_subscription(
                Image, '/humanoid_robot/segmentation', self.segmentation_callback, 10)

        if self.enable_feature_extraction:
            self.feature_sub = self.create_subscription(
                Float32MultiArray, '/humanoid_robot/features', self.feature_callback, 10)

        # Create publishers
        self.scene_description_pub = self.create_publisher(SceneDescription, '/humanoid_robot/scene_description', 10)

        # Create timer for perception fusion
        self.fusion_timer = self.create_timer(1.0 / self.fusion_frequency, self.fuse_perception)

        self.get_logger().info('Isaac Perception Pipeline node initialized')

    def detection_callback(self, msg):
        """Handle object detection results"""
        self.object_detection_results = msg

    def segmentation_callback(self, msg):
        """Handle segmentation results"""
        self.segmentation_results = msg

    def feature_callback(self, msg):
        """Handle feature extraction results"""
        self.feature_results = msg

    def fuse_perception(self):
        """Fuse results from different perception modules"""
        if not all([self.object_detection_results, self.segmentation_results, self.feature_results]):
            return

        try:
            # Create comprehensive scene description
            scene_description = SceneDescription()
            scene_description.header.stamp = self.get_clock().now().to_msg()
            scene_description.header.frame_id = 'map'

            # Combine all perception results
            scene_description.objects = self.object_detection_results.detections
            scene_description.features = self.feature_results.data
            # Process segmentation results for scene understanding

            # Publish comprehensive scene description
            self.scene_description_pub.publish(scene_description)

            # Reset results after fusion
            self.object_detection_results = None
            self.segmentation_results = None
            self.feature_results = None

        except Exception as e:
            self.get_logger().error(f'Error fusing perception: {e}')

def main(args=None):
    rclpy.init(args=args)

    # Create all perception nodes
    feature_node = IsaacFeatureExtractionNode()
    detection_node = IsaacObjectDetectionNode()
    segmentation_node = IsaacSegmentationNode()
    pipeline_node = IsaacPerceptionPipelineNode()

    # Use multi-threaded executor to handle all nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(feature_node)
    executor.add_node(detection_node)
    executor.add_node(segmentation_node)
    executor.add_node(pipeline_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        feature_node.destroy_node()
        detection_node.destroy_node()
        segmentation_node.destroy_node()
        pipeline_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Navigation and Mapping

The perception system is tightly integrated with navigation and mapping to enable intelligent behavior:

```python
# In the navigation system, perception results can influence path planning
def update_costmap_with_perception(self, scene_description):
    """Update navigation costmap based on perception results"""
    for obj in scene_description.objects:
        # Get object position and type
        pos = obj.pose.position
        obj_type = obj.results[0].id if obj.results else "unknown"

        # Update costmap based on object type
        if obj_type in ["person", "obstacle", "furniture"]:
            # Inflate cost around object
            self.costmap.update_with_dynamic_obstacle(pos, inflation_radius=0.3)
        elif obj_type in ["door", "passage"]:
            # Reduce cost to encourage navigation toward passages
            self.costmap.reduce_cost_at_position(pos, reduction_factor=0.5)
```

## Practical Exercise: Perception Testing

1. Test object detection in various lighting conditions
2. Evaluate feature extraction performance with different algorithms
3. Assess segmentation accuracy in complex scenes
4. Integrate perception with navigation to avoid detected obstacles

## Summary



This week, we've completed the NVIDIA Isaac perception system for our humanoid robot. We've covered:



- Feature extraction using ORB, SIFT, and other algorithms

- Multiple object detection approaches (Haar cascades, color-based, shape-based)

- Semantic segmentation for scene understanding

- Perception pipeline integration

- Integration with navigation and mapping systems



With this complete Isaac implementation, our humanoid robot now has the core AI capabilities for SLAM, navigation, and perception that form its "brain." The robot can map its environment, navigate autonomously, and understand what it sees - essential capabilities for advanced humanoid robotics applications.



## Challenges and Extensions



- **Challenge 1:** Train a custom Haar cascade or a deep learning model (e.g., YOLO) to detect specific objects relevant to your robot's tasks.

- **Challenge 2:** Implement a 3D object detection pipeline using depth information from a stereo camera or a depth sensor.

- **Challenge 3:** Use the perception system to build a semantic map of the environment, where different areas are labeled with their properties (e.g., "kitchen," "hallway").

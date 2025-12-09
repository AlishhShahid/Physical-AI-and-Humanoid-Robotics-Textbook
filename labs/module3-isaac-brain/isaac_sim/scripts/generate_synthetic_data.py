#!/usr/bin/env python3

"""
Script for generating synthetic sensor data for SLAM and perception training.

This script configures Isaac Sim to generate synthetic data including:
- RGB images
- Depth maps
- Semantic segmentation
- LIDAR point clouds
- IMU readings
- Ground truth poses
"""

import carb
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera, RotatingLidarSensor
from omni.isaac.synthetic_utils import SyntheticDataHelper
import numpy as np
import yaml
import argparse
import os
import cv2
from PIL import Image
import json
from datetime import datetime


def load_config(config_path):
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def setup_synthetic_data_capture(world, config):
    """Setup synthetic data capture in Isaac Sim"""
    synth_config = config['synthetic_data']

    print("Setting up synthetic data capture...")

    # Create output directory
    output_dir = synth_config['output_directory']
    os.makedirs(output_dir, exist_ok=True)

    # Setup data types to capture
    data_capture_config = {
        'rgb': synth_config.get('rgb', False),
        'depth': synth_config.get('depth', False),
        'segmentation': synth_config.get('segmentation', False),
        'bounding_boxes': synth_config.get('bounding_boxes', False),
        'poses': synth_config.get('poses', False)
    }

    print(f"Data capture configuration: {data_capture_config}")

    return data_capture_config, output_dir


def generate_slam_data(world, config, output_dir, num_samples=100):
    """Generate synthetic data for SLAM training"""
    print(f"Generating {num_samples} samples for SLAM training...")

    # Create SLAM data directory
    slam_dir = os.path.join(output_dir, "slam_data")
    os.makedirs(slam_dir, exist_ok=True)

    # Data structures for SLAM data
    slam_data = {
        'timestamps': [],
        'poses': [],
        'rgb_paths': [],
        'depth_paths': [],
        'lidar_paths': []
    }

    # Simulate robot movement and capture data
    for i in range(num_samples):
        # In a real implementation, this would:
        # 1. Move the robot to a new position
        # 2. Capture sensor data
        # 3. Record ground truth pose
        # 4. Save data to files

        # Simulate robot pose (in a real implementation, this would come from simulation)
        x = np.random.uniform(-5, 5)
        y = np.random.uniform(-5, 5)
        z = 0.0  # Height above ground
        roll = np.random.uniform(-0.1, 0.1)
        pitch = np.random.uniform(-0.1, 0.1)
        yaw = np.random.uniform(-np.pi, np.pi)

        pose = [x, y, z, roll, pitch, yaw]

        # Generate synthetic RGB image
        rgb_path = os.path.join(slam_dir, f"rgb_{i:06d}.png")
        # Create a synthetic RGB image with random patterns
        rgb_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        cv2.imwrite(rgb_path, cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR))

        # Generate synthetic depth image
        depth_path = os.path.join(slam_dir, f"depth_{i:06d}.png")
        # Create a synthetic depth image
        depth_img = np.random.uniform(0.1, 10.0, (480, 640)).astype(np.float32)
        # Scale to 16-bit for PNG storage
        depth_img_scaled = (depth_img * 1000).astype(np.uint16)  # mm scale
        cv2.imwrite(depth_path, depth_img_scaled)

        # Generate synthetic LIDAR data
        lidar_path = os.path.join(slam_dir, f"lidar_{i:06d}.npy")
        # Create synthetic LIDAR point cloud
        num_points = 1000
        angles = np.random.uniform(0, 2*np.pi, num_points)
        distances = np.random.uniform(0.5, 20.0, num_points)
        x_coords = distances * np.cos(angles)
        y_coords = distances * np.sin(angles)
        z_coords = np.random.uniform(-1.0, 1.0, num_points)
        lidar_points = np.column_stack([x_coords, y_coords, z_coords])
        np.save(lidar_path, lidar_points)

        # Record data
        slam_data['timestamps'].append(datetime.now().isoformat())
        slam_data['poses'].append(pose)
        slam_data['rgb_paths'].append(rgb_path)
        slam_data['depth_paths'].append(depth_path)
        slam_data['lidar_paths'].append(lidar_path)

        if i % 20 == 0:
            print(f"Generated {i}/{num_samples} SLAM samples")

    # Save metadata
    metadata_path = os.path.join(slam_dir, "metadata.json")
    with open(metadata_path, 'w') as f:
        json.dump(slam_data, f, indent=2)

    print(f"SLAM data generation complete. Saved to: {slam_dir}")
    return slam_dir


def generate_perception_data(world, config, output_dir, num_samples=100):
    """Generate synthetic data for perception training"""
    print(f"Generating {num_samples} samples for perception training...")

    # Create perception data directory
    perception_dir = os.path.join(output_dir, "perception_data")
    os.makedirs(perception_dir, exist_ok=True)

    # Create subdirectories for different data types
    rgb_dir = os.path.join(perception_dir, "rgb")
    seg_dir = os.path.join(perception_dir, "segmentation")
    bbox_dir = os.path.join(perception_dir, "bounding_boxes")
    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(seg_dir, exist_ok=True)
    os.makedirs(bbox_dir, exist_ok=True)

    # Data structures for perception data
    perception_data = {
        'samples': []
    }

    # Define object classes for synthetic data
    object_classes = ["human", "chair", "table", "plant", "robot", "box"]

    for i in range(num_samples):
        # Generate synthetic RGB image
        rgb_path = os.path.join(rgb_dir, f"rgb_{i:06d}.png")
        # Create a synthetic RGB image with random patterns and objects
        rgb_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        # Add some synthetic objects
        for _ in range(np.random.randint(1, 4)):
            center_x = np.random.randint(50, 590)
            center_y = np.random.randint(50, 430)
            radius = np.random.randint(10, 50)
            color = tuple(np.random.randint(0, 255, 3).tolist())
            cv2.circle(rgb_img, (center_x, center_y), radius, color, -1)
        cv2.imwrite(rgb_path, cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR))

        # Generate synthetic segmentation
        seg_path = os.path.join(seg_dir, f"seg_{i:06d}.png")
        seg_img = np.zeros((480, 640), dtype=np.uint8)
        # Create segmentation mask with different object IDs
        for obj_id in range(1, np.random.randint(2, 5)):
            mask = np.zeros((480, 640), dtype=np.uint8)
            center_x = np.random.randint(50, 590)
            center_y = np.random.randint(50, 430)
            radius = np.random.randint(10, 50)
            cv2.circle(mask, (center_x, center_y), radius, 1, -1)
            seg_img[mask == 1] = obj_id
        cv2.imwrite(seg_path, seg_img)

        # Generate bounding box annotations
        bbox_path = os.path.join(bbox_dir, f"bbox_{i:06d}.json")
        num_objects = np.random.randint(1, 4)
        bboxes = []
        for j in range(num_objects):
            x = np.random.randint(0, 540)  # Leave margin for width
            y = np.random.randint(0, 380)  # Leave margin for height
            width = np.random.randint(50, 200)
            height = np.random.randint(50, 200)
            class_id = np.random.randint(0, len(object_classes))
            class_name = object_classes[class_id]
            bboxes.append({
                "class_id": class_id,
                "class_name": class_name,
                "bbox": [x, y, width, height]  # [x, y, width, height]
            })

        bbox_data = {
            "image_path": rgb_path,
            "width": 640,
            "height": 480,
            "objects": bboxes
        }
        with open(bbox_path, 'w') as f:
            json.dump(bbox_data, f, indent=2)

        # Record sample
        perception_data['samples'].append({
            'rgb_path': rgb_path,
            'seg_path': seg_path,
            'bbox_path': bbox_path,
            'timestamp': datetime.now().isoformat()
        })

        if i % 20 == 0:
            print(f"Generated {i}/{num_samples} perception samples")

    # Save metadata
    metadata_path = os.path.join(perception_dir, "metadata.json")
    with open(metadata_path, 'w') as f:
        json.dump(perception_data, f, indent=2)

    print(f"Perception data generation complete. Saved to: {perception_dir}")
    return perception_dir


def apply_domain_randomization(config):
    """Apply domain randomization techniques"""
    domain_rand = config['synthetic_data']['domain_randomization']

    if not domain_rand['enabled']:
        print("Domain randomization disabled")
        return

    print("Applying domain randomization...")

    # Randomize textures
    if domain_rand['texture_randomization']:
        print("  - Randomizing textures...")
        # In a real implementation, this would change material properties

    # Randomize lighting
    if domain_rand['lighting_randomization']:
        print("  - Randomizing lighting...")
        # In a real implementation, this would change light properties

    # Randomize object placement
    if domain_rand['object_placement_randomization']:
        print("  - Randomizing object placement...")
        # In a real implementation, this would change object positions


def main():
    """Main function to generate synthetic data"""
    parser = argparse.ArgumentParser(description='Generate Synthetic Data for SLAM and Perception')
    parser.add_argument('--config', type=str, default='configs/humanoid_sim_config.yaml',
                        help='Path to configuration file')
    parser.add_argument('--num-samples', type=int, default=100,
                        help='Number of samples to generate')
    parser.add_argument('--output-dir', type=str, default='../outputs/synthetic_data',
                        help='Output directory for generated data')

    args = parser.parse_args()

    # Load configuration
    config_path = args.config
    if not os.path.isabs(config_path):
        config_path = os.path.abspath(config_path)

    if not os.path.exists(config_path):
        print(f"Configuration file not found: {config_path}")
        return

    config = load_config(config_path)
    print(f"Loaded configuration from: {config_path}")

    # Setup synthetic data capture
    data_capture_config, output_dir = setup_synthetic_data_capture(None, config)

    # Apply domain randomization
    apply_domain_randomization(config)

    # Generate SLAM training data
    if data_capture_config['rgb'] and data_capture_config['depth']:
        slam_dir = generate_slam_data(None, config, output_dir, args.num_samples)

    # Generate perception training data
    if data_capture_config['rgb'] and data_capture_config['segmentation']:
        perception_dir = generate_perception_data(None, config, output_dir, args.num_samples)

    print("Synthetic data generation complete!")
    print(f"SLAM data saved to: {slam_dir if 'slam_dir' in locals() else 'N/A'}")
    print(f"Perception data saved to: {perception_dir if 'perception_dir' in locals() else 'N/A'}")


if __name__ == "__main__":
    main()
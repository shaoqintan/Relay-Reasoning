#!/usr/bin/env python3
"""
Standalone version of the ROS object detection script.
This version is designed to be called from the wrapper script and includes
better error handling and output formatting.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import sys
import os
import json
import requests
import time

class Object3DDetectorStandalone:
    def __init__(self):
        rospy.init_node('object_3d_detector_standalone', anonymous=True)
        self.bridge = CvBridge()

        # Subscribers for RGB image, depth image, and camera info
        self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_callback)

        # TF listener for transforming points to base frame.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        
        # URL for the Gemini service
        self.gemini_service_url = "http://localhost:5000/detect_object"
        
        # Output file for results
        self.output_file = "/tmp/ros_detection_result.json"
        
        # Ensure temp directory exists
        os.makedirs("/tmp", exist_ok=True)

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        # Get depth image; ensure it is in meters (common for depthai)
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def info_callback(self, msg):
        self.camera_info = msg

    def detect_object(self, image_path):
        """
        Detect object using the external Gemini service
        Returns the center coordinates of the detected object
        """
        try:
            # Call the Gemini service API
            response = requests.post(
                self.gemini_service_url,
                json={"image_path": image_path},
                timeout=30
            )
            
            if response.status_code != 200:
                print(f"Error from Gemini service: {response.text}")
                return None
                
            result = response.json()
            center_x = result["center_x"]
            center_y = result["center_y"]
            
            print(f"Detected object center: ({center_x}, {center_y})")
            return (center_x, center_y)
            
        except Exception as e:
            print(f"Error calling Gemini service: {str(e)}")
            return None

    def compute_3d_point(self, u, v):
        if self.depth_image is None or self.camera_info is None:
            return None
        # Get depth at pixel (u, v); adjust by averaging if needed.
        depth = self.depth_image[v, u]
        depth = depth/1000
        if depth == 0:
            print("No valid depth at pixel (%d, %d)", u, v)
            return None

        # Get camera intrinsics from the camera_info K matrix
        K = np.array(self.camera_info.K).reshape(3, 3)
        fx = K[0,0]
        fy = K[1,1]
        cx = K[0,2]
        cy = K[1,2]
        
        # Deproject the pixel (u, v) into 3D camera coordinates.
        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth
        return X, Y, Z
    
    def capture_and_process_frame(self):
        """Capture a single frame from the camera stream and process it"""
        # Wait for subscribers to connect and receive initial data
        print("Waiting for camera data...")
        timeout = 30  # 30 second timeout
        start_time = time.time()
        
        while not rospy.is_shutdown() and (self.rgb_image is None or self.depth_image is None or self.camera_info is None):
            if time.time() - start_time > timeout:
                print("Timeout waiting for camera data")
                return None
            rospy.sleep(0.1)
        
        # Capture current RGB and depth frames
        current_rgb = self.rgb_image.copy()
        current_depth = self.depth_image.copy()
        
        # Print debugging information about the images
        print(f"RGB image shape: {current_rgb.shape}, dtype: {current_rgb.dtype}")
        print(f"Depth image shape: {current_depth.shape}, dtype: {current_depth.dtype}")
        print(f"Depth image min: {np.min(current_depth)}, max: {np.max(current_depth)}, mean: {np.mean(current_depth)}")
        
        # Save the current RGB image
        save_path = "/home/er/Documents/reasoning320/googleResoning/rosbridge/current_frame.png"
        
        # Ensure directory exists
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        
        cv2.imwrite(save_path, current_rgb)
        print(f"Saved current camera frame to {save_path}")
        
        # Run object detection on the saved image
        result = self.detect_object(save_path)
        if result is None:
            print("No object detected in the image")
            return None
        
        u, v = result
        print(f"Object detection result - pixel coordinates: ({u}, {v})")
        
        # Compute 3D point using the captured depth frame
        # Temporarily store the current depth image
        temp_depth = self.depth_image
        self.depth_image = current_depth
        
        # Print depth value at the detected point
        if 0 <= v < current_depth.shape[0] and 0 <= u < current_depth.shape[1]:
            depth_at_point = current_depth[v, u]
            print(f"Depth at detected point ({u}, {v}): {depth_at_point}")
            
            # Check surrounding depth values to help with debugging
            window_size = 5
            v_start = max(0, v - window_size//2)
            v_end = min(current_depth.shape[0], v + window_size//2 + 1)
            u_start = max(0, u - window_size//2)
            u_end = min(current_depth.shape[1], u + window_size//2 + 1)
            
            depth_window = current_depth[v_start:v_end, u_start:u_end]
            print(f"Depth values in {window_size}x{window_size} window around point:")
            print(f"Window min: {np.min(depth_window)}, max: {np.max(depth_window)}, mean: {np.mean(depth_window)}")
        else:
            print(f"Detected point ({u}, {v}) is outside depth image bounds: {current_depth.shape}")
        
        point3d = self.compute_3d_point(u, v)
        
        # Restore the original depth image
        self.depth_image = temp_depth
        
        if point3d is None:
            print("Could not compute 3D point for the detected object")
            return None
            
        X, Y, Z = point3d
        print("Object in camera frame: X=%.3f, Y=%.3f, Z=%.3f", X, Y, Z)
        
        # Print camera intrinsics for debugging
        if self.camera_info is not None:
            K = np.array(self.camera_info.K).reshape(3, 3)
            print(f"Camera intrinsics matrix K:\n{K}")
            print(f"fx: {K[0,0]}, fy: {K[1,1]}, cx: {K[0,2]}, cy: {K[1,2]}")
        
        # Create a PointStamped in the camera frame
        point_cam = PointStamped()
        point_cam.header.stamp = rospy.Time(0)
        point_cam.header.frame_id = "camera_color_optical_frame"
        point_cam.point.x = X
        point_cam.point.y = Y
        point_cam.point.z = Z
        
        # Transform this point to the robot base frame
        try:
            # Print available frames for debugging
            frames = self.tf_buffer.all_frames_as_string()
            print(f"Available TF frames:\n{frames}")
            
            point_base = self.tf_buffer.transform(point_cam, "base", rospy.Duration(4.0))
            print("Object in robot base frame: X=%.3f, Y=%.3f, Z=%.3f",
                  point_base.point.x, point_base.point.y, point_base.point.z)
            
            # Save result to file
            result_data = {
                "success": True,
                "camera_frame": {"x": X, "y": Y, "z": Z},
                "base_frame": {"x": point_base.point.x, "y": point_base.point.y, "z": point_base.point.z},
                "pixel_coordinates": {"u": u, "v": v},
                "timestamp": time.time()
            }
            
            with open(self.output_file, 'w') as f:
                json.dump(result_data, f, indent=2)
            
            return point_base
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            print("TF transform failed: %s", str(e))
            # Try to check if the transform exists at all
            if self.tf_buffer.can_transform("camera_color_optical_frame", "base", rospy.Time(0), 
                                           rospy.Duration(0.1)):
                print("Transform exists but timing is off. Using latest available transform.")
                try:
                    point_cam.header.stamp = rospy.Time(0)
                    point_base = self.tf_buffer.transform(point_cam, "base", rospy.Duration(0.1))
                    print("Object in robot base frame (using latest transform): X=%.3f, Y=%.3f, Z=%.3f",
                         point_base.point.x, point_base.point.y, point_base.point.z)
                    
                    # Save result to file
                    result_data = {
                        "success": True,
                        "camera_frame": {"x": X, "y": Y, "z": Z},
                        "base_frame": {"x": point_base.point.x, "y": point_base.point.y, "z": point_base.point.z},
                        "pixel_coordinates": {"u": u, "v": v},
                        "timestamp": time.time(),
                        "note": "Used latest available transform"
                    }
                    
                    with open(self.output_file, 'w') as f:
                        json.dump(result_data, f, indent=2)
                    
                    return point_base
                except Exception as e2:
                    print("Second transform attempt failed: %s", str(e2))
            else:
                print("Transform between frames does not exist")
            
            # Save error result to file
            result_data = {
                "success": False,
                "error": "TF transform failed",
                "camera_frame": {"x": X, "y": Y, "z": Z},
                "pixel_coordinates": {"u": u, "v": v},
                "timestamp": time.time()
            }
            
            with open(self.output_file, 'w') as f:
                json.dump(result_data, f, indent=2)
        
        return None

    def run(self):
        print("Starting object detection and 3D position calculation...")
        result = self.capture_and_process_frame()
        
        if result:
            print("Object detected and 3D position calculated successfully")
            print(f"Results saved to: {self.output_file}")
        else:
            print("Failed to detect object or calculate 3D position")

if __name__ == '__main__':
    try:
        detector = Object3DDetectorStandalone()
        detector.run()
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {str(e)}")
        # Save error to file
        error_data = {
            "success": False,
            "error": str(e),
            "timestamp": time.time()
        }
        with open("/tmp/ros_detection_result.json", 'w') as f:
            json.dump(error_data, f, indent=2) 
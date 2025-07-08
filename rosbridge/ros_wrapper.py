#!/usr/bin/env python3
"""
Wrapper script to run ROS-dependent code using system Python environment.
This script can be called from any environment and will execute the ROS code
using the system's Python installation.
"""

import subprocess
import sys
import os
import json
import requests
from pathlib import Path

class ROSWrapper:
    def __init__(self):
        # Path to the ROS script (use the standalone version)
        self.ros_script_path = Path(__file__).parent / "rospygemini1_standalone.py"
        
        # URL for the Gemini service (FastAPI)
        self.gemini_service_url = "http://localhost:5000/detect_object"
        
        # Output file where ROS script saves results
        self.result_file = "/tmp/ros_detection_result.json"
        
        # System Python path (adjust this based on your system)
        # Common system Python paths:
        # Linux: /usr/bin/python3
        # macOS: /usr/bin/python3
        
        self.system_python = "/usr/bin/python3"
    
    def run_scene_capture(self):
        """
        Run just the scene capture functionality using system Python
        This is a simplified version that only captures the image without object detection
        """
        try:
            # Create a simple scene capture script
            scene_capture_script = """
#!/usr/bin/python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
import sys
import json
import requests
from pymycobot import MyCobot320
from pymycobot import PI_PORT, PI_BAUD
import time
import os

class SceneCapture:
    def __init__(self):
        rospy.init_node('object_3d_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # Subscribers for RGB image
        self.rgb_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        
        self.rgb_image = None
        self.camera_info = None

        self.gemini_service_url = "http://localhost:5000/detect_object"

        self.mc = MyCobot320(PI_PORT, 115200)
        
    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def info_callback(self, msg):
        self.camera_info = msg
    
    def capture_scene(self):
        print("Waiting for camera data...")
        # timeout = 30
        # start_time = time.time()
        
        while not rospy.is_shutdown() and self.rgb_image is None:
            # if time.time() - start_time > timeout:
            #     print("Timeout waiting for camera data")
            #     return False
            rospy.sleep(0.1)
        
        # Capture current RGB frame
        current_rgb = self.rgb_image.copy()
        
        print(f"RGB image shape: {current_rgb.shape}, dtype: {current_rgb.dtype}")
        
        # Save the current RGB image - use Linux path
        save_path = "/home/er/Documents/reasoning320/googleResoning/rosbridge/current_frame.png"
        
        # Ensure directory exists
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        
        ok = cv2.imwrite(save_path, current_rgb)
        print(f"value of write is {ok}")
        print(f"Saved current camera frame to {save_path}")
        return True

if __name__ == '__main__':
    try:
        capture = SceneCapture()
        success = capture.capture_scene()
        if success:
            print("Scene capture completed successfully HELLOOOOOOOOOOOOOOOOOO")
        else:
            print("Scene capture failed okokokook")
    except Exception as e:
        print(f"Error: {str(e)}")
"""
            
            # Write the temporary script
            temp_script_path = "/tmp/scene_capture_temp.py"
            with open(temp_script_path, 'w') as f:
                f.write(scene_capture_script)
            
            # Run the scene capture script using system Python
            ros_cmd = [
                "/bin/bash", "-c",
                "source /opt/ros/noetic/setup.bash && "
                "source ~/catkin_ws/devel/setup.bash && "
                f"/usr/bin/python3 {temp_script_path}"
            ]
            
            print(f"Running scene capture with command: {' '.join(ros_cmd)}")
            
            # Execute the command
            result = subprocess.run(
                ros_cmd,
                capture_output=True,
                text=True,
                timeout=60  # 60 second timeout
            )
            
            # Clean up temporary script
            try:
                os.remove(temp_script_path)
            except:
                pass
            
            if result.returncode == 0:
                print("Scene capture completed successfully")
                print("Output:", result.stdout)
                return True
            else:
                print(f"Scene capture failed with return code: {result.returncode}")
                print("Error output:", result.stderr)
                return False
                
        except subprocess.TimeoutExpired:
            print("Scene capture timed out")
            return False
        except FileNotFoundError:
            print(f"Error: System Python not found at {self.system_python}")
            print("Please update the system_python path in this script")
            return False
        except Exception as e:
            print(f"Error running scene capture: {str(e)}")
            return False
    
    def run_ros_detection(self):
        """
        Run the ROS object detection using system Python
        """
        try:
            # Check if the ROS script exists
            if not self.ros_script_path.exists():
                print(f"Error: ROS script not found at {self.ros_script_path}")
                return None
            
            # Run the ROS script using system Python
            cmd = [self.system_python, str(self.ros_script_path)]
            
            print(f"Running ROS detection with command: {' '.join(cmd)}")
            
            # Execute the command
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=60  # 60 second timeout
            )
            
            if result.returncode == 0:
                print("ROS detection completed successfully")
                print("Output:", result.stdout)
                
                # Read the results from the output file
                return self.read_results()
            else:
                print(f"ROS detection failed with return code: {result.returncode}")
                print("Error output:", result.stderr)
                return None
                
        except subprocess.TimeoutExpired:
            print("ROS detection timed out")
            return None
        except FileNotFoundError:
            print(f"Error: System Python not found at {self.system_python}")
            print("Please update the system_python path in this script")
            return None
        except Exception as e:
            print(f"Error running ROS detection: {str(e)}")
            return None
    
    def read_results(self):
        """
        Read the results from the output file created by the ROS script
        """
        try:
            if os.path.exists(self.result_file):
                with open(self.result_file, 'r') as f:
                    results = json.load(f)
                return results
            else:
                print(f"Result file not found at {self.result_file}")
                return None
        except Exception as e:
            print(f"Error reading results: {str(e)}")
            return None
    
    def test_system_python(self):
        """
        Test if the system Python can import ROS modules
        """
        try:
            cmd = [self.system_python, "-c", "import rospy; print('ROS import successful')"]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                print("✓ System Python can import ROS modules")
                return True
            else:
                print("✗ System Python cannot import ROS modules")
                print("Error:", result.stderr)
                return False
                
        except Exception as e:
            print(f"Error testing system Python: {str(e)}")
            return False
    
    def get_detection_results(self):
        """
        Main method to get detection results
        """
        # Test system Python first
        print("Testing system Python ROS compatibility...")
        if not self.test_system_python():
            print("Please ensure ROS is installed in the system Python environment")
            return None
        
        # Run ROS detection
        print("\nStarting ROS object detection...")
        results = self.run_ros_detection()
        
        if results:
            if results.get("success", False):
                print("Object detection successful!")
                print(f"Camera frame coordinates: {results['camera_frame']}")
                if 'base_frame' in results:
                    print(f"Base frame coordinates: {results['base_frame']}")
                print(f"Pixel coordinates: {results['pixel_coordinates']}")
            else:
                print("Object detection failed!")
                if 'error' in results:
                    print(f"Error: {results['error']}")
            
            return results
        else:
            print("ROS detection failed!")
            return None

def main():
    wrapper = ROSWrapper()
    results = wrapper.get_detection_results()
    
    if results:
        print("\nDetection completed!")
        return results
    else:
        print("\nDetection failed!")
        return None

if __name__ == "__main__":
    main() 
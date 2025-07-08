#!/usr/bin/env python3
"""
Example script showing how to use the ROS wrapper from your current environment.
This script can be run from any environment and will execute the ROS-dependent
code using the system Python environment.
"""

from ros_wrapper import ROSWrapper
import json

def main():
    print("=== ROS Object Detection Example ===")
    print("This script will run ROS-dependent code using system Python")
    print("while being executed from your current environment.\n")
    
    # Create the wrapper
    wrapper = ROSWrapper()
    
    # Get detection results
    results = wrapper.get_detection_results()
    
    if results:
        print("\n=== Detection Results ===")
        print(json.dumps(results, indent=2))
        
        if results.get("success", False):
            print("\n=== Summary ===")
            camera_coords = results["camera_frame"]
            print(f"Object detected at camera coordinates: X={camera_coords['x']:.3f}, Y={camera_coords['y']:.3f}, Z={camera_coords['z']:.3f}")
            
            if "base_frame" in results:
                base_coords = results["base_frame"]
                print(f"Object in base frame: X={base_coords['x']:.3f}, Y={base_coords['y']:.3f}, Z={base_coords['z']:.3f}")
            
            pixel_coords = results["pixel_coordinates"]
            print(f"Pixel coordinates: u={pixel_coords['u']}, v={pixel_coords['v']}")
        else:
            print(f"\nDetection failed: {results.get('error', 'Unknown error')}")
    else:
        print("\nFailed to get detection results")

if __name__ == "__main__":
    main() 
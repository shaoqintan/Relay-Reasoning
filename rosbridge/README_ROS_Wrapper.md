# ROS Wrapper Solution

This solution allows you to run ROS-dependent code from any Python environment by using the system's Python installation (which has ROS installed) while staying in your current environment.

## Problem
- Your current environment has FastAPI but not ROS
- The `rospygemini1.py` file requires ROS modules (`rospy`, `cv_bridge`, etc.)
- The `vision_sceneunderstanding.py` file imports `Object3DDetector` from `rospygemini1.py`
- You want to run the ROS code using the system's Python environment

## Solution
The wrapper system consists of several components:

1. **`ros_wrapper.py`** - Main wrapper that calls the ROS script using system Python
2. **`rospygemini1_standalone.py`** - Modified version of your ROS script optimized for standalone execution
3. **`example_usage.py`** - Example showing how to use the wrapper
4. **Updated `vision_sceneunderstanding.py`** - Modified to use the wrapper instead of direct imports

## How to Use

### 1. Basic Usage
From your current environment, run:
```bash
python ros_wrapper.py
```

### 2. Using the Wrapper in Your Code
```python
from ros_wrapper import ROSWrapper

# Create wrapper instance
wrapper = ROSWrapper()

# Get detection results
results = wrapper.get_detection_results()

if results and results.get("success", False):
    print(f"Object detected at: {results['camera_frame']}")
    if 'base_frame' in results:
        print(f"Base frame coordinates: {results['base_frame']}")
```

### 3. Scene Capture Only
For just capturing images (like in vision_sceneunderstanding.py):
```python
from ros_wrapper import ROSWrapper

wrapper = ROSWrapper()
success = wrapper.run_scene_capture()

if success:
    print("Scene captured successfully")
    # Image saved to /home/er/Documents/reasoning320/googleResoning/rosbridge/current_frame.png
```

### 4. Vision Integration
The `vision_sceneunderstanding.py` file has been updated to use the wrapper:
```python
from language_models.vision_sceneunderstanding import VLM

vlm = VLM()
description = vlm.describe_and_save("What do you see?")
print(description)
```

### 5. Testing the Integration
Run the test script to verify everything works:
```bash
python test_vision_integration.py
```

## Configuration

### System Python Path
You may need to adjust the system Python path in `ros_wrapper.py`:

**Linux:**
```python
self.system_python = "/usr/bin/python3"
```

**macOS:**
```python
self.system_python = "/usr/bin/python3"
```

### Testing System Python
The wrapper automatically tests if the system Python can import ROS:
```bash
python ros_wrapper.py
```

You can also manually test the system Python:
```bash
/usr/bin/python3 -c "import rospy; print('ROS import successful')"
```

## How It Works

1. **Wrapper Script** (`ros_wrapper.py`):
   - Uses `subprocess.run()` to execute the ROS script with system Python
   - Captures output and handles errors
   - Reads results from a temporary JSON file
   - Provides both full object detection and simple scene capture methods

2. **Standalone ROS Script** (`rospygemini1_standalone.py`):
   - Modified version of your original script
   - Saves results to `/tmp/ros_detection_result.json`
   - Better error handling and timeout management
   - Uses temporary files for image storage

3. **Vision Integration** (`vision_sceneunderstanding.py`):
   - Updated to use the ROS wrapper instead of direct imports
   - No longer imports `Object3DDetector` directly
   - Uses `run_scene_capture()` method for image capture
   - Maintains the same interface for the rest of your code

4. **Communication**:
   - ROS script communicates with your FastAPI service via HTTP
   - Results are saved to a JSON file and read by the wrapper
   - No direct Python module imports between environments

## File Structure
```
googleResoning/
├── rosbridge/
│   ├── ros_wrapper.py              # Main wrapper script
│   ├── rospygemini1_standalone.py  # Standalone ROS script
│   ├── rospygemini1.py             # Original ROS script
│   ├── example_usage.py            # Usage example
│   ├── gemini_service.py           # Your FastAPI service
│   └── README_ROS_Wrapper.md       # This file
├── src/
│   └── language_models/
│       └── vision_sceneunderstanding.py  # Updated to use wrapper
└── test_vision_integration.py      # Test script
```

## Requirements

### System Environment (where ROS is installed)
- ROS (Robot Operating System)
- Python packages: `rospy`, `cv_bridge`, `tf2_ros`, `numpy`, `opencv-python`
- ROS topics: `/camera/color/image_raw`, `/camera/depth/image_rect_raw`, `/camera/color/camera_info`

### Current Environment
- Python packages: `requests`, `json`, `subprocess`, `pathlib`, `google-genai`, `PIL`
- FastAPI service running on `http://localhost:5000`

## Troubleshooting

### Common Issues

1. **System Python not found**:
   - Update the `system_python` path in `ros_wrapper.py`
   - Test with: `which python3` (Linux/macOS)

2. **ROS import fails**:
   - Ensure ROS is installed in the system Python environment
   - Source ROS setup: `source /opt/ros/<version>/setup.bash`
   - Check ROS installation: `rosversion -d`

3. **Timeout errors**:
   - Increase timeout in `ros_wrapper.py` (currently 60 seconds)
   - Check if camera topics are publishing: `rostopic list | grep camera`

4. **TF transform errors**:
   - Ensure TF is running: `rosrun tf2_ros tf2_echo`
   - Check available frames: `rosrun tf2_tools view_frames`
   - View TF tree: `rosrun tf2_tools view_frames.py`

5. **Vision module import errors**:
   - Run `python test_vision_integration.py` to test the integration
   - Check that the path to rosbridge is correct in `vision_sceneunderstanding.py`
   - Verify file permissions: `ls -la rosbridge/`

### Debug Mode
To see more detailed output, modify the subprocess call in `ros_wrapper.py`:
```python
result = subprocess.run(
    cmd,
    capture_output=True,
    text=True,
    timeout=60,
    env=os.environ.copy()  # Pass current environment variables
)
```

## Integration with Your FastAPI Service

The ROS script communicates with your FastAPI service via HTTP requests:
```python
response = requests.post(
    "http://localhost:5000/detect_object",
    json={"image_path": image_path},
    timeout=30
)
```

Make sure your FastAPI service is running before executing the ROS wrapper.

## Testing

### Test Vision Integration
```bash
python test_vision_integration.py
```

This will test:
- Import of the vision module
- Initialization of the VLM class
- Existence of the capture_scene method

### Test ROS Wrapper
```bash
python ros_wrapper.py
```

This will test:
- System Python ROS compatibility
- Full object detection pipeline
- Result parsing and output

### Test System Python ROS Compatibility
```bash
/usr/bin/python3 -c "import rospy; print('ROS import successful')"
```

### Test Scene Capture Only
```python
from ros_wrapper import ROSWrapper
wrapper = ROSWrapper()
success = wrapper.run_scene_capture()
```

This will test just the image capture functionality used by the vision module.

### Check ROS Topics
```bash
rostopic list | grep camera
```

### Check ROS Installation
```bash
rosversion -d
source /opt/ros/noetic/setup.bash  # or your ROS version
``` 
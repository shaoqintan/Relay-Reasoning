from google import genai
from google.genai import types

import PIL.Image
import os
from dotenv import load_dotenv

import json
from datetime import datetime
import sys
from pathlib import Path

# Add the rosbridge directory to the path so we can import the wrapper
rosbridge_path = Path(__file__).parent.parent.parent / "rosbridge"
sys.path.append(str(rosbridge_path))

from ros_wrapper import ROSWrapper

# Load the environment
load_dotenv()  # take environment variables
GEMINI_API_KEY = os.getenv('google_api_key')


class VLM:
    def __init__(self):
        self.client = genai.Client(api_key=GEMINI_API_KEY)
        self.memory_directory = "/home/er/Documents/reasoning320/googleResoning/memory"
        self.SCENE_DB = "scene_descriptions.jsonl"
        
        # Initialize the ROS wrapper instead of direct Object3DDetector
        self.ros_wrapper = ROSWrapper()

    def capture_scene(self):
        """
        Capture scene using the ROS wrapper instead of direct Object3DDetector
        """
        try:
            # Use the ROS wrapper to capture the scene
            # This will run the ROS code using system Python
            print("Capturing scene using ROS wrapper...")
            
            # Run the scene capture using the wrapper
            result = self.ros_wrapper.run_scene_capture()
            
            if result:
                print("Scene capture completed successfully")
                return True
            else:
                print("Scene capture failed")
                return False
                
        except Exception as e:
            print(f"Error during scene capture: {str(e)}")
            return False

    def describe_and_save(self, user_query):
        # Use the wrapper to capture scene instead of direct Object3DDetector
        capture_success = self.capture_scene()
        
        if not capture_success:
            print("Warning: Scene capture failed, but continuing with description...")
        
        # Try to open the captured image
        image_path = '/home/er/Documents/reasoning320/googleResoning/rosbridge/current_frame.png'
        
        # Check if the image exists
        if not os.path.exists(image_path):
            print(f"Warning: Image not found at {image_path}")
            # You might want to use a default image or handle this case
            return "Scene capture failed - no image available for analysis"
        
        try:
            image = PIL.Image.open(image_path)
            print("Image loaded successfully")
        except Exception as e:
            print(f"Error loading image: {str(e)}")
            return f"Error loading captured image: {str(e)}"

        template = """
            You are a highly skilled visual inspection agent embedded within a robotic system.

            Your core expertise lies in:
            - Identifying the following objects: drawstring pouch, plastic cube, titanium blade, squishy toy car.
            - For each object you spot, analyze its material, softness, surface texture, deformability, and fragility.
            - Recommend a suitable grip strength classification (low, moderate, high, very high) and a numeric value (0–100) for the robotic gripper, based on your analysis.
            - Justify your recommendation for each object.

            ==============================
            Primary Objective
            ==============================
            Process the given observation input (from the robot's camera) and generate a structured yet conversational description of the scene. 
            For each of the four target objects detected, provide:
                - The most suitable grip strength classification (low, moderate, high, very high)
                - The numeric grip strength value (0–100)
                - A brief justification for your choice (e.g., "The drawstring pouch is soft fabric, so a low grip strength of 10 is appropriate.")
            If none of the four objects are present, state that none were found.

            ==============================
            Behavior & Capabilities
            ==============================
            1. Object Identification (Highest Priority)
               - Examine the scene for the four target objects.
               - For each detected object, analyze its properties and recommend grip strength.
            2. Scene Description
               - Describe all visible objects with clarity, focusing on the four target objects.
            3. Output
               - For each object: report the classification, value, and justification.
               - If no object is found, state so.

            ==============================
            Example Dialogue
            ==============================
            Input:
            Observation: "A plastic cube and a drawstring pouch are on the table. The cube is hard and smooth, the pouch is soft fabric."
            
            VLM Output:
            "I see a plastic cube and a drawstring pouch on the table. The cube is hard and smooth, so a moderate grip strength of 40 is suitable. The pouch is soft fabric, so a low grip strength of 10 is appropriate."

            ==============================
            Design Notes
            ==============================
            - Tone: Professional but conversational. Speak naturally as if explaining to a technician or teammate.
            - Focus: Prioritize the four target objects over background or non-critical elements.
            - Dialogue: Maintain flow and engagement by summarizing clearly and naturally.
            - Limitations: Do not infer actions or intent—only describe what is directly visible.
            - Background: De-emphasize background details unless they're directly relevant to the grip task.
        """.strip()

        # print(template)

        response = self.client.models.generate_content(
            model="gemini-2.5-pro-preview-05-06",
            contents=[template, image])

        # print(response.text)

        scene_desc = response.text.strip()

        entry = {
            "timestamp": datetime.utcnow().isoformat(),
            "observation": user_query,
            "description": scene_desc,
        }

        # Construct the full path
        file_name = os.path.join(self.memory_directory, self.SCENE_DB)

        with open(file_name, "a") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")

        print(scene_desc)
        return scene_desc


# if __name__ == '__main__':
#     ob = VLM()
#     ob.describe_and_save(user_query="What do you see")

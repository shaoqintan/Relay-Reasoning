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
            - Identifying surface-level defects in electrical components—especially relays—with precision and clarity.
            - Interpreting the surrounding environment to provide situational awareness.
            - Communicating findings in natural, conversational language suitable for dialogue with human operators.
            
            You are optimized for defect detection tasks such as bent relay pins and visible surface damage. In addition, you contribute to overall scene understanding to support downstream robotic planning or operator decision-making. Your responses should be structured, informative, and lightly conversational to maintain fluid interaction.
            
            ==============================
            Primary Objective
            ==============================
            Process the given observation input (from the robot's camera) and generate a structured yet conversational description of the scene. 
            Prioritize identifying **defects in relays**, especially bent pins or surface-level body damage, while also describing environmental context when relevant.
            
            ==============================
            Behavior & Capabilities
            ==============================
            
            1. Relay Defect Detection (Highest Priority)
               - Examine each visible relay for external, surface-level defects.
               - Focus on:
                 - **Bent pins**
                 - **Surface cracks or body damage**
               - Clearly identify and tag defective relays along with the nature and location of the defect.
               - Example:
                 - "Relay 3 has a visibly bent pin on the lower-right side."
                 - "Relay 1 shows a surface crack on the top panel."
            
            2. Scene Description
               - Describe all visible objects with clarity.
               - Include:
                 - Object types and approximate layout
                 - Relative positions (left, right, center)
                 - Orientations and spatial relationships (e.g., "next to", "under", "on top of")
               - Avoid guessing uncertain measurements or making assumptions.
               - Example:
                 - "Three relays are arranged in a line. A screwdriver is placed to the left of Relay 2."
            
            3. Spatial Awareness
               - Summarize general object arrangement and layout.
               - Focus on how relays and nearby items are positioned.
               - Avoid numerical estimates unless clearly visible.
               - Example:
                 - "The relays appear clustered toward the right side of the workspace."
            
            4. Contextual Awareness
               - Note any unusual features or obstacles that may affect manipulation or task execution.
               - Example:
                 - "A stray wire is lying close to Relay 4 and may obstruct the gripper."
            
            5. Change Detection (if previous scene is available)
               - Compare the current observation with the past state.
               - Highlight changes relevant to safety, positioning, or damage.
               - Example:
                 - "Relay 2 has been moved from the center to the front-left corner of the table."
            
            ==============================
            Output Format
            ==============================
            Provide a natural language description with structured, conversational observations.
            
            Your output should include:
            - Relay defect identification (tagged, with description)
            - General scene summary (visible objects, layout)
            - Environmental context (only if task-relevant)
            - A tone that is professional, informative, and slightly conversational
            
            At the end of the response, include a concise "Defect Summary" section listing:
            - Relay index or position
            - Location of defect (e.g., pin side or body surface)
            This helps downstream systems or human operators quickly act on the issues found.

            
            ==============================
            Example Dialogue
            ==============================
            
            Input:
            Observation: "Four relays on a workbench. Relay 1 has a bent pin. Relay 3 has a crack on its surface. A yellow screwdriver lies on the left."
            
            VLM Output:
            "I see four relays placed on a white workbench. Relay 1, on the top-left, has a pin slightly bent outward on the right side. Relay 3 has a visible crack on its top surface. A yellow screwdriver is resting on the far left side. The rest of the relays appear intact."
            
            ==============================
            Design Notes
            ==============================
            - Tone: Professional but conversational. Speak naturally as if explaining to a technician or teammate.
            - Focus: Prioritize relay defects over background or non-critical elements.
            - Dialogue: Maintain flow and engagement by summarizing clearly and naturally.
            - Limitations: Do not infer actions or intent—only describe what is directly visible.
            - Background: De-emphasize background details unless they're directly relevant to defects or tasks.
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

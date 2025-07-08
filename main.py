from src.audio_transcription.asr import ASR
from src.audio_transcription.tts import TTS
from src.audio_transcription.fast_tts import FastTTS 
from src.routing.router import Router
from src.language_models.vision_sceneunderstanding import VLM
from src.language_models.llm_user_action_response import UAResponse
from src.language_models.util import Utility
import subprocess
# from rosbridge.rospygemini1 import Object3DDetector

'''
 Pipeline

 1. ASR -> User Query Transcribe
 2. Router -> Check if this is a perception query or a router query

If perception query:
    3.1. VLM for scene understanding
    3.2. Store the scene description


 4. Retrieve the store scene description
 5. LLM -> For Planning and execution

 6. TTS -> Voice output of the scene description or the action response.
'''

import json

# Creating Object
asr_ob = ASR()
# tts_ob = TTS()
tts_ob = FastTTS()  # Use FastTTS instead
router_ob = Router()
vlm_ob = VLM()
action_ob = UAResponse()
ut_ob = Utility()
# ros_obj = Object3DDetector()

def audio_transcription():
    # Create Object
    transcribed_text = asr_ob.transcribe_audio()
    return transcribed_text


def routing(user_query):
    identified_route = router_ob.identify_route(user_query=user_query)
    return identified_route


def scene_description(user_query):
    # Describe & persist
    vlm_ob.describe_and_save(user_query=user_query)  # saves the scene description in memory/jsonl file
    # Todo: Add voice output that describes the scene -> use the jsonl file
    return

def scene_description_to_user(scene_desc):
    response = ut_ob.vlm_user_response(scene_desc)
    return response


def load_latest_scene() -> str:
    SCENE_DB = "/Users/shubhamrathod/PycharmProjects/googleResoning/memory/scene_descriptions.jsonl"
    try:
        last_line = None
        with open(SCENE_DB, "r") as f:
            for line in f:
                if line.strip():
                    last_line = line
        if not last_line:
            return ""
        entry = json.loads(last_line)
        return entry.get("description", "")
    except FileNotFoundError:
        # No file yet
        return ""
    except json.JSONDecodeError as e:
        print(f"JSON parse error: {e}")
        return ""


def llm_reasoning(user_query, scene_desc):
    conversational = action_ob.response_and_action(user_query=user_query, scene_desc=scene_desc)
    return conversational


def process_query(transcribed_text):
    # step 2: Routing
    identified_route = routing(transcribed_text)
    print("Route: ", identified_route)
    print("========================")

    # Step 3: Check if this is a perception query
    if identified_route == "vision":
        # Describe & persist
        scene_description(transcribed_text)
        scene_desc = load_latest_scene()
        vlm_user_response = scene_description_to_user(scene_desc)
        print("scene_desc: ", vlm_user_response)
        print("========================")
        # Speak the scene description
        tts_ob.speak_text(vlm_user_response)
    else:
        # Action query: retrieve the latest known scene
        scene_desc = load_latest_scene()
        conversational = llm_reasoning(user_query=transcribed_text, scene_desc=scene_desc)
        print("Action: ", conversational)
        print("========================")
        # Speak the action response
        tts_ob.speak_text(conversational)
        ros_command = [
        "/bin/bash", "-c",
        "source /opt/ros/noetic/setup.bash && "
        "source ~/catkin_ws/devel/setup.bash && "
        "/usr/bin/python3 /home/er/Documents/rostransform/rosbridge/rospygemini1_mod.py"
        ]
        subprocess.run(ros_command)
        # ros_obj.capture_and_process_frame()

def pipeline():
    print("Starting conversation... ")
    tts_ob.speak_text("Hello, I am Jetty the robot! I'm ready to help. What would you like me to do?")
    
    while True:
        # step 1: Perform ASR
        transcribed_text = audio_transcription()
        print("ASR: ", transcribed_text)
        print("========================")
        
        # For multiple exit phrases
        exit_phrases = ["thank you", "goodbye", "bye", "exit", "quit"]
        if any(phrase in transcribed_text.lower() for phrase in exit_phrases):
            print("Ending conversation...")
            tts_ob.speak_text("Thank you for chatting with me. Goodbye!")
            break
            
        # Process the query
        process_query(transcribed_text)
        print("hello we have action")
        tts_ob.speak_text("What else can I help you with?")
    


if __name__ == '__main__':
    pipeline()
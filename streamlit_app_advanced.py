import streamlit as st
import os
import json
import time
import threading
from datetime import datetime
import subprocess
from PIL import Image
import speech_recognition as sr
import pyttsx3
from google import genai
from dotenv import load_dotenv

# Import your existing components
import sys
sys.path.append('.')
from src.audio_transcription.asr import ASR
from src.audio_transcription.fast_tts import FastTTS
from src.routing.router import Router
from src.language_models.vision_sceneunderstanding import VLM
from src.language_models.llm_user_action_response import UAResponse
from src.language_models.util import Utility

# Load environment variables
load_dotenv()
google_api_key = os.getenv('google_api_key')

# Initialize session state
if 'messages' not in st.session_state:
    st.session_state.messages = []
if 'is_listening' not in st.session_state:
    st.session_state.is_listening = False
if 'mic_enabled' not in st.session_state:
    st.session_state.mic_enabled = True
if 'system_initialized' not in st.session_state:
    st.session_state.system_initialized = False

# Paths
IMAGE_PATH = "/home/er/Documents/reasoning320/googleResoning/rosbridge/current_frame.png"
AUDIO_FILE = "/home/er/Documents/reasoning320/googleResoning/audiofile/ip.wav"
SCENE_DB = "/home/er/Documents/reasoning320/googleResoning/memory/scene_descriptions.jsonl"

# Initialize system components
def initialize_system():
    if not st.session_state.system_initialized:
        try:
            st.session_state.asr_ob = ASR()
            st.session_state.tts_ob = FastTTS()
            st.session_state.router_ob = Router()
            st.session_state.vlm_ob = VLM()
            st.session_state.action_ob = UAResponse()
            st.session_state.ut_ob = Utility()
            st.session_state.system_initialized = True
            return True
        except Exception as e:
            st.error(f"Error initializing system: {str(e)}")
            return False
    return True

def load_latest_scene():
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
        return ""
    except json.JSONDecodeError as e:
        st.error(f"JSON parse error: {e}")
        return ""

def capture_image():
    """Capture image from ROS camera"""
    try:
        # Run the image saver script
        subprocess.run([
            "/bin/bash", "-c",
            "source /opt/ros/noetic/setup.bash && "
            "source ~/catkin_ws/devel/setup.bash && "
            "python3 /home/er/Documents/reasoning320/googleResoning/rosbridge/image_saver.py"
        ], timeout=10)
        return True
    except Exception as e:
        st.error(f"Error capturing image: {str(e)}")
        return False

def process_user_input(user_input):
    """Process user input using the existing system components"""
    if not initialize_system():
        return "System initialization failed. Please check the console for errors."
    
    try:
        # Step 1: Routing
        identified_route = st.session_state.router_ob.identify_route(user_query=user_input)
        
        # Step 2: Check if this is a perception query
        if identified_route == "vision":
            # Capture image and describe scene
            if capture_image():
                # Describe & persist
                st.session_state.vlm_ob.describe_and_save(user_query=user_input)
                scene_desc = load_latest_scene()
                if scene_desc:
                    vlm_user_response = st.session_state.ut_ob.vlm_user_response(scene_desc)
                    return vlm_user_response
                else:
                    return "I captured an image but couldn't describe the scene yet."
            else:
                return "Sorry, I couldn't capture an image right now."
        else:
            # Action query: retrieve the latest known scene
            scene_desc = load_latest_scene()
            conversational = st.session_state.action_ob.response_and_action(
                user_query=user_input, 
                scene_desc=scene_desc
            )
            return conversational
            
    except Exception as e:
        st.error(f"Error processing input: {str(e)}")
        return f"I heard you say: '{user_input}'. How can I help you with that?"

def transcribe_audio_streamlit():
    """Streamlit-compatible audio transcription"""
    try:
        r = sr.Recognizer()
        with sr.Microphone() as source:
            st.info("Listening... Please say something...")
            audio = r.listen(source)

        with open(AUDIO_FILE, "wb") as f:
            f.write(audio.get_wav_data())

        client = genai.Client(api_key=google_api_key)
        myfile = client.files.upload(file=AUDIO_FILE)
        
        response = client.models.generate_content(
            model="gemini-2.5-pro-preview-05-06",
            contents=["Transcribe this audio clip", myfile]
        )
        
        return response.text
    except Exception as e:
        st.error(f"Error in transcription: {str(e)}")
        return None

def speak_text_streamlit(text):
    """Streamlit-compatible text-to-speech"""
    try:
        engine = pyttsx3.init()
        engine.setProperty('rate', 175)
        engine.setProperty('volume', 1.0)
        engine.setProperty('pitch', 50)
        engine.say(text)
        engine.runAndWait()
        return True
    except Exception as e:
        st.error(f"Error in TTS: {str(e)}")
        return False

def main():
    st.set_page_config(
        page_title="Robot Assistant",
        page_icon="🤖",
        layout="wide"
    )
    
    st.title("🤖 Robot Assistant")
    
    # Initialize system
    if not initialize_system():
        st.error("Failed to initialize system components. Please check your environment setup.")
        return
    
    # Create two columns
    col1, col2 = st.columns([1, 1])
    
    with col1:
        st.subheader("📸 Current View")
        
        # Display the current image
        if os.path.exists(IMAGE_PATH):
            try:
                image = Image.open(IMAGE_PATH)
                st.image(image, caption="Current camera view", use_column_width=True)
            except Exception as e:
                st.error(f"Error loading image: {str(e)}")
                st.info("No image available")
        else:
            st.info("No image available")
        
        # Refresh button for image
        if st.button("🔄 Refresh Image"):
            if capture_image():
                st.success("Image captured!")
                st.rerun()
            else:
                st.error("Failed to capture image")
        
        # System status
        st.markdown("---")
        st.subheader("🔧 System Status")
        
        # Check if scene database exists
        scene_exists = os.path.exists(SCENE_DB)
        st.write(f"Scene Database: {'✅ Available' if scene_exists else '❌ Not found'}")
        
        # Check if image exists
        image_exists = os.path.exists(IMAGE_PATH)
        st.write(f"Current Image: {'✅ Available' if image_exists else '❌ Not found'}")
        
        # System components status
        st.write(f"ASR: {'✅ Ready' if st.session_state.system_initialized else '❌ Not ready'}")
        st.write(f"TTS: {'✅ Ready' if st.session_state.system_initialized else '❌ Not ready'}")
        st.write(f"Router: {'✅ Ready' if st.session_state.system_initialized else '❌ Not ready'}")
        st.write(f"VLM: {'✅ Ready' if st.session_state.system_initialized else '❌ Not ready'}")
    
    with col2:
        st.subheader("💬 Chat Interface")
        
        # Chat messages container
        chat_container = st.container()
        
        with chat_container:
            # Display chat messages
            for message in st.session_state.messages:
                with st.chat_message(message["role"]):
                    st.markdown(message["content"])
        
        # Input area
        st.markdown("---")
        
        # Text input
        user_input = st.text_input("Type your message:", key="text_input")
        
        # Voice input
        col3, col4 = st.columns([1, 1])
        
        with col3:
            if st.button("🎤 Voice Input", disabled=not st.session_state.mic_enabled):
                st.session_state.is_listening = True
                transcribed_text = transcribe_audio_streamlit()
                st.session_state.is_listening = False
                
                if transcribed_text:
                    # Add user message
                    st.session_state.messages.append({
                        "role": "user",
                        "content": transcribed_text,
                        "timestamp": datetime.now().isoformat()
                    })
                    
                    # Process and get robot response
                    robot_response = process_user_input(transcribed_text)
                    
                    # Add robot message
                    st.session_state.messages.append({
                        "role": "assistant",
                        "content": robot_response,
                        "timestamp": datetime.now().isoformat()
                    })
                    
                    # Speak the response
                    speak_text_streamlit(robot_response)
                    
                    st.rerun()
        
        with col4:
            if st.button("🔇 Toggle Mic", key="toggle_mic"):
                st.session_state.mic_enabled = not st.session_state.mic_enabled
                st.rerun()
            
            mic_status = "🟢 Enabled" if st.session_state.mic_enabled else "🔴 Disabled"
            st.write(f"Microphone: {mic_status}")
        
        # Handle text input
        if user_input and st.button("Send", key="send_text"):
            # Add user message
            st.session_state.messages.append({
                "role": "user",
                "content": user_input,
                "timestamp": datetime.now().isoformat()
            })
            
            # Process and get robot response
            robot_response = process_user_input(user_input)
            
            # Add robot message
            st.session_state.messages.append({
                "role": "assistant",
                "content": robot_response,
                "timestamp": datetime.now().isoformat()
            })
            
            # Speak the response
            speak_text_streamlit(robot_response)
            
            st.rerun()
        
        # Clear chat button
        if st.button("🗑️ Clear Chat"):
            st.session_state.messages = []
            st.rerun()
        
        # Test system button
        if st.button("🧪 Test System"):
            test_response = "Hello! I am Jetty the robot! I'm ready to help. What would you like me to do?"
            st.session_state.messages.append({
                "role": "assistant",
                "content": test_response,
                "timestamp": datetime.now().isoformat()
            })
            speak_text_streamlit(test_response)
            st.rerun()
    
    # Status bar
    st.markdown("---")
    col5, col6, col7 = st.columns([1, 1, 1])
    
    with col5:
        st.write(f"**Messages:** {len(st.session_state.messages)}")
    
    with col6:
        if st.session_state.is_listening:
            st.write("🎤 **Listening...**")
        else:
            st.write("🔇 **Not listening**")
    
    with col7:
        st.write(f"**Mic Status:** {mic_status}")

if __name__ == "__main__":
    main() 
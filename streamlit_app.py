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
try:
    from google import genai
except ImportError:
    import genai
from dotenv import load_dotenv

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

# Paths
IMAGE_PATH = "/home/er/Documents/reasoning320/googleReasoning/rosbridge/current_frame.png"
AUDIO_FILE = "/home/er/Documents/reasoning320/googleReasoning/audiofile/ip.wav"
SCENE_DB = "/home/er/Documents/reasoning320/googleReasoning/memory/scene_descriptions.jsonl"

class StreamlitASR:
    def __init__(self):
        self.r = sr.Recognizer()
        self.audio_file = AUDIO_FILE
        
    def transcribe_audio(self):
        try:
            with sr.Microphone() as source:
                st.info("Listening... Please say something...")
                audio = self.r.listen(source)

            with open(self.audio_file, "wb") as f:
                f.write(audio.get_wav_data())

            client = genai.Client(api_key=google_api_key)
            myfile = client.files.upload(file=self.audio_file)
            
            response = client.models.generate_content(
                model="gemini-2.5-pro-preview-05-06",
                contents=["Transcribe this audio clip", myfile]
            )
            
            return response.text
        except Exception as e:
            st.error(f"Error in transcription: {str(e)}")
            return None

class StreamlitTTS:
    def __init__(self):
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 175)
        self.engine.setProperty('volume', 1.0)
        self.engine.setProperty('pitch', 50)
        
    def speak_text(self, text):
        try:
            self.engine.say(text)
            self.engine.runAndWait()
            return True
        except Exception as e:
            st.error(f"Error in TTS: {str(e)}")
            return False

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
            "python3 /home/er/Documents/reasoning320/googleReasoning/rosbridge/image_saver.py"
        ], timeout=10)
        return True
    except Exception as e:
        st.error(f"Error capturing image: {str(e)}")
        return False

def process_user_input(user_input):
    """Process user input and generate robot response"""
    # This is a simplified version - you can integrate with your existing routing and LLM logic
    if "vision" in user_input.lower() or "see" in user_input.lower() or "what" in user_input.lower():
        # Capture image and describe scene
        if capture_image():
            scene_desc = load_latest_scene()
            if scene_desc:
                return f"I can see: {scene_desc}"
            else:
                return "I captured an image but couldn't describe the scene yet."
        else:
            return "Sorry, I couldn't capture an image right now."
    else:
        # Simple response for other queries
        return f"I heard you say: '{user_input}'. How can I help you with that?"

def main():
    st.set_page_config(
        page_title="Robot Assistant",
        page_icon="🤖",
        layout="wide"
    )
    
    st.title("🤖 Robot Assistant")
    
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
                asr = StreamlitASR()
                transcribed_text = asr.transcribe_audio()
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
                    tts = StreamlitTTS()
                    tts.speak_text(robot_response)
                    
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
            tts = StreamlitTTS()
            tts.speak_text(robot_response)
            
            st.rerun()
        
        # Clear chat button
        if st.button("🗑️ Clear Chat"):
            st.session_state.messages = []
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
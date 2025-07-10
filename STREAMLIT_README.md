# Robot Assistant Streamlit UI

This Streamlit application provides a web-based interface for the Robot Assistant system with real-time image display, chat functionality, and voice interaction.

## Features

### 🖼️ Image Display
- **Left Panel**: Shows the current camera view from the ROS camera
- **Refresh Button**: Manually capture a new image from the camera
- **Real-time Updates**: Displays the latest captured image

### 💬 Chat Interface
- **Right Panel**: Chat interface with message bubbles
- **User Messages**: Displayed on the right side (blue bubbles)
- **Robot Messages**: Displayed on the left side (gray bubbles)
- **Text Input**: Type messages directly
- **Voice Input**: Use microphone for voice commands

### 🎤 Voice Controls
- **Voice Input Button**: Click to start voice recording
- **Toggle Mic Button**: Enable/disable microphone
- **Status Indicators**: Shows microphone status and listening state

### 🔧 System Integration
- **Full Pipeline Integration**: Uses your existing ASR, TTS, Router, VLM, and LLM components
- **Scene Understanding**: Automatically captures and describes scenes
- **Action Processing**: Handles both vision and action queries
- **System Status**: Shows component availability and system health

## Installation

1. **Install Streamlit** (if not already installed):
```bash
pip install streamlit
```

2. **Update requirements** (already done):
```bash
pip install -r requirements.txt
```

## Running the Application

### Basic Version
```bash
streamlit run streamlit_app.py
```

### Advanced Version (Full Integration)
```bash
streamlit run streamlit_app_advanced.py
```

## Usage

1. **Start the application** using one of the commands above
2. **Wait for system initialization** - the app will show component status
3. **Use the interface**:
   - Type messages in the text input
   - Click "🎤 Voice Input" for voice commands
   - Click "🔄 Refresh Image" to capture a new image
   - Use "🔇 Toggle Mic" to enable/disable microphone

## System Requirements

- **Linux System** (as specified in your paths)
- **ROS Noetic** installed and configured
- **Python dependencies** from requirements.txt
- **Google API Key** in .env file
- **Microphone access** for voice input
- **Audio output** for text-to-speech

## File Structure

```
├── streamlit_app.py              # Basic Streamlit UI
├── streamlit_app_advanced.py     # Full system integration
├── requirements.txt              # Updated with Streamlit
└── STREAMLIT_README.md          # This file
```

## Paths Used

The application uses these Linux paths (as specified):
- **Image Path**: `/home/er/Documents/reasoning320/googleResoning/rosbridge/current_frame.png`
- **Audio File**: `/home/er/Documents/reasoning320/googleResoning/audiofile/ip.wav`
- **Scene Database**: `/home/er/Documents/reasoning320/googleResoning/memory/scene_descriptions.jsonl`

## Troubleshooting

### Common Issues

1. **System not initializing**:
   - Check that all dependencies are installed
   - Verify your .env file has the Google API key
   - Ensure ROS is properly configured

2. **Image not displaying**:
   - Check if the image file exists at the specified path
   - Verify ROS camera is running
   - Try the "Refresh Image" button

3. **Voice input not working**:
   - Check microphone permissions
   - Verify PyAudio is installed correctly
   - Try toggling the microphone button

4. **TTS not working**:
   - Check audio output settings
   - Verify pyttsx3 installation
   - Check system audio drivers

### Debug Mode

To run with debug information:
```bash
streamlit run streamlit_app_advanced.py --logger.level debug
```

## Integration with Existing System

The advanced version (`streamlit_app_advanced.py`) integrates with your existing components:

- **ASR**: Uses your existing ASR class for audio transcription
- **TTS**: Uses your existing FastTTS class for speech synthesis
- **Router**: Uses your existing Router for query classification
- **VLM**: Uses your existing VLM for scene understanding
- **LLM**: Uses your existing UAResponse for action processing

This ensures the Streamlit UI works seamlessly with your existing robot assistant pipeline.

## Customization

You can customize the UI by modifying:
- **Colors and styling**: Edit the CSS in the Streamlit app
- **Layout**: Adjust the column ratios and component placement
- **Features**: Add new buttons or functionality as needed
- **Integration**: Modify how it integrates with your existing components 
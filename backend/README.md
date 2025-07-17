# FastAPI Backend for Robot Assistant

## Features
- Audio transcription (ASR) using Google Gemini
- Text-to-speech (TTS) with pyttsx3
- Image capture via ROS
- Scene understanding and routing with Gemini and LangChain
- Action response planning

## Setup

### 1. Install dependencies
```bash
cd backend
pip install -r requirements.txt
```

### 2. Environment Variables
Create a `.env` file in the backend directory with your Google API key:
```
google_api_key=YOUR_GOOGLE_API_KEY
```

### 3. ROS Setup (for image capture)
- Make sure ROS is installed and running.
- The ROS node expects a camera publishing to `/camera/color/image_raw`.
- `cv_bridge` and `rospy` must be installed in your Python environment.

### 4. Run the backend
```bash
uvicorn main:app --reload
```

The API will be available at `http://localhost:8000/`.

## Endpoints
- `POST /asr/` — Audio transcription (upload WAV file)
- `POST /tts/` — Text-to-speech (returns WAV audio)
- `POST /capture_image/` — Capture image from ROS camera
- `GET /scene_description/` — Get latest scene description
- `POST /route/` — Route user query
- `POST /action_response/` — Get action response
- `GET /image/` — Get latest image

## Troubleshooting
- If you get errors about missing ROS or cv_bridge, ensure your ROS environment is sourced and dependencies are installed.
- For Google Gemini, ensure your API key is valid and you have internet access.
- For TTS, pyttsx3 may require additional system voices or drivers.

## Notes
- The backend expects certain file paths (see main.py and modules). Adjust as needed for your environment.
- For full functionality, ensure all Python modules from the original project are accessible to the backend. 
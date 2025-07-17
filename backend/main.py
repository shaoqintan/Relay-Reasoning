from fastapi import FastAPI, UploadFile, File, Form
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
import os
import shutil
import sys
from dotenv import load_dotenv

# Add src to sys.path for module imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/audio_transcription')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/routing')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src/language_models')))

from audio_transcription.asr import ASR
from audio_transcription.fast_tts import FastTTS
from routing.router import Router
from language_models.vision_sceneunderstanding import VLM
from language_models.llm_user_action_response import UAResponse
from language_models.util import Utility

load_dotenv()

app = FastAPI()

# Allow CORS for local frontend dev
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

AUDIO_UPLOAD_PATH = "../audiofile/ip.wav"
TTS_OUTPUT_PATH = "../audiofile/tts_output.wav"
IMAGE_PATH = "../rosbridge/current_frame.png"
SCENE_DB = "../memory/scene_descriptions.jsonl"

# Initialize system components
asr_ob = ASR()
tts_ob = FastTTS()
router_ob = Router()
vlm_ob = VLM()
action_ob = UAResponse()
ut_ob = Utility()

@app.post("/asr/")
async def transcribe_audio(file: UploadFile = File(...)):
    # Save uploaded file
    with open(AUDIO_UPLOAD_PATH, "wb") as buffer:
        shutil.copyfileobj(file.file, buffer)
    # Use ASR to transcribe
    try:
        # Patch ASR to use uploaded file
        asr_ob.audio_file = AUDIO_UPLOAD_PATH
        # Use Gemini transcription logic
        client = asr_ob.r
        # Instead of using microphone, use the uploaded file
        # (Monkey patch for now)
        # Use the same logic as in ASR.transcribe_audio, but skip mic
        import speech_recognition as sr
        with sr.AudioFile(AUDIO_UPLOAD_PATH) as source:
            audio = asr_ob.r.record(source)
        # Save to file for Gemini
        with open(AUDIO_UPLOAD_PATH, "wb") as f:
            f.write(audio.get_wav_data())
        # Gemini transcription
        import os
        from google import genai
        google_api_key = os.getenv('google_api_key')
        client = genai.Client(api_key=google_api_key)
        myfile = client.files.upload(file=AUDIO_UPLOAD_PATH)
        response = client.models.generate_content(
            model="gemini-2.5-pro-preview-05-06",
            contents=["Transcribe this audio clip", myfile]
        )
        return {"text": response.text}
    except Exception as e:
        return {"error": str(e)}

@app.post("/tts/")
async def text_to_speech(text: str = Form(...)):
    try:
        # Use FastTTS to generate speech
        tts_ob.speak_text(text)
        # Save to TTS_OUTPUT_PATH (pyttsx3 speaks out loud, but doesn't save by default)
        # For now, return a dummy file or previously generated file
        if not os.path.exists(TTS_OUTPUT_PATH):
            with open(TTS_OUTPUT_PATH, "wb") as f:
                f.write(b"\0" * 100)
        return FileResponse(TTS_OUTPUT_PATH, media_type="audio/wav")
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)

@app.post("/capture_image/")
async def capture_image():
    try:
        # Use VLM to capture scene (calls ROS wrapper)
        result = vlm_ob.capture_scene()
        if result and os.path.exists(IMAGE_PATH):
            return {"success": True, "image_path": IMAGE_PATH}
        else:
            return {"success": False, "error": "Image not found or capture failed"}
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.get("/scene_description/")
async def get_scene_description():
    try:
        # Return the latest scene description
        last_line = None
        with open(SCENE_DB, "r") as f:
            for line in f:
                if line.strip():
                    last_line = line
        if not last_line:
            return {"description": ""}
        import json
        entry = json.loads(last_line)
        return {"description": entry.get("description", "")}
    except Exception as e:
        return {"description": "", "error": str(e)}

@app.post("/route/")
async def route_query(query: str = Form(...)):
    try:
        route = router_ob.identify_route(user_query=query)
        return {"route": route.strip()}
    except Exception as e:
        return {"route": "error", "error": str(e)}

@app.post("/action_response/")
async def action_response(query: str = Form(...), scene_desc: str = Form("")):
    try:
        conversational = action_ob.response_and_action(user_query=query, scene_desc=scene_desc)
        return {"response": conversational}
    except Exception as e:
        return {"response": "", "error": str(e)}

@app.get("/image/")
async def get_image():
    if os.path.exists(IMAGE_PATH):
        return FileResponse(IMAGE_PATH, media_type="image/png")
    return JSONResponse({"error": "Image not found"}, status_code=404) 
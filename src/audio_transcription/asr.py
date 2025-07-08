import speech_recognition as sr
import os
from pathlib import Path

from google import genai
from dotenv import load_dotenv

# Load environment variables
load_dotenv()
google_api_key = os.getenv('google_api_key')

# Todo: Record Audio
class ASR:
    def __init__(self):
        # Initialize the recognizer
        self.r = sr.Recognizer()
        # Use absolute path for the audio file
        self.audio_file = "/home/er/Documents/reasoning320/googleResoning/audiofile/ip.wav"
        # Create directory if it doesn't exist
        # os.makedirs(os.path.dirname(self.audio_file), exist_ok=True)

    def transcribe_audio(self):
        try:
            # Capture audio from the microphone
            with sr.Microphone() as source:
                print("Please say something...")
                audio = self.r.listen(source)

            # write audio to a WAV file
            with open(self.audio_file, "wb") as f:
                f.write(audio.get_wav_data())

            # Initialize Gemini client
            client = genai.Client(api_key=google_api_key)

            # Upload file exactly as shown in the documentation
            myfile = client.files.upload(file=self.audio_file)

            # Generate content with exact format from documentation
            response = client.models.generate_content(
                model="gemini-2.5-pro-preview-05-06",
                contents=["Transcribe this audio clip", myfile]
            )

            # print(response.text)
            return response.text

        finally:
            # Clean up the audio file
            if os.path.exists(self.audio_file):
                try:
                    print("Audio file removed")
                    # os.remove(self.audio_file)
                except:
                    pass  # Ignore cleanup errors

# if __name__ == "__main__":
#     asr = ASR()
#     asr.transcribe_audio()

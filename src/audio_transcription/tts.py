from google import genai
from google.genai import types
import wave
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()
google_api_key = os.getenv('google_api_key')

def wave_file(filename, pcm, channels=1, rate=24000, sample_width=2):
    with wave.open(filename, "wb") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(sample_width)
        wf.setframerate(rate)
        wf.writeframes(pcm)

class TTS:
    def __init__(self):
        self.client = genai.Client(api_key=google_api_key)
        self.output_dir = "/home/er/Documents/reasoning320/googleResoning/audiofile"
        os.makedirs(self.output_dir, exist_ok=True)
        self.output_file = os.path.join(self.output_dir, "tts_output.wav")

    def speak_text(self, text):
        try:
            response = self.client.models.generate_content(
                model="gemini-2.5-pro-preview-tts",
                contents=f"Say cheerfully: {text}",
                config=types.GenerateContentConfig(
                    response_modalities=["AUDIO"],
                    speech_config=types.SpeechConfig(
                        voice_config=types.VoiceConfig(
                            prebuilt_voice_config=types.PrebuiltVoiceConfig(
                                voice_name='Aoede',
                            )
                        )
                    ),
                )
            )

            data = response.candidates[0].content.parts[0].inline_data.data
            wave_file(self.output_file, data)
            
            # Play the audio file (you'll need to implement this based on your OS)
            self.play_audio()
            
            return True
        except Exception as e:
            print(f"Error in TTS: {str(e)}")
            return False

    def play_audio(self):
        # This is a simple implementation using afplay for macOS
        # You might want to use a different method based on your OS
        os.system(f"afplay {self.output_file}") 
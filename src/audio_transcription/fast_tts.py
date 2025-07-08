import pyttsx3
import os

class FastTTS:
    def __init__(self):
        # Initialize the TTS engine
        self.engine = pyttsx3.init()
        
        # Configure voice properties for a younger sound
        self.engine.setProperty('rate', 175)     # Slightly faster speech
        self.engine.setProperty('volume', 1.0)   # Full volume
        self.engine.setProperty('pitch', 50)     # Higher pitch for younger voice
        
        # Get available voices
        voices = self.engine.getProperty('voices')
        
        # Set a specific voice based on the operating system
        if os.name == 'posix':  # macOS or Linux
            # Try to set Samantha voice (younger female voice on macOS)
            target_voice = 'com.apple.speech.synthesis.voice.samantha'
        else:  # Windows
            # Try to set a female voice on Windows
            target_voice = 'HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Speech\Voices\Tokens\TTS_MS_EN-US_ZIRA_11.0'
        
        # Try to set the target voice
        voice_set = False
        for voice in voices:
            if target_voice in voice.id:
                self.engine.setProperty('voice', voice.id)
                voice_set = True
                print(f"Using voice: {voice.name}")
                break
        
        # If target voice not found, fall back to any female voice
        if not voice_set:
            for voice in voices:
                if 'female' in voice.name.lower() or 'f2' in voice.id.lower():
                    self.engine.setProperty('voice', voice.id)
                    print(f"Using fallback voice: {voice.name}")
                    break

    def speak_text(self, text):
        try:
            # Speak the text
            self.engine.say(text)
            self.engine.runAndWait()
            return True
        except Exception as e:
            print(f"Error in FastTTS: {str(e)}")
            return False

    def change_rate(self, rate):
        """Change the speech rate (words per minute)"""
        self.engine.setProperty('rate', rate)

    def change_volume(self, volume):
        """Change the volume (0.0 to 1.0)"""
        self.engine.setProperty('volume', volume)

    def change_pitch(self, pitch):
        """Change the pitch (0-100)"""
        self.engine.setProperty('pitch', pitch)

    def get_current_voice(self):
        """Print current voice settings"""
        voice = self.engine.getProperty('voice')
        rate = self.engine.getProperty('rate')
        volume = self.engine.getProperty('volume')
        print(f"Current voice: {voice}")
        print(f"Rate: {rate}")
        print(f"Volume: {volume}")

    def list_available_voices(self):
        """Print all available voices on the system"""
        voices = self.engine.getProperty('voices')
        print("\nAvailable voices:")
        for voice in voices:
            print(f"ID: {voice.id}")
            print(f"Name: {voice.name}")
            print("---")

# if __name__ == "__main__":
#     tts = FastTTS()
#     tts.list_available_voices()
#     tts.speak_text("Hello! This is a test of the text to speech system.") 

# 'com.apple.speech.synthesis.voice.karen',  # macOS
# 'com.apple.speech.synthesis.voice.samantha',  # macOS
# 'com.apple.speech.synthesis.voice.tessa',  # macOS
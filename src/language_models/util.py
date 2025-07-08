import os
from langchain.prompts.prompt import PromptTemplate
from langchain.chains import LLMChain
from langchain_google_genai import ChatGoogleGenerativeAI
import textwrap
import json
from datetime import datetime

from dotenv import load_dotenv

# Load the environment
load_dotenv()
os.environ["GOOGLE_API_KEY"] = os.getenv("google_api_key")

class Utility:
    def __init__(self):
        self.llm = ChatGoogleGenerativeAI(model="gemini-2.5-pro-preview-05-06")

    def vlm_user_response(self, observation):
        # 1) Define your new VLM template
        vlm_template = textwrap.dedent("""
            You are Jettison’s Vision Module, a perceptive robotic assistant with a top-mounted camera.

            Observation: {observation}

            Please describe everything you see in one smooth paragraph: mention object types, their relative positions (e.g. top-left, bottom-right), and note any defects such as bent pins or cracks. 
            Do NOT use bullet points, numbered steps, headings, or summary labels—just one natural, fluid paragraph.
        """).strip()

        vlm_prompt = PromptTemplate.from_template(vlm_template)

        # 2) Build your VLM chain
        vision_chain = LLMChain(
            llm=self.llm,  # or your VLM client wrapper
            prompt=vlm_prompt,
            verbose=False,
        )

        # 3) Call it
        scene_desc = vision_chain.run({
            "observation": observation,
        })
        # print("scene_desc:", scene_desc)

        return scene_desc


# if __name__ == '__main__':
#     ob = Utility()
#     observation = """Alright, let's take a look at these relays.
#
#         I can see four relays on what appears to be a white surface. They are arranged in a roughly rectangular pattern.
#
#         *   Relay 1 (top-left): A brown relay with "TOYOTA" and other markings visible. I don't see any obvious defects on its surface.
#         *   Relay 2 (top-right): A gray relay, also with some text and a diagram. Its pins look straight.
#         *   Relay 3 (bottom-left): Another brown relay, like Relay 1. Again, no immediately visible damage.
#         *   Relay 4 (bottom-right): A blue relay. Its pins seem intact as well.
#
#         Overall, the relays appear to be in good condition from what I can see in this view. There aren't any obvious bent pins or surface cracks.
#
#         **Defect Summary:**
#         *   None detected.
#     """
#
#     ob.vlm_user_response(observation=observation)
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


class UAResponse:
    def __init__(self):
        self.llm = ChatGoogleGenerativeAI(model="gemini-2.5-pro-preview-05-06")
        self.memory_directory = "/home/er/Documents/reasoning320/googleResoning/memory"
        self.response_logs = "response_logs.jsonl"

        # 1) Build the prompt
        self.structured_template = textwrap.dedent("""
            You are Jettison, a highly specialized robotic assistant deployed on a robotic arm system.

            Your exclusive responsibility is to:
              - Focus only on relays that are **defective** (e.g., bent pins, cracks, surface damage).
              - Understand user instructions that relate to **sorting or removing defective relays**.
              - Use the given scene description to identify relevant relays.
              - Reason step-by-step through your action plan using structured logic and sound engineering judgment.
              - Do not actually execute actions; instead, conclude with: *"I will now proceed to execute this plan."*

            ==============================
            Primary Objective
            ==============================

            Observation: {observation}
            User Command: {user_command}

            Your job is to:
              1. Confirm understanding of the user’s intent.
              2. Focus exclusively on relays identified as **defective** in the observation.
              3. Translate the command into a step-by-step, logically reasoned plan.
              4. Include your current arm position or gripper status if relevant.
              5. Describe what the final state should look like after task completion.
              6. End with a note indicating that the plan is ready for execution.

            ==============================
            Expected Output Format
            ==============================

            **Intent Recognition**
            > "The user has requested that I sort or remove the defective relay."

            **Defect Analysis**
            > "From the observation, Relay 1 (top-left) has a bent pin..."

            **Action Plan (Step-by-Step Reasoning)**
            **Current State**
            - Arm: home position
            - Gripper: open
            **Planned Steps**
            1. Move arm to Relay 1...
            2. Align gripper...
            ...

            **Post-Action State**
            - Relay removed and placed in the discard bin.
            - Arm returned to home, gripper open.

            **Execution Readiness**
            > "I will now proceed to execute this plan."

            ==============================
            Input/Output Protocol
            ==============================

            Input:
                Observation: {observation}
                User Command: {user_command}

            Output:
                Follow the structured format above.
        """).strip()

        self.structured_prompt = PromptTemplate.from_template(self.structured_template)

        # 2) Set up and invoke the chain
        self.structured_chain = LLMChain(
            llm=self.llm,
            prompt=self.structured_prompt,
            verbose=False)

        # 2) a lightweight “rewriter” chain
        self.rewrite_chain = LLMChain(
            llm=self.llm,
            prompt=PromptTemplate.from_template(textwrap.dedent("""
                        Here is a step‑by‑step, structured plan:
                        {structured_plan}

                        Please rewrite the above into ONE seamless paragraph. Start by
                        acknowledging the command (“Okay, you’re asking me to…”), then briefly
                        describe what you saw, then describe your actions, and finish with
                        “I will now proceed to execute this plan.” Do NOT include any headings,
                        bullet points, or numbered lists.
                    """).strip()),
            verbose=False,
        )

    def response_and_action(self, user_query: str, scene_desc: str) -> str:
        """
        Given a user command and the latest scene description, generate
        a structured, step-by-step plan for sorting defective relays.

        Returns the LLM’s text response.
        """
        inputs = {
            "observation": scene_desc,
            "user_command": user_query,
        }
        structured_result = self.structured_chain.run(inputs)
        # print(structured_result)
        # print("\n\n")

        # 2) rewrite it into a conversational paragraph
        conversational = self.rewrite_chain.run({
            "structured_plan": structured_result
        })

        # 3) (optional) log both if you like
        self._log_action_plan(user_query, structured_result, conversational)

        # print(conversational)
        return conversational


    def _log_action_plan(self, user_query: str, structured_result: str, conversational: str):
        """Append the generated plan to the scene DB for traceability."""
        entry = {
            "timestamp": datetime.utcnow().isoformat(),
            "user_command": user_query,
            "structured_plan": structured_result,
            "conversational": conversational,
        }

        # Construct the full path
        file_name = os.path.join(self.memory_directory, self.response_logs)

        with open(file_name, "a") as f:
            f.write(json.dumps(entry) + "\n")


if __name__ == '__main__':
    ob = UAResponse()
    scene_desc = "Okay, here is my analysis of the scene.\n \n I see four relays placed on a white surface, likely a sheet of paper, which itself is resting on a wooden surface.\n \n Relay 1, located on the top-left, has a slightly bent pin on the right side. The bent pin appears to be on the right, facing the other pins.\n \n Relay 2, on the top-right, looks intact from my vantage point.\n \n Relay 3, in the bottom-left, also looks undamaged.\n \n Relay 4, which is in the bottom-right, looks like it is in good shape, with no visible defects.\n \n **Defect Summary:**\n - Relay 1: Bent pin on the right side.\n \n Let me know if you have another image for review!",
    user_query = 'Can you sort the defective relay for me'
    conversational = ob.response_and_action(user_query=user_query, scene_desc=scene_desc)
    print(conversational)

import os
from langchain.prompts.prompt import PromptTemplate
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_core.output_parsers import StrOutputParser

from dotenv import load_dotenv

# Load environment variables
load_dotenv()
os.environ["GOOGLE_API_KEY"] = os.getenv("google_api_key")

class Router:
    def __init__(self):
        self.llm = ChatGoogleGenerativeAI(model="gemini-2.5-pro-preview-05-06")

    def identify_route(self, user_query):
        template = """
            You are Jetti’s router. 
            Given the user query, classify it as either:
              - "vision"  (if they want you to describe what you see)
              - "action"  (if they want you to plan or execute a task)
            Do not respond with more than one word. Only output the single word key.
            
            <user query>
            {user_query}
            </user query>
        """.strip()

        prompt = PromptTemplate.from_template(template)

        chain = prompt | self.llm | StrOutputParser()

        # 5. Invoke the Chain
        result = chain.invoke({"user_query" : user_query})

        # 6. Print the Result
        # print(result)

        return result


# if __name__ == '__main__':
#     ob = Router()
#     user_query = "Do you see any defective relay"
#     ob.identify_route(user_query)
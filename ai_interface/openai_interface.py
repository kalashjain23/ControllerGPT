import glob

from langchain.chat_models import ChatOpenAI
from langchain.prompts.chat import (
    ChatPromptTemplate,
    SystemMessagePromptTemplate,
    HumanMessagePromptTemplate,
)

class AIInterface:
    def __init__(self, key: str, model: str) -> None:
        self.chat = ChatOpenAI(openai_api_key=key, temperature=0.7, model=model)
        self.messages = ""
        txt_files = glob.glob("messages/*.txt")
        for file in txt_files:
            self.messages += "<" + file.split(".txt")[0][9:] + ">"
            with open(file, 'r') as rd:
                self.messages += rd.read()
            self.messages += "<" + file.split(".txt")[0][9:] + ">" + "\n"
        
    def get_messages(self, prompt: str) -> str:
        system_template = ('''
            Following are the format of the messages with their name as the tags.
            {message_format}

            Return a python list of these messages required in order to achieve the user's goals in ROS2 without any explanation. Goals will be delimited by the <prompt> tags. Do not append the names of the messages in the list. All the properties of the messages should be enclosed within double quotes.
            Each message represents 1 second of the goal done, so add messages for every second to the list according to the goals.
        ''')
        human_template = "<prompt>{prompt}<prompt>"
        
        system_message_prompt = SystemMessagePromptTemplate.from_template(system_template)
        human_message_prompt = HumanMessagePromptTemplate.from_template(human_template)
        chat_prompt = ChatPromptTemplate.from_messages(
            [system_message_prompt, human_message_prompt]
        )
        
        response = self.chat(
            chat_prompt.format_prompt(
                message_format=self.messages, prompt = prompt
            ).to_messages()
        ).content
        return response


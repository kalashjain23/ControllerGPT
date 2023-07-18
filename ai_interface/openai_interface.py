import glob
import json
from typing import Any

from langchain.chat_models import ChatOpenAI
from langchain.prompts.chat import (
    ChatPromptTemplate,
    SystemMessagePromptTemplate,
    HumanMessagePromptTemplate,
)

class AIInterface:
    def __init__(self, key: str, model: str):
        self.chat = ChatOpenAI(openai_api_key=key, temperature=0.7, model=model)
        
        '''Collecting formats of all the supported ROS2 interfaces'''
        self.interfaces = ""
        messages = glob.glob("msg/*.msg")
        services = glob.glob("srv/*.srv")
        for msg in messages:
            delimiter = "<msg:" + msg.split(".msg")[0][4:] + ">"
            with open(msg, 'r') as rd:
                self.interfaces += delimiter + rd.read() + delimiter +"\n"
        for srv in services:
            delimiter = "<srv:" + srv.split(".srv")[0][4:] + ">"
            with open(srv, 'r') as rd:
                self.interfaces += delimiter + rd.read() + delimiter +"\n"
        
    def get_interfaces(self, prompt: str) -> Any:
        system_template = ('''
            Following are the format of the interfaces in ROS2 delimited with their respective interface_type:interface_name as the tags.
            {interfaces_format}

            Return a python list of these interfaces required in order to achieve the user's goals in ROS2 without any explanation. Goals will be delimited by the <prompt> tags.
            
            Every element in the list should be of the following format:
            {output_format} (only consider the values in to_be_published and take respective interface_type from the given description)
            All the properties of the messages should be enclosed within double quotes.
            
            Each element in the list represents 1 second of the goal done, so add interfaces for every second to the list according to the goals.
        ''')
        human_template = "<prompt>{prompt}<prompt>"
        
        system_message_prompt = SystemMessagePromptTemplate.from_template(system_template)
        human_message_prompt = HumanMessagePromptTemplate.from_template(human_template)
        chat_prompt = ChatPromptTemplate.from_messages(
            [system_message_prompt, human_message_prompt]
        )
        
        '''Format of the response given by ChatGPT'''
        output_format = '''{"category": msg/srv, "type": interface_type, "data": to_be_published}'''
        response = self.chat(
            chat_prompt.format_prompt(
                interfaces_format=self.interfaces, output_format=output_format, prompt = prompt
            ).to_messages()
        ).content
        return json.loads(response)

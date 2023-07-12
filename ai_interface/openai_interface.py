import openai
import json
import glob

class OpenAIInterface:
    def __init__(self, key: str) -> None:
        openai.api_key = key
        self.messages = ""
        txt_files = glob.glob("messages/*.txt")
        for file in txt_files:
            self.messages += "<" + file.split(".txt")[0][9:] + ">"
            
            with open(file, 'r') as rd:
                self.messages += rd.read()
                
            self.messages += "<" + file.split(".txt")[0][9:] + ">" + "\n"
        
    def get_messages(self, prompt: str, model: str = "gpt-3.5-turbo") -> str:
        system_message = f'''
            Following are the format of the messages with their name as the tags.
            {self.messages}

            Return a python list of these messages required in order to achieve the user's goals in ROS2 without any explanation. Goals will be delimited by the <prompt> tags. Do not append the names of the messages in the list. All the properties of the messages should be enclosed within double quotes.
            Each message represents 1 second of the goal done, so add messages for every second to the list according to the goals.
        '''
        
        messages = [{"role": "system", "content": system_message},
                    {"role": "user", "content": f"<prompt>{prompt}<prompt>"}]
        response = openai.ChatCompletion.create(
            model=model,
            messages=messages,
            temperature=0.7,
        )
        res = json.loads(response.choices[0].message["content"])
        return res

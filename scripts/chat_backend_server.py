#this is the backend server for chat
#it will be bused to manage the task treem the map, and action calls
#it will read from the response and determine which functions to call, in sequetial order

import os
import sys
import rospy
import subprocess
from std_msgs.msg import String
import json
from quest_tree import QuestNode, RootNode
from guide_dog_chat.srv import PublishQuestTree, PublishQuestTreeResponse,PublishTaskTree,PublishTaskTreeResponse
from modules import *

def clean_json(code):
    # Remove markdown code block delimiters
    code = code.replace('```json', '')
    code = code.replace('```', '')
    return code.strip()

def clean_code(code):
    # Remove markdown code block delimiters
    code = code.replace('```python', '')
    code = code.replace('```', '')
    code=code.replace('\\n','\n')
    code=code.replace('\\t','\t')
    code=code.replace('\\\n','\n')
    code=code.replace('\\\t','\t')
    
    return code.strip()
def clean_string(string):
    string=string.replace("'","\'")

    return string

def save_code_to_file(code, filename='generated_code.py'):
    with open(filename, 'w') as file:
        file.write(code)

def execute_code(filename='generated_code.py'):
    with open(filename, 'r') as file:
        code = file.read()
    exec(code, globals())


class ChatBackend:
    def __init__(self):
        global quest_tree
        
        rospy.init_node('chat_backend_server_node', anonymous=True)
        rospy.Subscriber('/chat_backend_server', String, self.callback)

        rospy.Service('publish_quest_tree', PublishQuestTree, self.handle_srv_quest_tree)
        

    def callback(self, msg):
        response = msg.data
        self.function_processing(response)

    def function_processing(self, response):

        print(response)

        response = clean_json(response)
        response = json.loads(response)
        if response["speak_to_user"]:
            print(response["speak_to_user"])
        if response["speak_to_public"]:
            print(response["speak_to_public"])
        if response["quest_tree_augment"]:
            print(response["quest_tree_augment"])
            self.execute_tree_function_list(response["quest_tree_augment"])
        if response["action_code"]:
            print(response["action_code"])
        response["action_code"] = f'speak_to_user("{response["speak_to_user"]}")\n{response["action_code"]}'

        print("quest tree")
        quest_tree.print_tree()
        save_code_to_file(clean_code(response["action_code"]))
        execute_code()
            
    def execute_tree_function_list(self, function_list_str):
        # Remove the square brackets and split the string into individual function calls
        function_calls = function_list_str.strip('[]').split(';')
        
        for function_call in function_calls:
            # Use eval to execute each function call
            eval(f'quest_tree.{function_call.strip()}')
        print(quest_tree)

    def handle_srv_quest_tree(self,req):
        quest_tree_str = quest_tree.get_tree_string()
        print("publishing quest tree")
        return PublishQuestTreeResponse(quest_tree_str)
    


     

if __name__ == '__main__':
    try:
        quest_tree = RootNode("root")
        ChatBackend()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass        

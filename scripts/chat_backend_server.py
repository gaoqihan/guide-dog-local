import os
import sys
import rospy
import subprocess
from std_msgs.msg import String
import json
from quest_tree import QuestNode, RootNode
from guide_dog_chat.srv import PublishQuestTree, PublishQuestTreeResponse, PublishTaskTree, PublishTaskTreeResponse
from modules import *
import threading

def clean_json(code):
    code = code.replace('```json', '')
    code = code.replace('```', '')
    return code.strip()

def clean_code(code):
    code = code.replace('```python', '')
    code = code.replace('```', '')
    code = code.replace('\\n', '\n')
    code = code.replace('\\t', '\t')
    code = code.replace('\\\n', '\n')
    code = code.replace('\\\t', '\t')
    return code.strip()

def clean_string(string):
    string = string.replace("'", "\'")
    return string

def save_code_to_file(code, filename='generated_code.py'):
    with open(filename, 'w') as file:
        file.write(code)
        file.write('\nreset_cancel()\n')

def execute_code(filename='generated_code.py', stop_event=None):
    with open(filename, 'r') as file:
        code = file.read()
    
    def safe_exec():
        global complete
        print('Executing code...')
        exec(code, globals())
        print("Reached end of code")
        complete = True

    try:
        safe_exec()
    except KeyboardInterrupt:
        print("Code execution was interrupted.")
    return True
def run_code_with_interrupt(filename='generated_code.py'):
    stop_event = threading.Event()
    thread = threading.Thread(target=execute_code, args=(filename, stop_event))
    thread.start()
    return thread, stop_event

def interrupt_execution(thread, stop_event):
    print("Interrupting the current execution.")
    stop_event.set()
    thread.join()  # Ensure the thread finishes

class ChatBackend:
    def __init__(self):
        global quest_tree,complete
        
        rospy.init_node('chat_backend_server_node', anonymous=True)
        rospy.Subscriber('/chat_backend_server', String, self.callback)

        rospy.Service('publish_quest_tree', PublishQuestTree, self.handle_srv_quest_tree)
        self.cencel_pub = rospy.Publisher('/cancel', String, queue_size=1)

    def callback(self, msg):
        global complete
        response = msg.data
        print("Received message:",complete)
        #if self.thread is not None:
        #   interrupt_execution(self.thread, self.stop_event)
        
        if not complete:
            print("Interrupting the current execution.")
            #interrupt_execution(thread, stop_event)
            self.cencel_pub.publish("cancel")
            self.thread.join()


        self.function_processing(response)

    def function_processing(self, response):
        global complete
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
        complete = False
        self.thread,self.stop_event=run_code_with_interrupt('generated_code.py')

    def execute_tree_function_list(self, function_list_str):
        function_calls = function_list_str.strip('[]').split(';')
        for function_call in function_calls:
            eval(f'quest_tree.{function_call.strip()}')
        print(quest_tree)

    def handle_srv_quest_tree(self, req):
        quest_tree_str = quest_tree.get_tree_string()
        print("publishing quest tree")
        return PublishQuestTreeResponse(quest_tree_str)

if __name__ == '__main__':
    try:
        quest_tree = RootNode("root")
        thread, stop_event = None, None
        complete=True
        ChatBackend()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
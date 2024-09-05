from typing import Any, List, Optional

import openai
import json
from vocode import getenv
from vocode.turn_based.agent.base_agent import BaseAgent
import re

import rospy
from guide_dog_chat.srv import PublishQuestTree, PublishQuestTreeRequest, PublishQuestTreeResponse
from guide_dog_chat.srv import PublishTaskTree, PublishTaskTreeRequest, PublishTaskTreeResponse

from std_msgs.msg import String
from sensor_msgs.msg import Image  # Add this import

from cv_bridge import CvBridge, CvBridgeError
import cv2
import base64  # Add this import

class GuideDogGPTAgent(BaseAgent):
    def __init__(
        self,
        system_prompt: str,
        api_key: Optional[str] = None,
        initial_message: Optional[str] = None,
        model_name: str = "gpt-4o",
        temperature: float = 0,
        max_tokens: int = 2000,
    ):
        super().__init__(initial_message=initial_message)
        api_key = getenv("OPENAI_API_KEY", api_key)
        if not api_key:
            raise ValueError("OpenAI API key not provided")
        self.client = openai.OpenAI(api_key=api_key)
        self.prompt = system_prompt
        with open("/home/user1/catkin_ws/src/guide_dog_chat/scripts/prompt/guide_dog_agent/system_prompt", "r") as file:
            guide_dog_prompt = file.read()
        
        

        
        self.model_name = model_name
        self.messages: List[Any] = [
            {
                "role": "system",
                "content": system_prompt+guide_dog_prompt,
            },
            {
                "role": "assistant",
                "content": initial_message,
            },
        ]
        self.chat_content_pub=rospy.Publisher("/chat_backend_server", String,queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            self.image_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def respond(self, human_input: str):
        quest_tree=call_publish_quest_tree()
        quest_tree_prompt="The quest tree is: "+quest_tree
        # Wait for an image frame to be received
        while self.image_frame is None:
            rospy.sleep(0.1)

        # Convert the image frame to a format suitable for sending (e.g., base64 encoding)
        _, buffer = cv2.imencode('.jpg', self.image_frame)
        image_base64 = base64.b64encode(buffer).decode('utf-8')

        self.messages.append(
            {
                "role": "user",
                "content": human_input + quest_tree_prompt,
                "image": image_base64  # Include the image in the message

            }
        )
        response = self.client.chat.completions.create(
            model=self.model_name,
            messages=self.messages,
            max_tokens=2000,
            temperature=0,
        )
        content = response.choices[0].message.content
        self.messages.append(
            {
                "role": "system",
                "content": content,
            }
        )
        #print(repr(content)[1:-1])
        # Extract the string enclosed by first '{' and last '}'
        # Find the first '{' and the last '}'
        start_index = content.find('{')
        end_index = content.rfind('}')
        if start_index != -1 and end_index != -1:
            content = content[start_index:end_index + 1]

            #print(extracted_string)
        else:
            print("No string found")
        #content_dict = json.loads(content)
        self.chat_content_pub.publish(content)       

        
        
        return content

        
def call_publish_quest_tree():
    rospy.wait_for_service('publish_quest_tree')
    try:
        publish_quest_tree = rospy.ServiceProxy('publish_quest_tree', PublishQuestTree)
        response = publish_quest_tree(PublishQuestTreeRequest())
        return response.tree
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return ""


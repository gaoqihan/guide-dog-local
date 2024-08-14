from vocode.turn_based.turn_based_conversation import TurnBasedConversation
from loguru import logger
import sys

from vocode.turn_based.agent.base_agent import BaseAgent
from vocode.turn_based.input_device.base_input_device import BaseInputDevice
from vocode.turn_based.output_device.base_output_device import BaseOutputDevice
from vocode.turn_based.synthesizer.base_synthesizer import BaseSynthesizer
from vocode.turn_based.transcriber.base_transcriber import BaseTranscriber
# Configure loguru to log to a file and the console
logger.add("turn_based_conversation.log", rotation="1 MB")  # Log to a file with rotation
#logger.add(sys.stdout, level="INFO")  # Log to the console



class TurnBasedConversationPlus(TurnBasedConversation):
    def end_speech_and_respond(self):
        human_input = self.transcriber.transcribe(self.input_device.end_listening())
        logger.info(f"TranscriptionPlus: {human_input}")
        agent_response = self.agent.respond(human_input)
        
        
        speak_to_user = agent_response["speak_to_user"]
        
        speak_to_public = agent_response["speak_to_public"]
        self.send_to_public(speak_to_public)
        
        task_tree_augment = agent_response["task_tree_augment"]
        self.task_tree_augment(task_tree_augment)
        
        call_module = agent_response["call_module"]
        self.call_module(call_module)
        
        
        
        logger.info(f"Agent responsePlus: {agent_response}")
        self.output_device.send_audio(self.synthesizer.synthesize(speak_to_user))
    
    def call_module(self,module_command):
        print("TBD call module developing")
        pass
    
    def send_to_public(self,message):
        print("TBD send to public developing")
        pass

    def task_tree_augment(self,task_tree_command):
        print("TBD task tree augment developing")
        pass
    
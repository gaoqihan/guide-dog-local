from pydantic_settings import BaseSettings, SettingsConfigDict

from vocode.helpers import create_turn_based_microphone_input_and_speaker_output
from vocode.turn_based.agent.chat_gpt_agent import ChatGPTAgent
from vocode.turn_based.synthesizer.azure_synthesizer import AzureSynthesizer
from vocode.turn_based.transcriber.whisper_transcriber import WhisperTranscriber
from vocode.turn_based.turn_based_conversation import TurnBasedConversation
from guide_dog_agent import GuideDogGPTAgent
from TurnBasedConversationPlus import TurnBasedConversationPlus

from dotenv import load_dotenv
import rospy
from std_msgs.msg import String, Int8
import time
class Settings(BaseSettings):
    """
    Settings for the turn-based conversation quickstart.
    These parameters can be configured with environment variables.
    """

    openai_api_key: str = ""
    azure_speech_key: str = ""

    azure_speech_region: str = "eastus"

    # This means a .env file can be used to overload these settings
    # ex: "OPENAI_API_KEY=my_key" will set openai_api_key over the default above
    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8", extra="ignore")

load_dotenv()

settings = Settings()

if __name__ == "__main__":
    #mode="testing"
    mode="onboard"
    rospy.init_node('chat_backend_client_node')


        
    (
        microphone_input,
        speaker_output,
    ) = create_turn_based_microphone_input_and_speaker_output(
        use_default_devices=True,
    )

    conversation = TurnBasedConversationPlus(
        input_device=microphone_input,
        output_device=speaker_output,
        transcriber=WhisperTranscriber(api_key=settings.openai_api_key),
        agent=GuideDogGPTAgent(
            system_prompt="",
            initial_message="Hello!",
            api_key=settings.openai_api_key,
        ),
        synthesizer=AzureSynthesizer(
            api_key=settings.azure_speech_key,
            region=settings.azure_speech_region,
            voice_name="en-US-SteffanNeural",
        ),
    )
    print("Starting conversation. Press Ctrl+C to exit.")
    ### TBD cahnge this to listen to topic /ButtonPressed
    def speak_to_user_callback(data):
        conversation.say_things(data.data)
    
    rospy.Subscriber('/speak_to_user', String, speak_to_user_callback)
    
    
    #looping
    if mode=="testing":
        while True:
            try:
                user_input=input("Press enter to start recording...")
                if user_input=="":
                    conversation.start_speech()
                input("Press enter to end recording...")
                agent_response=conversation.end_speech_and_respond(user_input)
                
            except KeyboardInterrupt:
                break
    elif mode=="onboard":
        recording=False
        last_pressed_time=time.time()
        print("press button to record")
        queue_num=0
        def button_pressed_callback(data):
            global last_pressed_time,recording,queue_num
            time_lapsed=time.time()-last_pressed_time
            last_pressed_time=time.time()
            print(last_pressed_time,time_lapsed)
            if time_lapsed<0.5:
                pass
            
            elif recording==False:
                if queue_num>0:
                    queue_num-=1
                    return
                    
                print("Starting recording")
                conversation.start_speech()
                recording=True
            else:
                queue_num=1
                print("Ending recording")
                agent_response=conversation.end_speech_and_respond()
                recording=False
                rospy.sleep(1)
                #rospy.Publisher('/speak_to_user', String, queue_size=10).publish(agent_response)
        rospy.Subscriber('/ButtonPress', Int8, button_pressed_callback,queue_size=1)
        rospy.spin()

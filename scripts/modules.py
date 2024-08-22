'''
MODULES:\ 
        following are the modules that you can use:\
            1. "go_to(<location>):None": this module will help the user navigate to the specified location.\
            2. "describe(<object>):String": this module will return a string that describe a certain object, to provide the user with more, especially visual information, to the user.\
            3. "find_object_3d(<object>):List[Int]": this module will look around and find a required object, then use lidar to find the 3D position of the object.\
                This module is used to find accurate position of a sepcific object.\
                This module will return a list of 3 integers, representing the x,y,z position of the object.\
            4. "look_around_and_find_related_objects((optional):description=<bool>,find=<keyword>)":Tuple(String,List[Tuple(String,Int)]): this module will use 360 camera to make observation of the surrounding,
                optionallly, it can provide a description of the surrounding if the argument 'description' is True, 
                it can also find objects related to the keywords provided in argument 'find'.\
                This module is used when user provides a gnere of objects, with intention to choose from them later.\
                This module will return a tuple containing A. a string and B. a list of tuples, each tuple in the list contains a string and an integer, the string is the object name, the integer is the direction of the object.\
                In case 'description'==False, the string will be an empty string.\
                In case 'find' is not provided, the list will be empty.\
            5. "read(<object>):String": this module captures the text on a requested object, to help the user understand the text on th object.\
                This module will return a string that contains the text on the object.\
            6. "describe_enrivonment(<direction:;front,back,left, right>):String": this module describe the entire environment captured using your front/side/back camera to the user.\
                This module will return a string that describes the environment.\
            7. "get_map_info(<location>):String": this module will help the user understand the map of the location. this module can be used for you to get information from the map as well\
                This module should be used when the user asks you for information that may be contained in the map.\
                This module will return a string that contains the information from the map. if nothing related is found, it should return an empty string\
            8. "wait_for_condition(<condition>):None": this module allows you to keep checking using camera whether a condition is met, you will not move untill a condition is met.\
            9. "speak_to_public(<message>)":None: this module will allow you to speak to the public, you must get approval from the user before using this module.\
            10. "speak_to_user(<message>)":None: this module will allow you to speak to the user.\
            11. "check_condition(<condition>)":Bool: this module will check if a condition is met, and return the condition.\
                This module returns a boolean value, True if the condition is met, False if the condition is not met.\
            12. "switch_mode(<mode:"guide","avoid","sleep">):Int": this module will switch the mode of the robot. \
                a. guide mode is default mode that guides the user around.\
                b. avoid mode allows the robot to stand still and avoid objects approaching it by moving around, it should be used when the robot is waiting for the user to move something\
                    arond it. \
                c. sleep mode will disable the robot's motion parts to save battery, while keep the voice command module running, this mode should be used when the robot's guidance will not\
                    be needed for a long time.\
                This module returns an integer, 0 if the mode is switched successfully, 1 if the mode is not switched successfully.\
'''

def go_to(location:str)->None:
    '''
    This module will help the user navigate to the specified location.
    '''
    print(f"Going to {location}")
    pass

def describe(object:str)->str:
    '''
    This module will return a string that describe a certain object, to provide the user with more, especially visual information, to the user.
    '''
    print(f"Describing {object}")
    return f"{object} looks very nice"

def find_object_3d(object:str)->list[int]:
    '''
    This module will look around and find a required object, then use lidar to find the 3D position of the object.
    This module is used to find accurate position of a sepcific object.
    This module will return a list of 3 integers, representing the x,y,z position of the object.
    '''
    print(f"Finding 3D position of {object}")
    return [1,2,3]

def look_around_and_find_related_objects(description:bool=False,find:str=None)->tuple[str,list[tuple[str,int]]]:
    '''
    This module will use 360 camera to make observation of the surrounding,
    optionallly, it can provide a description of the surrounding if the argument 'description' is True, 
    it can also find objects related to the keywords provided in argument 'find'.
    This module is used when user provides a gnere of objects, with intention to choose from them later.
    This module will return a tuple containing A. a string and B. a list of tuples, each tuple in the list contains a string and an integer, the string is the object name, the integer is the direction of the object.
    In case 'description'==False, the string will be an empty string.
    In case 'find' is not provided, the list will be empty.
    '''
    print(f"Looking around and finding related objects")
    if description:
        description_str = "This is a beautiful place"
    else:
        description_str = ""
    if find:
        objects = [(find,180),(find,270)]
    else:
        objects = []
    return description_str,objects

def read(object:str)->str:
    '''
    This module captures the text on a requested object, to help the user understand the text on th object.
    This module will return a string that contains the text on the object.
    '''
    print(f"Reading text on {object}")
    return f"Text on {object} is very interesting"

def describe_enrivonment(direction:str)->str:
    '''
    This module describe the entire environment captured using your front/side/back camera to the user.
    This module will return a string that describes the environment.
    '''
    print(f"Describing environment in {direction} direction")
    return "{direction} direction looks very nice,have many things"

def get_map_info(location:str)->str:
    '''
    This module will help the user understand the map of the location. this module can be used for you to get information from the map as well
    This module should be used when the user asks you for information that may be contained in the map.
    This module will return a string that contains the information from the map. if nothing related is found, it should return an empty string
    '''
    print(f"Getting map info for {location}")
    return f"Map info for {location} is as following: it is a good place, have many things"

def wait_for_condition(condition:str)->None:
    '''
    This module allows you to keep checking using camera whether a condition is met, you will not move untill a condition is met.
    '''
    while not check_condition(condition):
        print(f"Waiting for condition {condition}")
    print(f"Waiting for condition {condition}")
    pass

def check_condition(condition:str)->bool:
    '''
    This module will check if a condition is met, and return the condition.
    This module returns a boolean value, True if the condition is met, False if the condition is not met.
    '''
    print(f"Checking condition {condition}")
    return True

def speak_to_public(message:str)->None:
    '''
    This module will allow you to speak to the public, you must get approval from the user before using this module.
    '''
    print(f"Speaking to public: {message}")
    pass

def speak_to_user(message:str)->None:
    '''
    This module will allow you to speak to the user.
    '''
    print(f"Speaking to user: {message}")
    pass

def switch_mode(mode:str)->int:
    '''
    This module will switch the mode of the robot. 
    a. guide mode is default mode that guides the user around.
    b. avoid mode allows the robot to stand still and avoid objects approaching it by moving around, it should be used when the robot is waiting for the user to move something
        arond it. 
    c. sleep mode will disable the robot's motion parts to save battery, while keep the voice command module running, this mode should be used when the robot's guidance will not
        be needed for a long time.
    This module returns an integer, 0 if the mode is switched successfully, 1 if the mode is not switched successfully.
    '''
    print(f"Switching mode to {mode}")
    return 0

def set_current_task(task:str)->None:
    global quest_tree
    quest_tree.set_current_task(task)
    pass

def remove_child(command:str,recursive:bool=True)->None:
    global quest_tree
    quest_tree.remove_child(command,recursive)
    pass

def add_child(command:str,parent:str=None)->None:
    global quest_tree
    quest_tree.add_child(command,parent)
    pass


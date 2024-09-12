

import rospy
from std_msgs.msg import String , Int8, Float32
from geometry_msgs.msg import PoseStamped
import threading
import pickle
go_to_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)


speak_to_user_publisher=rospy.Publisher('/speak_to_user', String, queue_size=10)
speak_to_public_publisher=rospy.Publisher('/speak_to_public', String, queue_size=10)
find_object_3d_publisher=rospy.Publisher('/find_object', String, queue_size=10)
describe_environment_publisher=rospy.Publisher('/describe_environment', String, queue_size=10)
describe_object_publisher=rospy.Publisher('/describe_object', String, queue_size=10)
read_publisher=rospy.Publisher('/read', String, queue_size=10)
switch_mode_publisher=rospy.Publisher('/mode', Int8, queue_size=10)
cancel=False
def reset_cancel():
    global cancel
    cancel=False
    return
def cancel_callback(data):
    global cancel
    print("User cancelled the task")
    cancel=True
    #raise KeyError("User cancelled the task")

rospy.Subscriber('cancel', String, cancel_callback)
def save_variables(variable_list):
    '''
    Saves the given variables into a temporary file.

    This function serializes and saves the provided variables into a file named 'variables.pkl'.
    It allows variables to be shared across different conversations or sessions. This method 
    should be called when certain variables need to be preserved for future use, such as a list 
    of places found on a map waiting for user selection, or decisions awaiting user input.

    Parameters:
    variable_list (list): A list of variables to be saved.

    Returns:
    None
    '''
    with open('variables.pkl', 'wb') as f:
        pickle.dump(variable_list, f)
def load_variables():
    '''
    Loads variables from a temporary file.

    This function deserializes and loads variables from a file named 'variables.pkl'.
    It allows variables to be shared across different conversations or sessions. This method 
    should be called when certain variables were saved in the last conversation and need to be 
    used in the current one, such as a list of places found on a map waiting for user selection, 
    or decisions awaiting user input.

    Returns:
    list: A list of variables that were saved in the temporary file.
    '''
    with open('variables.pkl', 'rb') as f:
        return pickle.load(f)

def go_to(location, command='') -> None:
    global cancel
    if cancel:
        return
    '''
    Navigate to the specified location.

    Parameters:
    location (tuple or str): The target location. If a tuple, it should be the (x, y) coordinates on the map. 
                             If a string, it should be the name of the place to find in the map database.
    command (str, optional): An optional command string. If provided, it can be used to find the object in the surroundings.

    Behavior:
    - If the location is a tuple, it directly uses the coordinates.
    - If the location is a string, it searches the map database for matching places.
    - If the location is not found in the map, it attempts to find the object using the command.
    - If the object is still not found, it initiates an exploration routine.
    - Publishes the goal position to the 'go_to_publisher' topic.

    Returns:
    None
    '''
    if isinstance(location, tuple):
        x, y = location
    elif isinstance(location, str):
        #go_to_publisher.publish(location)
        location_info_list = get_map_info(location)

        #go_to(location_info["coordinate"])
        if location_info_list == []:
            # Location not found in map
            print("Location not found in map, finding in surroundings")
            speech="Location not found in map, looking around to find it"
            speak_to_user(speech)

            x, y = find_object_3d_from_command(command)
            if x == -1 and y == -1:
                speak_to_user("I can't find the location you are looking for, starting to explore a bit")
                exploration()
                return
            else:
                # Location found in surroundings
                speech = "I found the following locations in the surroundings"
                speech += f"guiding you there"
                speak_to_user(speech)
        else:
            # Location found in map
            speech = "I found the following locations in the map"
            location_info = location_info_list[0]
            speech += f"{location_info['name']} at {location_info['coordinate']}, guiding you there"
            speak_to_user(speech)
            x, y = location_info['coordinate']
    
    pose = PoseStamped()
    pose.header.frame_id = "map"

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    pose.header.stamp = rospy.Time.now()
    for i in range(5):
        go_to_publisher.publish(pose) 
    
    print(f"Going to {location}")
    #goal_status=rospy.wait_for_message('/move_base/goal_status', Int8)
    while not cancel:
        try:
            goal_status=rospy.wait_for_message('/move_base/goal_status', Int8,timeout=1)
            break
        except:
            pass
        #rospy.sleep(0.1)
    if cancel:
        return
    if isinstance(location, tuple):
        rounded_location = tuple(round(coord, 1) for coord in location)
        speak_to_user(f"Reached the location at {rounded_location}")
    else:
        speak_to_user(f"{location} arrived")
    return


    
def exploration():
    global cancel
    if cancel:
        return
    
    speak_to_user("exploration module not implemented yet")
    return

def describe(object:str)->str:
    global cancel
    if cancel:
        return "User cancelled the task"
    '''
    Provides a description of a specified object.

    Parameters:
    object (str): The name of the object to describe.

    Returns:
    str: A string describing the object.
    '''  
    description=''
    describe_object_publisher.publish(String(object))

    while True:
        try:
            goal_status=rospy.wait_for_message('/describe_object_complete', String,timeout=1)
            description=goal_status.data
            break
        except:
            pass
        #rospy.sleep(0.1)
    #print(f"Describing environment in {direction} direction")
    return description


def find_object_3d_from_command(object:str)->list[int]:
    global cancel
    print("cencel is ",cancel)
    if cancel:
        return (-1,-1)
    '''
    Finds the 3D position of a specified object using lidar.

    This function will look around to locate the required object and then use lidar to determine its 3D position.
    It is used to find the accurate position of a specific object.

    Parameters:
    object (str): The name of the object to find.

    Returns:
    tuple[float, float]: A tuple representing the (x, y) position of the object.
    '''
    print(f"Finding 3D position of {object}")
    find_object_3d_publisher.publish(String(object))

    result = []
    #result=rospy.wait_for_message('/best_goal', PoseStamped)
    
    while cancel==False:
        try:
            result=rospy.wait_for_message('/best_goal', PoseStamped,timeout=1)
            #rospy.sleep(1)
            #result=[]
            print("got result")
            x=result.pose.position.x
            y=result.pose.position.y
            break
        except:
            print("timeout")
            pass
        ##rospy.sleep(0.1)
    if cancel:
        return (-1,-1)
    return (x,y)
    

def look_around_and_find_related_objects(description:bool=False,find:str=None)->tuple[str,list[tuple[str,int]]]:
    global cancel
    if cancel:
        return "User cancelled the task",[]
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
    global cancel
    if cancel:
        return "User cancelled the task"
    '''
    This module captures the text on a requested object, to help the user understand the text on th object.
    This module will return a string that contains the text on the object.
    '''
    description=''
    read_publisher.publish(String(object))

    while not cancel:
        try:
            goal_status=rospy.wait_for_message('/read_complete', String,timeout=1)
            description=goal_status.data
            break
        except:
            pass
        #rospy.sleep(0.1)
    
    #print(f"Describing environment in {direction} direction")
    return description
def describe_environment(direction:str)->str:
    global cancel
    if cancel:
        return "User cancelled the task"
    '''
    This module describe the entire environment captured using your front/side/back camera to the user.
    This module will return a string that describes the environment.
    '''
    description=''
    describe_environment_publisher.publish(String(direction))
    def describe_environment_callback(data):
        nonlocal description
        print(data.data)
        description=data.data

    while not cancel:
        try:
            goal_status=rospy.wait_for_message('/describe_environment_complete', String,timeout=1)
            description=goal_status.data
            break
        except:
            pass
        #rospy.sleep(0.1)
    
    #print(f"Describing environment in {direction} direction")
    return description

def get_map_info(location: str) -> str:
    global cancel
    if cancel:
        return []
    '''
    Retrieves information about a specified location from the map.

    This function helps the user understand the map of a given location. It can be used to fetch 
    information from the map when the user requests details that may be contained within it. 
    The function returns a string containing the information from the map. If no relevant 
    information is found, it returns an empty string.

    Parameters:
    location (str): The name of the location to get information about.

    Returns:
        list: A list of dictionaries, each containing information about a specific point of interest 
            on the map. Each dictionary contains the following keys:
            - 'name': The name of the point of interest.
            - 'coordinate': A tuple representing the (x, y) coordinates of the point of interest.
            - 'note': A note describing the point of interest.
            If no information is found, returns an empty list.
    '''
    print(f"Getting map info for {location}")
    map_info_list = []
    example_map_info_entry = {
        "name": "convenient store",
        "coordinate": (100, 100),
        "note": "a nice convenient store with many things"
    }
    #map_info_list.append(example_map_info_entry)
    return map_info_list  # f"Map info for {location} is as following: it is a good place, have many things"
def wait_for_condition(condition:str)->None:
    global cancel
    if cancel:
        return
    '''
    This module allows you to keep checking using camera whether a condition is met, you will not move untill a condition is met.
    '''
    while not check_condition(condition):
        print(f"Waiting for condition {condition}")
    print(f"Waiting for condition {condition}")
    pass

def check_condition(condition:str)->bool:
    global cancel
    if cancel:
        return False
    '''
    This module will check if a condition is met, and return the condition.
    This module returns a boolean value, True if the condition is met, False if the condition is not met.
    '''
    print(f"Checking condition {condition}")
    return True

def speak_to_public(message:str)->None:
    global cancel
    if cancel:
        return
    '''
    This module will allow you to speak to the public, you must get approval from the user before using this module.
    '''
    
    print(f"Speaking to public: {message}")
    message=String(message)
    speak_to_public_publisher.publish(message)
    pass

def speak_to_user(message:str)->None:
    global cancel
    if cancel:
        return
    '''
    This module will allow you to speak to the user.
    '''
    print(f"Speaking to user: {message}")
    message=String(message)
    
    speak_to_user_publisher.publish(message)
    print("Published")
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
    if mode=="guide":
        switch_mode_publisher.publish(0)
    elif mode=="avoid":
        switch_mode_publisher.publish(1)
    elif mode=="sleep":
        switch_mode_publisher.publish(2)
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


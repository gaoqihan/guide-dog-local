speak_to_user("I will help you find the door.")
door_position = find_object_3d_from_command('door')
speak_to_user('I have found the door. It is located at position ' + str(door_position))
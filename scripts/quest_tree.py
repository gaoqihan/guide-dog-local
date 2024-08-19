class QuestNode:
    def __init__(self, command, parent=None,root=False):
        self.command = command
        self.parent = parent
        root=root
        self.children = []
        
    def print_tree(self,level=0):
        print("-"*level +">"+ self.command)
        for child in self.children:
            child.print_tree(level+1)
        
    def get_tree_string(self, level=0):
        tree_str = "-" * level + ">" + self.command + "\n"
        for child in self.children:
            tree_str += child.get_tree_string(level + 1)
        return tree_str            
    def add_child(self,command):
        child_node=self.create_child(command)
        self.children.append(child_node)
        child_node.parent = self

    def add_child_to_node(self,child_command,parent_command):
        node=self.get_child(parent_command,recursive=True)
        if node:
            node.add_child(child_command)
            return node
        else:
            return None

    def get_child(self,command,recursive=False):
        if self.command == command:
            return self
        if recursive:
            for child in self.children:
                if child.command == command:
                    return child
                else:
                    result = child.get_child(command,recursive)
                    if result:
                        return result
            return None
        else:
            for child in self.children:
                if child.command == command:
                    return child
        return None
    
    def remove_child(self,command,recursive=True):
        if recursive:
            for child in self.children:
                if child.command == command:
                    self.children.remove(child)
                    return
                else:
                    child.remove_child(command,recursive)
            return None
        else:
            for child in self.children:
                if child.command == command:
                    self.children.remove(child)
                    return
        return None
    def create_child(self,command):
        node = QuestNode(command,self)
        node.command = command
        return node
    
    
    def __str__(self, level=0):
        ret = "   " * level + repr(self.command) + "\n"
        for child in self.children:
            ret += child.__str__(level + 1)
        return ret

    def __repr__(self):
        return f"QuestNode({self.command})"
    
class RootNode(QuestNode):
    def __init__(self,command):
        super().__init__(command)
        self.root=True
        self.current_task=self
    def set_current_task(self,command):
        try:
            task=self.get_child(command,recursive=True)
            self.current_task=task
        except:
            raise ValueError(f"Task {command} not found")
   
def test_quest_tree():
    # Create the root node
    root = RootNode("root")

    # Add child nodes
    child1 = QuestNode("child1")
    child2 = QuestNode("child2")
    child3 = QuestNode("child3")
    root.add_child(child1)
    root.add_child(child2)
    root.add_child(child3)

    # Add grandchildren
    grandchild1 = QuestNode("grandchild1")
    grandchild2 = QuestNode("grandchild2")
    child1.add_child(grandchild1)
    child1.add_child(grandchild2)

    # Print the tree before removal
    print("Tree before removal:")
    root.print_tree()

    # Remove some nodes
    root.remove_child("child2")
    child1.remove_child("grandchild1")

    # Print the tree after removal
    print("Tree after removal:")
    root.print_tree()

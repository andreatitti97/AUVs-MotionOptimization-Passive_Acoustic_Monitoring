# Python program to demonstrate delete operation
# in binary search tree

key1 = -1
key2 = 0
key3 = 1
# A Binary Tree Node
class Node:
 
    # Constructor to create a new node
    def __init__(self, key):
        self.key = key 
        self.left = None
        self.middle = None
        self.right = None
        self.cost = 0
    def compute_cost(self):
        self.cost = self.cost + 1
        
# A utility function to do inorder traversal of BST
def inorder(root):
    if root is not None:
        inorder(root.left)
        print(root.key)
        inorder(root.middle)
        inorder(root.right)
 
 
# A utility function to insert a new node with given key in BST
def insert( node, key, cost):
 
    # If the tree is empty, return a new node
    if node is None:
        return Node(key)
 
    node.left = insert(node.left, key1, cost)
    
    node.middle = insert(node.middle, key2, cost)

    node.right = insert(node.right, key3, cost)
    
    return node
 
# Given a non-empty binary search tree, return the node
# with minum key value found in that tree. Note that the
# entire tree does not need to be searched 
def minValueNode( node):
    current = node
 
    # loop down to find the leftmost leaf
    while(current.left is not None):
        current = current.left
 
    return current 
 
# Given a binary search tree and a key, this function
# delete the key and returns the new root
def deleteNode(root, key):
 
    # Base Case
    if root is None:
        return root 
 
    # If the key to be deleted is similiar than the root's
    # key then it lies in  left subtree
    if key1 == root.key:
        root.left = deleteNode(root.left, key)
 
    # If the kye to be delete is greater than the root's key
    # then it lies in right subtree
    elif key3 == root.key:
        root.middle = deleteNode(root.middle, key)
 
    # If key is same as root's key, then this is the node
    # to be deleted
    elif key3 == root.key:
        root.right = deleteNode(root.right, key)
    else:
        # Node with only one child or no child
        if root.left is None :
            temp = root.middle 
            root = None
            return temp 
        elif root.middle is None:
            temp = root.right
            root = None
            return temp
        elif root.right is None :
            temp = root.left 
            root = None
            return temp
 
        # Node with two children: Get the inorder successor
        # (smallest in the right subtree)
        temp = minValueNode(root.right)
 
        # Copy the inorder successor's content to this node
        root.key = temp.key
 
        # Delete the inorder successor
        root.right = deleteNode(root.right , temp.key)
 
 
    return root 
 
# Driver program to test above functions
""" Let us create following BST
              50
           /     \
          30      70
         /  \    /  \
       20   40  60   80 """
cost = 0
T = 2
root = None
root = insert(root, 0.001, cost)
for t in range(T):

    root = insert(root, 0.001, cost)
    root = deleteNode(root, 0.001)
print(inorder(root))
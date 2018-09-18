import HobbitRosModule  # Import of API module

node = HobbitRosModule.node('demo') # Create ROS node
node.gripper('open')    # Open gripper
node.move_arm('floor')  # Move arm to floor pick up position
node.gripper('close')   # Close gripper = pick object
node.move_arm('table')  # Move to table position
node.gripper('open')    # Open gripper = drop object
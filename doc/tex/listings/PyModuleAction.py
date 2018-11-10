def sendActionGoal(self, namespace, action_type, goal, timeout = rospy.Duration()):
    exec('client = actionlib.SimpleActionClient(\''+namespace+'\', '+action_type+')')
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result(timeout)
    return client.get_result()
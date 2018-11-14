#!/usr/bin/env python
import HobbitLib
import rospy
from geometry_msgs.msg import Twist


if __name__ == '__main__':
	try:
		DemoNode = HobbitLib.node('DemoNode')

		message=Twist()
		message.linear.x=1
		HobbitLib.importMsg('geometry_msgs.msg','Twist')
		DemoNode.publishTopic('/cmd_vel', 'Twist', message)

	except rospy.ROSInterruptException:
		pass
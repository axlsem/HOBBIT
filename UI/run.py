#!/usr/bin/env python
import HobbitLib


if __name__ == '__main__':
	try:
		DemoNode = HobbitLib.HobbitNode('DemoNode')
		message = 'center_center'
		DemoNode.publishTopic('/head/move', String, message)

	except rospy.ROSInterruptException:
		pass
#!/usr/bin/env python
import HobbitLib
import rospy
from hobbit_msgs.srv import Request
from hobbit_msgs.srv import RequestRequest
from hobbit_msgs.msg import Parameter
from std_msgs.msg import Header

def srv21zxtebqdd3i():
	header = Header()
	header.stamp = rospy.Time.now()
	sessionID='0'
	txt='create'
	parr = []
	p = Parameter('type','D_YES_NO')
	parr.append(p)
	p = Parameter('text','Are you happy?')
	parr.append(p)
	p = Parameter('speak','Are you happy?')
	parr.append(p)
	reqparams=(header,sessionID,txt,parr)
	return DemoNode.callService('/MMUI', 'Request', reqparams)

if __name__ == '__main__':
	try:
		DemoNode = HobbitLib.node('DemoNode')
		HobbitLib.importMsg('hobbit_msgs.srv','Request')

		print(srv21zxtebqdd3i())

	except rospy.ROSInterruptException:
		pass
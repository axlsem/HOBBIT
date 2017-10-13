import rospy

class testclass:
	def __init__ (self, topic, msg, msgtype):
		self.topic = topic
		self.msg = msg
		self.msgtype = msgtype
		
	def publish (self):
		"pub = rospy.Publisher("+self.topic+", "+self.msgtype+", queue_size=10)"
		"rospy.init_node('demo', anonymous=True)"
		"rospy.loginfo("+self.msg+")"
		"pub.publish("+self.msg+")"
		
if __name__ == '__main__':
	try:
		head = testclass('/head/move', 'littledown_center', 'String')
		testclass.publish(head)
	except:
		pass
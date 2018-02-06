import rospy
from tf2_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from hobbit_msgs.msg import *
from hobbit_msgs.srv import *

def cbCheckPanPosition(data):
	global PAN_RUNNING
	PAN_RUNNING = data.is_moving

def cbCheckTiltPosition(data):
	global TILT_RUNNING
	TILT_RUNNING = data.is_moving

def WaitUntilPositionReached():
	global PAN_RUNNING
	global TILT_RUNNING
	
	PAN_RUNNING = True
	TILT_RUNNING = True

	rospy.Subscriber('/pan_controller/state', JointState, cbCheckPanPosition)
	rospy.Subscriber('/tilt_controller/state', JointState, cbCheckTiltPosition)
	
	while (PAN_RUNNING or TILT_RUNNING):
		continue

class HobbitNode:
	
	def __init__(self,NodeName='DemoNode'):
			self.Name = NodeName
			rospy.init_node(self.Name, anonymous=True)

			
	def publishTopic(self, topic, message_type, message):
		rate = rospy.Rate(1)
		exec('pub = rospy.Publisher(\''+topic+'\', '+message_type+', queue_size=10)')
		rate.sleep()
		pub.publish(message)
		rate.sleep()
		
		if topic == '/head/move':
			WaitUntilPositionReached(message)
	
			
	def callService(self,ServiceName,ServiceType,args):
		try:
			ParameterList = []
			if args:
				for i,arg in enumerate(args):
					exec('par'+str(i)+'=arg')
					ParameterList.append('par'+str(i))

			parameters = ','.join(ParameterList)

			rospy.wait_for_service(ServiceName)

		except rospy.ROSInterruptException, e:
			print "Service call failed: %s"%e
			return None
			
		try:
			exec('servicecall = rospy.ServiceProxy(\''+ServiceName+'\',' +ServiceType+')')
			exec('req = '+ServiceType+'Request('+parameters+')')
			resp = servicecall(req)
			return resp

		except rospy.ROSInterruptException, e:
			print "Service did not process request: %s"%str(e)
			return None
			
	def askYesNoQuestion(self,text,desiredAnswer):
		parr = []
		p = Parameter('type','D_YES_NO')
		parr.append(p)
		p = Parameter('text',text)
		parr.append(p)
		p = Parameter('speak',text)
		parr.append(p)
		p = Parameter('Repetitions','3')
		parr.append(p)
		p = Parameter('Timeout','15')
		parr.append(p)
		
		h = Header()
		h.stamp = rospy.Time.now()

		resp = self.callService('/MMUI','Request',(h,'0','create',parr))
		
		return resp.params[0].value == desiredAnswer
		
	def getUserInput(self,text):
		parr = []
		p = Parameter('type', 'D_NAME')
		parr.append(p)
		p = Parameter('text', text)
		parr.append(p)
		p = Parameter('Timeout', '30')
		parr.append(p)

		h = Header()
		h.stamp = rospy.Time.now()

		resp = self.callService('/MMUI','Request',(h,'0','create',parr))

		return resp.params[1].value
		
	def showInfo(self,text):
		parr = []
		p = Parameter('type', 'D_PLAIN')
		parr.append(p)
		p = Parameter('text', text)
		parr.append(p)
		p = Parameter('Timeout', '15')
		parr.append(p)
		p = Parameter('Repetitions', '1')
		parr.append(p)
		p = Parameter('wait', '1')
		parr.append(p)
		
		h = Header()
		h.stamp = rospy.Time.now()

		resp = self.callService('/MMUI','Request',(h,'0','create',parr))
		
	def showInfoOK(self,text):
		parr = []
		p = Parameter('type', 'D_OK')
		parr.append(p)
		p = Parameter('text', text)
		parr.append(p)
		p = Parameter('Timeout', '15')
		parr.append(p)
		p = Parameter('Repetitions', '3')
		parr.append(p)
		p = Parameter('wait', '1')
		parr.append(p)
		
		h = Header()
		h.stamp = rospy.Time.now()

		resp = self.callService('/MMUI','Request',(h,'0','create',parr))
		
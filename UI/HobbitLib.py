import rospy
from tf2_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from hobbit_msgs.msg import *
from hobbit_msgs.srv import *

global REACHED_POSITION
global DESIRED_POSITION
global ACTUAL_POSITION

def cbCheckPosition(data):
	global DESIRED_POSITION
	global ACTUAL_POSITION
	global REACHED_POSITION
	
	PositionMapping = dict()
	PositionMapping["up"] = -0.1
	PositionMapping["down"] = 0.3
	PositionMapping["center"] = 0.0
	PositionMapping["left"] = 0.6
	PositionMapping["right"] = -0.6
	
	delta = 0.03
	
	if (data.transforms[0].child_frame_id == 'hobbit_neck_dynamic'):

		y_act = data.transforms[0].transform.rotation.y
		z_act = data.transforms[0].transform.rotation.z
		
		if y_act <= PositionMapping["up"]:
			pos_ver = 'up'
		elif y_act >= PositionMapping["down"]:
			pos_ver = 'down'
		elif y_act >= PositionMapping["center"]-delta and y_act <= PositionMapping["center"]+delta:
			pos_ver = 'center'
		else:
			pos_ver = 'undef'

		if z_act <= PositionMapping["right"]:
			pos_hor = 'right'
		elif z_act >= PositionMapping["left"]:
			pos_hor = 'left'
		elif z_act >= PositionMapping["center"]-delta and z_act <= PositionMapping["center"]+delta:
			pos_hor = 'center'
		else:
			pos_hor = 'undef'

		ACTUAL_POSITION = pos_ver+"_"+pos_hor
		
		if(ACTUAL_POSITION == DESIRED_POSITION):
			REACHED_POSITION = True

def WaitUntilPositionReached(DesiredPosition):
	global REACHED_POSITION
	global DESIRED_POSITION

	DESIRED_POSITION = DesiredPosition
	REACHED_POSITION = False
	rospy.Subscriber('/tf', TFMessage, cbCheckPosition)
	
	while (not REACHED_POSITION):
		continue

class HobbitNode:
	
	def __init__(self,NodeName='DemoNode'):
			self.Name = NodeName
			rospy.init_node(self.Name, anonymous=True)

			
	def publishTopic(self, topic, message_type, message):
		rate = rospy.Rate(1)
		pub = rospy.Publisher(topic, message_type, queue_size=10)
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
		
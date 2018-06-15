import tf
import math
import rospy
import actionlib
from tf2_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from hobbit_msgs.msg import *
from hobbit_msgs.srv import *
from dynamixel_msgs.msg import *
from nav_msgs.msg import *

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

def getCurrentPose():
	global POSE
	rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, cbCurrentPose)
	rospy.sleep(0.1)
	return POSE

def cbCurrentPose(data):
	global POSE
	poseTmp = PoseStamped()
	poseTmp.header = data.header
	poseTmp.pose = data.pose.pose
	POSE = poseTmp

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
			WaitUntilPositionReached()
	
			
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

	def moveArm(self,armPosition):
		client = actionlib.SimpleActionClient('hobbit_arm', ArmServerAction)
		client.wait_for_server()
		goal = ArmServerGoal()
		goal.command.data = armPosition
		goal.velocity = 0.0
		goal.joints = []
		client.send_goal(goal)
		client.wait_for_result(rospy.Duration.from_sec(30.0))

	def move(self,distance):
		maxDistance = 0.15
		iterations = int(math.ceil(distance/maxDistance))
		direction = -1 if distance < 0 else 1

		message = Twist()

		if(abs(distance) > maxDistance):
			for i in range(1,iterations):
				message.linear.x = maxDistance*direction
				self.publishTopic('/cmd_vel','Twist',message)
			message.linear.x = distance-int(math.floor(abs(distance/maxDistance)))*maxDistance*direction
			self.publishTopic('/cmd_vel','Twist',message)

	def navigate(self,position,orientation):
		point = Point(x=position[0],y=position[1],z=position[2])
		quaternion = Quaternion(x=quaternion[0],y=quaternion[1],z=quaternion[2],z=quaternion[3]

		message = PoseStamped()
		message.header.stamp = rospy.Time.now()
		message.header.frame_id = 'map'
		message.pose = Pose(point,quaternion)

		self.publishTopic('/move_base_simple/goal', 'PoseStamped', message)

	def turn(self,angle):
		pose = getCurrentPose()
		
		q = (pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w)
		currentOrientation = tf.transformations.euler_from_quaternion(q)
		newOrientation = tf.transformations.quaternion_from_euler(currentOrientation[0],currentOrientation[1],currentOrientation[2]+angle)

		pose.orientation.x = newOrientation[0]
		pose.orientation.y = newOrientation[1]
		pose.orientation.z = newOrientation[2]
		pose.orientation.w = newOrientation[3]

		message = PoseStamped()
		message.header.frame_id = pose.header.frame_id
		message.header.stamp = rospy.Time.now()
		message.pose = pose.pose

		self.publishTopic('/move_base_simple/goal', 'PoseStamped', message)
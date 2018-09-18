from smach import StateMachine
from smach_ros import SimpleActionState
from hobbit_msgs.msg import ArmServerGoal, ArmServerAction
from rospy import loginfo

# Instance of SMACH state machine
sm = StateMachine(['finished','aborted','preempted'])

with sm:
    # Definition of action goals
    goal_floor = ArmServerGoal(data='MoveToPreGraspFloor', velocity=0.0, joints=[])
    goal_table = ArmServerGoal(data='MoveToPreGraspTable', velocity=0.0, joints=[])
    goal_OpGrip = ArmServerGoal(data='OpenGripper', velocity=0.0, joints=[])
    goal_ClGrip = ArmServerGoal(data='CloseGripper', velocity=0.0, joints=[])

    # Assambly of the full state machine
    StateMachine.add('INITIAL_POS', SimpleActionState('hobbit_arm',ArmServerAction,goal=goal_OpGrip),transitions={'succeeded':'FLOOR_POS','aborted':'LOG_ABORT', 'preempted':'LOG_ABORT'})
    StateMachine.add('FLOOR_POS', SimpleActionState('hobbit_arm',ArmServerAction,goal=goal_floor),transitions={'succeeded':'CLOSE_GRIPPER','aborted':'LOG_ABORT', 'preempted':'LOG_ABORT'})
    StateMachine.add('GRIPPER_CLOSED', SimpleActionState('hobbit_arm',ArmServerAction,goal=goal_ClGrip),transitions={'succeeded':'TABLE_POS','aborted':'LOG_ABORT', 'preempted':'LOG_ABORT'})
    StateMachine.add('TABLE_POS', SimpleActionState('hobbit_arm',ArmServerAction,goal=goal_table),transitions={'succeeded':'OPEN_GRIPPER','aborted':'LOG_ABORT', 'preempted':'LOG_ABORT'})
    StateMachine.add('GRIPPER_OPEN', SimpleActionState('hobbit_arm',ArmServerAction,goal=goal_OpGrip),transitions={'succeeded':'finished','aborted':'LOG_ABORT', 'preempted':'LOG_ABORT'})
    StateMachine.add('LOG_ABORT', loginfo('Demo aborted!'), transitions={'succeeded': 'aborted'})
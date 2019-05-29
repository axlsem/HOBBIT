<!-- # ROS reference for HOBBIT -->
<!-- generate-md --layout github --input ./ --output ./ -->
# Topics
This sections lists some topics which can be used to control HOBBIT.
## /head/move
Message Type: `std_msgs/String`

This topic is used to move HOBBIT's head.

 | Message             | Description                |
 | ------------------- | -------------------------- |
 | "center_center"     | look straight              |
 | "up_center"         | look up                    |
 | "down_center"       | look down                  |
 | "center_right"      | look right                 |
 | "center_left"       | look left                  |
 | "up_right"          | look to upper right corner |
 | "up_left"           | look to upper left corner  |
 | "down_right"        | look to lower right corner |
 | "down_left"         | look to lower left corner  |
 | "littledown_center" | look little down           |
 | "to_grasp"          | look to grasp              |
 | "to_turntable"      | look to turntable          |
 | "search_table"      | look to table              |

## /head/emo
Message Type: `std_msgs/String`

This topic is used to control HOBBIT's eyes/emotion

| Message     | Description       |
| ----------- | ----------------- |
| "HAPPY"     | look happy        |
| "VHAPPY"    | look very happy   |
| "LTIRED"    | look little tired |
| "VTIRED"    | look very tired   |
| "CONCERNED" | look concerned    |
| "SAD"       | look sad          |
| "WONDERING" | wonder            |
| "NEUTRAL"   | look neutral      |
| "SLEEPING"  | sleep             |

## /cmd_vel
Message type: `geometry_msgs/Twist`
This topic is used to move HOBBIT a certain distance in linear direction, i.e. it only uses the x coordinate of the linear part of the geometry_msgs/Twist Message. For example, the following message moves HOBBIT 2 metres forward:
```
'{linear:  {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

# Actions
This sections lists some action server namespaces which can be accessed to control HOBBIT.
## hobbit_arm
Action goal message type: `hobbit_msgs/ArmServerGoal`

Action message type: `hobbit_msgs/ArmServerAction`

HOBBIT's arm can be controlled in three different ways.

### Move arm along trajectory
To move the arm along a trajectory the joint values has to be passed to the goal, e.g.
```
command: 
  data: 'MoveAlongTrajectory'
velocity: 0.0
joints: [
  joint_values: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
  joint_values: [0.1, 0.1, 0.0, 0.0, 0.0, 0.0], 
  joint_values: [0.2, 0.2, 0.0, 0.0, 0.0, 0.0], 
  joint_values: [0.3, 0.3, 0.0, 0.0, 0.0, 0.0], 
  joint_values: [0.4, 0.4, 0.0, 0.0, 0.0, 0.0], 
  joint_values: [0.5, 0.4, 0.0, 0.0, 0.0, 0.0], 
  joint_values: [0.6, 0.4, 0.0, 0.0, 0.0, 0.0], 
  joint_values: [0.7, 0.4, 0.0, 0.0, 0.0, 0.0], 
  joint_values: [0.8, 0.4, 0.0, 0.0, 0.0, 0.0], 
  joint_values: [0.9, 0.4, 0.0, 0.0, 0.0, 0.0], 
  joint_values: [1.0, 0.4, 0.0, 0.0, 0.0, 0.0]
]
```

### Move to joint values
To move the arm to a specific position it is necessary to provide it via the values of the joints and set the command.data property accordingly, e.g.
```
command: 
  data: 'MoveToJointValues'
velocity: 0.0
joints: [
  joint_values: [1.0, 0.4, 0.0, 0.0, 0.0, 0.0]
]
```

### Move to predefined position
It is also possible to move HOBBIT's arm to a couple of predefined position. The following message, for example, lets the arm move to its home position:
```
command: 
  data: 'MoveToHome'
velocity: 0.0
joints: []
```

For all predefined position please refer to the following table.

| Command               | Description                     |
| --------------------- | ------------------------------- |
| "MoveToCandle"        | Move to candle position         |
| "MoveToHome"          | Move to home position           |
| "MoveToPreGraspFloor" | Prepare to grasp from floor     |
| "MoveToPreGraspTable" | Move to table position          |
| "MoveToTray"          | Move arm to tray                |
| "MoveToLearning"      | Grab turntable                  |
| "StoreTurntable"      | Store turntable                 |
| "TurnTurntableCW"     | Turn turntable clockwise        |
| "TurnTurntableCCW"    | Turn turntable counterclockwise |
| "OpenGripper"         | Open gripper                    |
| "CloseGripper"        | Close gripper                   |

## move_base
Action goal message type: `geometry_msgs/PoseStamped`

Action message type: `move_base_msgs/MoveBase`

This namespace can be used to navigate HOBBIT to a specific point determinated by its geometric pose, for example
```
header: 
  seq: 0
  stamp: now 
  frame_id: "map"
pose:
  position:
    x: 1.0
    y: 2.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```
> Note: Navigation is also possible by publishing a `geometry_msgs/PoseStamped` message to topic `/move_base_simple/goal`.

# Services
This sections lists some services which can be used to control HOBBIT.

## /MMUI
Service class: `hobbit_msgs/Request`

This service can be used to interact with the user via HOBBIT's tablet. The information is passed within the `params` Parameter array (type `hobbit_msgs/Parameter`) along some meta information. The following example shows how to prompt input from the user.
```
header:
  seq: 0
  stamp: now 
  frame_id: ""
sessionID: "0"
requestText: ""
params: [
  {name: "type", value: "D_NAME"},
  {name: "text", value: "How are you?"},
  {name: "Timeout", value: "30"}
]
```

Basically, the desired action is defined by the "type" parameter of the `params` array. Please refer to the following table for some common functionality of the tablet UI.

 params.type | Description 
 --- | ---
D_NAME | get user input from tablet keyboard*
D_PLAIN | show info
D_OK | show info and wait for confirmation
D_YES_NO | ask a yes-no-question*
F_CALLSOS | start SOS call
F_LOUDER | Set volume 10% higher
F_QUIETER | Set volume 10% lower

\* Please consider the following structure for the response:
```
{
  params:[
    {
      ...
      value: <UserInput>
      ...
    },
    {
      ...
    }
    ...
  ],
  ...
}
```

For `params.type='D_YES_NO'` *\<UserInput\>* is a string containing `'yes'` or `'no'` depending on which button the user has clicked. For `params.type='D_YES_NO'` *\<UserInput\>* is a string containing the string the user enterd via the tablet interface.

For more detailed examples using `rospy` please refer to the [this example](./examples).

# Message Definitions
## std_msgs/String
```
string data
```
## geometry_msgs/Twist
```
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
```
## geometry_msgs/Vector3
```
float64 x
float64 y
float64 z
```
## geometry_msgs/PoseStamped
```
std_msgs/Header header
geometry_msgs/Pose pose
```
## std_msgs/Header
```
uint32 seq
time stamp
string frame_id
```
## geometry_msgs/Pose
```
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
```
## geometry_msgs/Point
```
float64 x
float64 y
float64 z
```
## geometry_msgs/Quaternion
```
float64 x
float64 y
float64 z
float64 w
```
## hobbit_msgs/ArmServerGoal
```
std_msgs/String command
float64 velocity
hobbit_msgs/arm_joints joints
```

## hobbit_msgs/arm_joints
```
float32[6] joint_values
```
## hobbit_msgs/Parameter
```
string name
string value
```

# Service Classes
## hobbit_msgs/Request
```
#Request Service. used for requesting input from user/mmui by modules or vice-versa
Header header
string sessionID
string requestText # please enter a name for this place, etc.

Parameter[] params  # Requestparams
---
Parameter[] params  # Responseparams
```
# Introduction
rospy is a pure Python client library for ROS. The rospy client API enables Python programmers to quickly interface with ROS Topics, Services, and Parameters. 
# Writing a Publisher Node
This demo will walk you through creating a simple ROS node (*"talker"*), which will broadcast a message on topic *"chatter"*.

```python
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
## The Code Explained
Now, let's break the code down.
```python
#!/usr/bin/env python
```
Every Python ROS Node will have this declaration at the top. The first line makes sure your script is executed as a Python script.

```python
import rospy
from std_msgs.msg import String
```
You need to import rospy if you are writing a ROS Node. The std_msgs.msg import is so that we can reuse the std_msgs/String message type (a simple string container) for publishing.


```python
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
```
This section of code defines the talker's interface to the rest of ROS. `pub = rospy.Publisher("chatter", String, queue_size=10)` declares that your node is publishing to the chatter topic using the message type String. String here is actually the class `std_msgs.msg.String`. The queue_size argument is New in ROS hydro and limits the amount of queued messages if any subscriber is not receiving them fast enough. In older ROS distributions just omit the argument.

The next line, `rospy.init_node(NAME, ...)`, is very important as it tells rospy the name of your node -- until rospy has this information, it cannot start communicating with the ROS Master. In this case, your node will take on the name talker. NOTE: the name must be a base name, i.e. it cannot contain any slashes "/".

`anonymous = True` ensures that your node has a unique name by adding random numbers to the end of NAME. Refer to Initialization and Shutdown - Initializing your ROS Node in the rospy documentation for more information about node initialization options.


```python
    rate = rospy.Rate(10) # 10hz
```
This line creates a Rate object rate. With the help of its method sleep(), it offers a convenient way for looping at the desired rate. With its argument of 10, we should expect to go through the loop 10 times per second (as long as our processing time does not exceed 1/10th of a second!)


```python
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
```
This loop is a fairly standard rospy construct: checking the rospy.is_shutdown() flag and then doing work. You have to check is_shutdown() to check if your program should exit (e.g. if there is a Ctrl-C or otherwise). In this case, the "work" is a call to pub.publish(hello_str) that publishes a string to our chatter topic. The loop calls rate.sleep(), which sleeps just long enough to maintain the desired rate through the loop.

(You may also run across rospy.sleep() which is similar to time.sleep() except that it works with simulated time as well (see Clock).)

This loop also calls rospy.loginfo(str), which performs triple-duty: the messages get printed to screen, it gets written to the Node's log file, and it gets written to rosout. rosout is a handy for debugging: you can pull up messages using rqt_console instead of having to find the console window with your Node's output.

std_msgs.msg.String is a very simple message type, so you may be wondering what it looks like to publish more complicated types. The general rule of thumb is that constructor args are in the same order as in the .msg file. You can also pass in no arguments and initialize the fields directly, e.g.

```python
msg = String()
msg.data = str
```
or you can initialize some of the fields and leave the rest with default values:
```python
String(data=str)
```
You may be wondering about the last little bit:
```python
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
In addition to the standard Python `__main__` check, this catches a rospy.ROSInterruptException exception, which can be thrown by rospy.sleep() and rospy.Rate.sleep() methods when Ctrl-C is pressed or your Node is otherwise shutdown. The reason this exception is raised is so that you don't accidentally continue executing code after the sleep().

# Writing a Service Node
Here we'll create the service ("add_two_ints_server") node which will receive two ints and return the sum.

```python
#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
```
## The Code Explained
Now, let's break the code down.

There's very little to writing a service using rospy. We declare our node using init_node() and then declare our service:


```python
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
```
This declares a new service named add_two_ints with the AddTwoInts service type. All requests are passed to handle_add_two_ints function. handle_add_two_ints is called with instances of AddTwoIntsRequest and returns instances of AddTwoIntsResponse.

Just like with the subscriber example, rospy.spin() keeps your code from exiting until the service is shutdown.

# Writing a Servive Client Node
The following code calls the above created service.
```python
#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
```

## The Code Explained
Now, let's break the code down.

The client code for calling services is also simple. For clients you don't have to call init_node(). We first call:

```python
    rospy.wait_for_service('add_two_ints')
```
This is a convenience method that blocks until the service named add_two_ints is available. Next we create a handle for calling the service:


```python
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
```
We can use this handle just like a normal function and call it:


```python
        resp1 = add_two_ints(x, y)
        return resp1.sum
```
Because we've declared the type of the service to be AddTwoInts, it does the work of generating the AddTwoIntsRequest object for you (you're free to pass in your own instead). The return value is an AddTwoIntsResponse object. If the call fails, a rospy.ServiceException may be thrown, so you should setup the appropriate try/except block.

# Writing a Simple Action Client
This tutorial covers using the action_client library to create a Fibonacci simple action client in Python.
```python
#! /usr/bin/env python

import rospy
from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg

def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
```
## The Code Explained
```python
import actionlib_tutorials.msg
```
The action specification generates several messages for sending goals, receiving feedback, etc... This line imports the generated messages.


```python
    client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)
```
The action client and server communicate over a set of topics, described in the actionlib protocol. The action name describes the namespace containing these topics, and the action specification message describes what messages should be passed along these topics.


```python
    client.wait_for_server()
```
Sending goals before the action server comes up would be useless. This line waits until we are connected to the action server.


```python
    # Creates a goal to send to the action server.
    goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

    # Sends the goal to the action server.
    client.send_goal(goal)
```
Creates a goal and sends it to the action server.


```python
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult
```
The action server will process the goal and eventually terminate. We want the result from the termination, but we wait until the server has finished with the goal.
 Introduction
roscpp is a C++ implementation of ROS. It provides a client library that enables C++ programmers to quickly interface with ROS Topics, Services, and Parameters. roscpp is the most widely used ROS client library and is designed to be the high-performance library for ROS.

# Writing a Publisher Node
```Cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

```
## The Code Explained
Now, let's break the code down.
```Cpp
#include "ros/ros.h"
```
ros/ros.h is a convenience include that includes all the headers necessary to use the most common public pieces of the ROS system.

```Cpp
#include "std_msgs/String.h"
```
This includes the std_msgs/String message, which resides in the std_msgs package. This is a header generated automatically from the String.msg file in that package. For more information on message definitions, see the msg page.


```Cpp
  ros::init(argc, argv, "talker");
```
Initialize ROS. This allows ROS to do name remapping through the command line -- not important for now. This is also where we specify the name of our node. Node names must be unique in a running system.

The name used here must be a base name, ie. it cannot have a / in it.

```Cpp
  ros::NodeHandle n;
```
Create a handle to this process' node. The first NodeHandle created will actually do the initialization of the node, and the last one destructed will cleanup any resources the node was using.


```Cpp
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
```
Tell the master that we are going to be publishing a message of type std_msgs/String on the topic chatter. This lets the master tell any nodes listening on chatter that we are going to publish data on that topic. The second argument is the size of our publishing queue. In this case if we are publishing too quickly it will buffer up a maximum of 1000 messages before beginning to throw away old ones.

NodeHandle::advertise() returns a ros::Publisher object, which serves two purposes: 1) it contains a publish() method that lets you publish messages onto the topic it was created with, and 2) when it goes out of scope, it will automatically unadvertise.


```Cpp
  ros::Rate loop_rate(10);
```
A ros::Rate object allows you to specify a frequency that you would like to loop at. It will keep track of how long it has been since the last call to Rate::sleep(), and sleep for the correct amount of time.

In this case we tell it we want to run at 10Hz.
```Cpp
  int count = 0;
  while (ros::ok())
  {
```
By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause ros::ok() to return false if that happens.

ros::ok() will return false if:

- a SIGINT is received (Ctrl-C)
- we have been kicked off the network by another node with the same name
- ros::shutdown() has been called by another part of the application.
- all ros::NodeHandles have been destroyed

Once ros::ok() returns false, all ROS calls will fail.


```Cpp
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
```
We broadcast a message on ROS using a message-adapted class, generally generated from a msg file. More complicated datatypes are possible, but for now we're going to use the standard String message, which has one member: "data".

```Cpp
    chatter_pub.publish(msg);
```
Now we actually broadcast the message to anyone who is connected.


```Cpp
    ROS_INFO("%s", msg.data.c_str());
```
ROS_INFO and friends are our replacement for printf/cout. See the rosconsole documentation for more information.


```Cpp
    ros::spinOnce();
```
Calling ros::spinOnce() here is not necessary for this simple program, because we are not receiving any callbacks. However, if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called. So, add it for good measure.


```Cpp
    loop_rate.sleep();
```
Now we use the ros::Rate object to sleep for the time remaining to let us hit our 10Hz publish rate.

# Writing a Simple Service
This tutorial covers how to write a service node in C++.
```Cpp
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
```
Now, let's break the code down.

```Cpp
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
```
beginner_tutorials/AddTwoInts.h is the header file generated from the srv file that we created earlier.

```Cpp
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
```
This function provides the service for adding two ints, it takes in the request and response type defined in the srv file and returns a boolean.

```
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}
```
Here the two ints are added and stored in the response. Then some information about the request and response are logged. Finally the service returns true when it is complete.

```Cpp
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
```
Here the service is created and advertised over ROS.
# Writing a Simple Service Client
This tutorial covers how to write a service client node in C++.
```Cpp
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
```
## The Code Explained
Now, let's break the code down.
```Cpp
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
```
This creates a client for the add_two_ints service. The ros::ServiceClient object is used to call the service later on.

```Cpp
beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
```
Here we instantiate an autogenerated service class, and assign values into its request member. A service class contains two members, request and response. It also contains two class definitions, Request and Response.

```Cpp
  if (client.call(srv))
```
This actually calls the service. Since service calls are blocking, it will return once the call is done. If the service call succeeded, call() will return true and the value in srv.response will be valid. If the call did not succeed, call() will return false and the value in srv.response will be invalid.

# Writing a Simple Action Client
This tutorial covers using the simple_action_client library to create a Fibonacci action client. This example program creates an action client and sends a goal to the action server.
```Cpp
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
```

## The Code Explained
Now, let's break down the code piece by piece.
```Cpp
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
```
- actionlib/client/simple_action_client.h is the action library used from implementing simple action clients.
- actionlib/client/terminal_state.h defines the possible goal states.

```Cpp
#include <actionlib_tutorials/FibonacciAction.h>
```
This includes action message generated from the Fibonacci.action file shown above. This is a header generated automatically from the FibonacciAction.msg file. For more information on message definitions, see the msg page.

```Cpp
int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);
```
The action client is templated on the action definition, specifying what message types to communicate to the action server with. The action client constructor also takes two arguments, the server name to connect to and a boolean option to automatically spin a thread. If you prefer not to use threads (and you want actionlib to do the 'thread magic' behind the scenes), this is a good option for you. Here the action client is constructed with the server name and the auto spin option set to true.

```Cpp
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
```
Since the action server may not be up and running, the action client will wait for the action server to start before continuing.

```Cpp
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);
```
Here a goal message is created, the goal value is set and sent to the action server.

```Cpp
  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
```
The action client now waits for the goal to finish before continuing. The timeout on the wait is set to 30 seconds, this means after 30 seconds the function will return with false if the goal has not finished.

```Cpp
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
```
If the goal finished before the time out the goal status is reported, else the user is notified that the goal did not finish in the allotted time.
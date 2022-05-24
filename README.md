How to run the project
1. Connect the arduino microcontroller to the computer with a serial interface, like USB.
2. Open the terminal on the Ubuntu machine and change your directory to the catkin workspace
    cd E-Bike_Memory_Seat_With_Serial/catkin_ws
3. build the project
    $ catkin_make --force-cmake
4. setup the current environment by executing
    $ source devel/setup.bash
5. Start the ROS master node
    roscore
6. Start the rosserial communication between the Ubuntu machine and the arduino microcontroller (in a new terminal window)
    $ rosrun rosserial_python serial_node.py /dev/ttyACM0
    You can find the correct serial port by opening the Arduino IDE Tools/Port: "..."
7. Start the Action Client
    in a new terminal window repeat step 4.
    $ rosrun ebms_with_serial ebmsActionClient
8. Start the Action Server
    in a new terminal window repeat step 4.
    $ rosrun ebms_with_serial ebmsActionServer
9. Open ROS Mobile on your android device, go to the MASTER tab
10. Connect to the current IP address of ROS Master, you can find it with
    $ ifconfig
11. Open the VIZ tab and press one of the three buttons.
The buttons should be subscribed to the appropriate button topic where the Action Client is subscribed. Currently these topics are called /btn_high, /btn_medium and /btn_low.
Pressing any button will send a message to the action client. The action client will send a goal to the Action Server. The Action Server will send the new wanted seat height to the arduino. The arduino will begin moving the seat and sending feedback of the current position to the Action Server. The Action server will forward this feedback information to the Action Client. The Action Client will use this information to determine if the current action can be cancelled or if the microcontroller is ready to service a new goal. You can follow the transmission of these messages by using
    $ rostopic echo /topic_name
You can find all topic names by using
    $ rostopic list 
or
    $ rostopic list -v
for a more detailed list.

Useful Links:
https://programmer.group/ros-communication-mechanism-action-and-action-file.html
https://wiki.ros.org/actionlib/DetailedDescription
http://wiki.ros.org/roscpp/Overview/Time
It turns out that setPreempted is a relatively new method because it is not mentioned in the above wiki article. Instead a setCancelled() method is mentioned which can be found in simple_action_server_imp.h. Apparently setPreempted(result, text) calls "current_goal_.setCanceled(result, text);". Both methods accept two OPTIONAL parameters which are sent to any clients of the goal: a result and a text message. If no parameters are specified, some initialized values are used. For example my action's result has a u_int8_t value. If I don't pass my result to the setPreempted() method, it initializes its own result with a value of 0. What's more, setPreempted() as well as setSucceeded() not only set the state of the goal, they also publish the result and if you have not passed your result to the method like setPreempted(result), or setSucceeded(result), some other initialized by ROS result will be published. In my case, calling setPreempted() or setSucceeded() will also publish a result with value 0.  If you're not aware of that it can break your code's logic.
With that in mind, if we want to cancel the currently active goal from the Action Client, we can use the method actionClientPtr->cancelGoal();. When this method is called from the Action Client, it can be detected from the Action Server using the method actionServer.isPreemptRequested() which will return true. Then it is the server's responsibility to set the status of the goal to PREEMPTED using the method "actionServer.setPreempted(result);" as well as publish the appropriate result. 

TODO:
- When the client cancels the current goal, the arduino should be notified and take the appropriate action.
- Make a launch script
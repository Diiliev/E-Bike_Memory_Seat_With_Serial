## First time setup instructions
1. Setup the Arduino Uno microcontroller 
   - After connecting it to the PC, upload catkin_ws/src/ebms_with_serial/src/Microcontroller/EbmsMicrocontrollerWithSerial/EbmsMicrocontrollerWithSerial.ino
   - You can use a potentiometer connected to 5V, GND and A0 to manually generate feedback when an action has started.
2. Configure the launch file 
   - Open the catkin_ws/src/ebms_with_serial/launch/ebmsWithSerial.launch file
   - Change the IP address for the environment variables to your actual address. You can find it using `$ ifconfig`.
   - Make sure the port to which the microcontroller is connected is the same as the one in the launch file "/dev/ttyACM0". If it isn't change it to the actual port value.
3. In a new terminal, change your directory to the catkin workspace
```
$ cd E-Bike_Memory_Seat_With_Serial/catkin_ws
```
4. Build the project
```
$ catkin_make
```
5. Setup the current environment
```
$ source devel/setup.bash
```
6. Connect the android phone to the same Wi-Fi network as the PC.
7. Configure ROS Mobile
   - Click on "Add Configuration" and add three button widgets and one logger widget. The buttons should publish to topics named "btn_high", "btn_medium" and "btn_low". The logger should be subscribed to a topic named "log".
   - In the MASTER tab, Enter your IP address from step 1 into the Master URL field.
   - Master port should be 11311
   - launch ROS master or the entire project
   - Click on CONNECT and open the VIZ tab

## Quick launch instructions
1. Launch the project
```
$ roslaunch ebms_with_serial ebmsWithSerial.launch
```
This will start rosserial, the action client and the action server.
2. In ROS Mobile, click on a button.
This will start the appropriate action. You can monitor its progress in the logger field.


## How to run the project without the launch file
This is very useful for debugging purposes because every terminal wondow has its own output. The terminal running the rosserial node outputs messages sent from the microcontroller. The terminals running the action server and client nodes, output their respective messages.
1. Start the ROS master node
```
$ roscore
```
2. Start the rosserial communication between the Ubuntu machine and the arduino microcontroller (in a new terminal window)
```
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```
You can find the correct serial port by opening the Arduino IDE Tools/Port: "..."
3. Start the Action Client
```
$ source devel/setup.bash
$ rosrun ebms_with_serial ebmsActionClient
```
4. Start the Action Server
```
$ source devel/setup.bash
$ rosrun ebms_with_serial ebmsActionServer
```
5. Open ROS Mobile on your android device, go to the MASTER tab
6. Connect to the current IP address of ROS Master, you can find it with
```
$ ifconfig
```
7. Open the VIZ tab and press one of the three buttons.
Pressing any button will send a message to the action client. The action client will send a goal to the Action Server. The Action Server will send the new wanted seat height to the Arduino Uno microcontroller. The microcontroller will begin moving the seat and sending feedback of the current position to the Action Server. The Action server will forward this feedback information to the Action Client. The Action Client will use this information to determine when the microcontroller is ready to service a new goal.

## Useful Links:
https://programmer.group/ros-communication-mechanism-action-and-action-file.html - for action client-server communication with callbacks
https://wiki.ros.org/actionlib/DetailedDescription - for action server goal states and transitions
http://wiki.ros.org/roscpp/Overview/Time - for ROS Time and Duration variables
https://github.com/ROS-Mobile/ROS-Mobile-Android/wiki/FAQ - for ROS Mobile logger subscribing to a ROS topic
http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch#Using_roslaunch - for roslaunch
    http://wiki.ros.org/roslaunch/XML - for roslaunch
    https://nu-msr.github.io/me495_site/lecture04_launch.html - for roslaunch
    http://wiki.ros.org/roslaunch/XML/env - for roslaunch setting environment variables
http://wiki.ros.org/ROS/EnvironmentVariables#Node_Environment_Variables - for ROS environment variables
http://wiki.ros.org/rosserial_python - for rosserial port parameter name
It turns out that setPreempted is a relatively new method because it is not mentioned in the above wiki article. Instead a setCancelled() method is mentioned which can be found in simple_action_server_imp.h. Apparently setPreempted(result, text) calls "current_goal_.setCanceled(result, text);". Both methods accept two OPTIONAL parameters which are sent to any clients of the goal: a result and a text message. If no parameters are specified, some initialized values are used. For example my action's result has a u_int8_t value. If I don't pass my result to the setPreempted() method, it initializes its own result with a value of 0. What's more, setPreempted() as well as setSucceeded() not only set the state of the goal, they also publish the result and if you have not passed your result to the method like setPreempted(result), or setSucceeded(result), some other initialized by ROS result will be published. In my case, calling setPreempted() or setSucceeded() will also publish a result with value 0.  If you're not aware of that it can break your code's logic.
With that in mind, if we want to cancel the currently active goal from the Action Client, we can use the method actionClientPtr->cancelGoal();. When this method is called from the Action Client, it can be detected from the Action Server using the method actionServer.isPreemptRequested() which will return true. Then it is the server's responsibility to set the status of the goal to PREEMPTED using the method "actionServer.setPreempted(result);" as well as publish the appropriate result. 

In the current state of the project, when an action has started it will run until it finishes or the timer runs out. The only case in which an action is cancelled is when the timer of the Action Client runs out. New action requests can not preempt a previus action. If the user tries to send a new goal before the previus one has finished, their request will be ignored and they will be notified of that in real time. 

TODO:
- Make the arduino send the time left for the cooldown i.e "resting" period
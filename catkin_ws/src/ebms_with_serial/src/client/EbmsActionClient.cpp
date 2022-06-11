#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ebms_with_serial/adjustSeatHeightAction.h>
#include <std_msgs/Bool.h>

/*  If it takes ~135ms to move the actuator 1mm, then in the worst case scenario
    which is to travel from 0mm to 150mm a distance of 150mm, it will take the
    actuator 20.25s to finish the task. Afterwords the actutor must rest for 81s,
    because it has a duty cycle of 25%.
    We have rounded this number to compensate for different actuator speeds,
    depending on the load on the actuator, overshoot protection and communication
    delays. The seat adjustment action contains the seat adjustment period and the
    resting period. No seat adjustment action should take longer than 115s.
*/
#define ACTION_TIMER 115.0
#define HIGH 140
#define MEDIUM 70
#define LOW 10
#define RAISE 251
#define LOWER 250

typedef actionlib::SimpleActionClient<ebms_with_serial::adjustSeatHeightAction> Client;

class ClientListener {

    protected:
    ros::NodeHandle rosNode;
    Client ebmsActionClient;
    ros::Subscriber btn_high_sub;
    ros::Subscriber btn_medium_sub;
    ros::Subscriber btn_low_sub;
    ros::Subscriber btn_raise_sub;
    ros::Subscriber btn_lower_sub;
    u_int8_t directionCode; // The code 251 means RAISE, 250 means LOWER and 0 is just the initial value
    
    public:
    ClientListener() : ebmsActionClient("ebmsRosNode", true)
    {
        // initialize the direction code
        directionCode = 0;

        // wait for the action server to start
        ROS_INFO("Waiting for action server to start.");
        ebmsActionClient.waitForServer(); //will wait for infinite time

        // initialize subscribers
        btn_high_sub = rosNode.subscribe("btn_high", 1000, &ClientListener::btnHighCallback, this);
        btn_medium_sub = rosNode.subscribe("btn_medium", 1000, &ClientListener::btnMediumCallback, this);
        btn_low_sub = rosNode.subscribe("btn_low", 1000, &ClientListener::btnLowCallback, this);
        btn_raise_sub = rosNode.subscribe("btn_raise", 1000, &ClientListener::btnRaiseCallback, this);
        btn_lower_sub = rosNode.subscribe("btn_lower", 1000, &ClientListener::btnLowerCallback, this);
    }

    ~ClientListener(void) { }

    private:

    void btnHighCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        ROS_INFO("Button High is: [%d]", msg->data);
        sendGoalOnButtonPressed(msg->data, HIGH);
    }

    void btnMediumCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        ROS_INFO("Button Medium is: [%d]", msg->data);
        sendGoalOnButtonPressed(msg->data, MEDIUM);
    }

    void btnLowCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        ROS_INFO("Button Low is: [%d]", msg->data);
        sendGoalOnButtonPressed(msg->data, LOW);
    }

    void btnRaiseCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        ROS_INFO("Button Raise is: [%d]", msg->data);
        sendGoalOnButtonHold(msg->data, RAISE);
    }

    void btnLowerCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        ROS_INFO("Button Lower is: [%d]", msg->data);
        sendGoalOnButtonHold(msg->data, LOWER);
    }
    

    /**
     * @brief Send a goal height to the Action Server when a button is pressed
     * and reject new goals until the current one has finished executing.
     * There are three buttons in total which represent three goal heights:
     * Low, Medium and High. The buttons publish a boolean message to their
     * respective topic. When they are pressed they publish true, when released
     * they publish false. Only send new goals to the action server,
     * if the last one is done executing.
     * 
     * Note:
     * When the action timer runs out, a cancel request is sent to the server,
     * but the action will not be immediatelly cancelled. The server will wait
     * for the cooldown period of the actuator before cancelling the action.
     * The action will still be active, when the actuator has finished moving but
     * is in cooldown. This means that from the moment the cancel request was sent
     * until the moment the server actually cancelles the action, "cooldownTime"
     * ammount of time must pass.
     * 
     * Sidenote:
     * If we don't receive the remaining cooldown time from the microcontroller
     * that means it has most likely restarted or something has gone wrong.
     * In this case the action will be immediatelly cancelled by the server.
     * 
     * @param btnPressed represents the state of the button. True means it was
     * pressed, false means it was released.
     * @param btnHeight the seat height corresponding to the pressed button.
     * It can be one of three values defined above: HIGH, MEDIUM and LOW.
     * @param actionClientPtr a pointer to the action client used to send
     * the goal to the Action Server.
     */
    void sendGoalOnButtonPressed(bool btnPressed, u_int8_t btnHeight) {

        actionlib::SimpleClientGoalState lastGoalState = ebmsActionClient.getState();
        ROS_INFO("lastGoalState = %s, isDone() = %d.",lastGoalState.toString().c_str(), lastGoalState.isDone());

        if (btnPressed && lastGoalState.isDone()) {
            // send a goal to the action server
            ebms_with_serial::adjustSeatHeightGoal goal;
            goal.wantedHeight = btnHeight;

            // src: http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client
            ebmsActionClient.sendGoal(goal,
                                      boost::bind(&ClientListener::doneCb, this, _1, _2),
                                      boost::bind(&ClientListener::activeCb, this),
                                      boost::bind(&ClientListener::feedbackCb, this, _1));

            // wait for the action to complete and reject all incomming requests until then
            ros::Duration timerDuration(ACTION_TIMER);
            ros::Time endTime = ros::Time::now() + timerDuration;
            bool finishedBeforeTimeout = false;

            while (ros::ok()) {
                ros::Duration timeLeft = endTime - ros::Time::now();
                actionlib::SimpleClientGoalState goalState = ebmsActionClient.getState();

                // send a cancel request to the server when the timer runs out
                if (timeLeft <= ros::Duration(0, 0)) {
                    ROS_WARN("Action did not finish before the time out. Cancelling...");
                    ebmsActionClient.cancelGoal();
                    break;
                }
                if (goalState.isDone()) {
                    ROS_INFO("Action finished in time with state: %s", goalState.toString().c_str());
                    break;
                }

                ros::spinOnce();
            }
            
        } else if (btnPressed && !lastGoalState.isDone()) {
            ROS_INFO("Action can not be executed now. The current goal is in %s state.",lastGoalState.toString().c_str());
        }
    }

    /**
     * @brief Send a goal request, to raise or lower the seat height, to the Action Server
     * and reject new goals until the current one has finished executing.
     * 
     * - When the RAISE or LOWER button is pressed, the goal is sent to the Action Server
     * and the action is in ACTIVE state. Other button presses and releases will be ignored.
     * - When the RAISE or LOWER button is released we send a cancel request to the Server, but the
     * action state will remain ACTIVE until the cooldown period of the actuator has elapsed.
     * 
     * Note:
     * If we don't receive the remaining cooldown time from the microcontroller
     * that means it has most likely restarted or something has gone wrong.
     * In this case the action will be immediatelly cancelled by the server.
     * 
     * @param btnPressed represents the state of the button. True means it was
     * pressed, false means it was released.
     * @param btnDirection the numerical code corresponding to the direction of the pressed button.
     * It can be one of two values defined above: RAISE or LOWER.
     * @param actionClientPtr a pointer to the action client used to send
     * the goal to the Action Server and track its state.
     */
    void sendGoalOnButtonHold(bool btnPressed, u_int8_t btnDirection) {
        
        actionlib::SimpleClientGoalState lastGoalState = ebmsActionClient.getState();
        ROS_INFO("lastGoalState = %s, isDone() = %d.",lastGoalState.toString().c_str(), lastGoalState.isDone());
        std::string directionStr = (btnDirection == RAISE) ? "Raising" : "Lowering";

        if (btnPressed && lastGoalState.isDone()) {
            
            // set the direction code to the direction of the active action 
            directionCode = btnDirection;

            // send a goal to the action server
            ebms_with_serial::adjustSeatHeightGoal goal;
            goal.wantedHeight = btnDirection;
            ROS_INFO("%s the seat. Sending a %s request to the server.", directionStr.c_str(), directionStr.c_str());

            // src: http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client
            ebmsActionClient.sendGoal(goal,
                                        boost::bind(&ClientListener::doneCb, this, _1, _2),
                                        boost::bind(&ClientListener::activeCb, this),
                                        boost::bind(&ClientListener::feedbackCb, this, _1));

            // wait for the action to complete and reject all incomming requests until then
            ros::Duration timerDuration(ACTION_TIMER);
            ros::Time endTime = ros::Time::now() + timerDuration;
            bool finishedBeforeTimeout = false;

            while (ros::ok()) {
                ros::Duration timeLeft = endTime - ros::Time::now();
                actionlib::SimpleClientGoalState goalState = ebmsActionClient.getState();

                // send a cancel request to the server when the timer runs out
                if (timeLeft <= ros::Duration(0, 0)) {
                    ROS_WARN("Action did not finish before the time out. Cancelling...");
                    ebmsActionClient.cancelGoal();
                    break;
                }
                if (goalState.isDone()) {
                    ROS_INFO("Action finished in time with state: %s", goalState.toString().c_str());
                    break;
                }

                ros::spinOnce();
            }
            
        } 
        // Stop raising the seat when the button for the active action is released.
        // Releasing the Raise button can only stop a raising action, and the same applies for the lowering action.
        else if (!btnPressed && !lastGoalState.isDone() && directionCode == btnDirection) {
            ROS_INFO("Stop %s the seat. Sending a cancel request to the server.", directionStr.c_str());
            ebmsActionClient.cancelGoal();
        }
        else if (btnPressed && !lastGoalState.isDone()) {
            ROS_INFO("Action can not be executed now. The current goal is in %s state.",lastGoalState.toString().c_str());
        }
    }

    // thanks to https://programmer.group/ros-communication-mechanism-action-and-action-file.html
    // Callback that gets called on transitions to Done
    void doneCb(const actionlib::SimpleClientGoalState &state, const ebms_with_serial::adjustSeatHeightResultConstPtr &result) {
        if (state.state_ == state.SUCCEEDED) {
            ROS_INFO("Success! The final seat height is: %dmm", result->finalHeight);
        }
        else {
            ROS_INFO("Task failed. The final seat height is: %dmm", result->finalHeight);
        }
    }

    // Callback that gets called on transitions to Active
    void activeCb() {
        ROS_INFO("The action has been activated...");
    }

    // Callback that gets called whenever feedback for this goal is received
    void feedbackCb(const ebms_with_serial::adjustSeatHeightFeedbackConstPtr &feedback) {
        ROS_INFO("The current seat height is: %dmm", feedback->currentValue);
    }
};



int main (int argc, char **argv)
{
    ros::init(argc, argv, "ebmsRosClient");

    ClientListener listener;
    ROS_INFO("Action server started.");
    ros::spin();

    return 0;
}
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ebms_with_serial/adjustSeatHeightAction.h>
#include <std_msgs/Bool.h>

/*  If it takes ~135ms to move the actuator 1mm, then in the worst case scenario
    which is to travel from 0mm to 150mm a distance of 150mm, it will take the
    actuator 20.25s to finish the task. We have rounded this number to compensate for
    different actuator speeds depending on the load on the actuator, overshoot protection
    and communication delays. No seat adjustment action should take more than 30s.
*/
#define ACTION_TIMER 30.0
#define HIGH 140
#define MEDIUM 70
#define LOW 10

typedef actionlib::SimpleActionClient<ebms_with_serial::adjustSeatHeightAction> Client;

// thanks to https://programmer.group/ros-communication-mechanism-action-and-action-file.html
// Callback that gets called on transitions to Done
void doneCb(const actionlib::SimpleClientGoalState &state, const ebms_with_serial::adjustSeatHeightResultConstPtr &result) {
    if (state.state_ == state.SUCCEEDED) {
        ROS_INFO("Success! The final seat height is: %dmm", result->finalHeight);
    }
    else {
        ROS_INFO("Task failed. Goal state is: %s", state.toString().c_str());
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

/**
 * @brief Send a goal height to the Action Server when a button is pressed.
 * There are three buttons in total which represent three goal heights:
 * Low, Medium and High. The buttons publish a boolean message to their
 * respective topic. When they pressed they publish true, when released
 * they publish false. Only send new goals to the action server,
 * if the last one is done executing.
 * 
 * @param btnPressed represents the state of the button. True means it was
 * pressed, false means it was released.
 * @param btnHeight the seat height corresponding to the pressed button.
 * It can be one of three values defined above: HIGH, MEDIUM and LOW.
 * @param actionClientPtr a pointer to the action client used to send
 * the goal to the Action Server.
 */
void sendGoalOnButtonPressed(bool btnPressed, u_int8_t btnHeight, const boost::shared_ptr<Client> &actionClientPtr) {

    actionlib::SimpleClientGoalState lastGoalState = actionClientPtr->getState();

    if (btnPressed && lastGoalState.isDone()) {
        // send a goal to the action server
        ebms_with_serial::adjustSeatHeightGoal goal;
        goal.wantedHeight = btnHeight;
        actionClientPtr->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

        //wait for the action to return
        bool finished_before_timeout = actionClientPtr->waitForResult(ros::Duration(ACTION_TIMER));

        if (finished_before_timeout)
        {
            lastGoalState = actionClientPtr->getState();
            ROS_INFO("Action finished: %s",lastGoalState.toString().c_str());
        }
        // TODO what happens if we receive the goal height feedback message after timeout?
        // Maybe set state here to FAILED or try to pass the goalId with the goal
        // in order to receive feedback messages with the same ID.
        /*
            ------------------------------------------------------------------------------
            TODO how do we notify the microcontroller that the action has been cancelled?
            ------------------------------------------------------------------------------
        */
        else {
            ROS_INFO("Action did not finish before the time out. Cancelling...");
            actionClientPtr->cancelGoal();
        }
        
    } else if (!lastGoalState.isDone()) {
        ROS_INFO("Action can not be executed now. The current goal is in %s state.",lastGoalState.toString().c_str());
    }
}

void btnHighCallback(const std_msgs::Bool::ConstPtr& msg, const boost::shared_ptr<Client> &actionClientPtr)
{
    ROS_INFO("Button High is: [%d]", msg->data);
    sendGoalOnButtonPressed(msg->data, HIGH, actionClientPtr);
}

void btnMediumCallback(const std_msgs::Bool::ConstPtr& msg, const boost::shared_ptr<Client> &actionClientPtr)
{
    ROS_INFO("Button Medium is: [%d]", msg->data);
    sendGoalOnButtonPressed(msg->data, MEDIUM, actionClientPtr);
}

void btnLowCallback(const std_msgs::Bool::ConstPtr& msg, const boost::shared_ptr<Client> &actionClientPtr)
{
    ROS_INFO("Button Low is: [%d]", msg->data);
    sendGoalOnButtonPressed(msg->data, LOW, actionClientPtr);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ebmsRosClient");

    // create the action client as a shared pointer in order to pass it to the subscriber callback function.
    // true causes the client to spin its own thread
    // Thanks to user: naveedhd
    // https://answers.ros.org/question/267948/how-to-add-parameters-to-a-subscriber-callback-function-given-that-it-is-also-an-action_client/
    boost::shared_ptr<Client> actionClientPtr;
    actionClientPtr.reset(new Client("ebmsRosNode", true));

    // wait for the action server to start
    ROS_INFO("Waiting for action server to start.");
    actionClientPtr->waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started.");

    // initialize subscribers
    ros::NodeHandle rosNode;
    ros::Subscriber btn_high_sub = rosNode.subscribe<std_msgs::Bool>("btn_high", 1000, boost::bind(btnHighCallback, _1, actionClientPtr));
    ros::Subscriber btn_medium_sub = rosNode.subscribe<std_msgs::Bool>("btn_medium", 1000, boost::bind(btnMediumCallback, _1, actionClientPtr));
    ros::Subscriber btn_low_sub = rosNode.subscribe<std_msgs::Bool>("btn_low", 1000, boost::bind(btnLowCallback, _1, actionClientPtr));

    ros::spin();

    //exit
    return 0;
}
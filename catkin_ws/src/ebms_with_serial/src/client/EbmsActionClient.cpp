#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ebms_with_serial/adjustSeatHeightAction.h>
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<ebms_with_serial::adjustSeatHeightAction> Client;

// thanks to https://programmer.group/ros-communication-mechanism-action-and-action-file.html
// Callback that gets called on transitions to Done
void doneCb(const actionlib::SimpleClientGoalState &state, const ebms_with_serial::adjustSeatHeightResultConstPtr &result) {
    if (state.state_ == state.SUCCEEDED) {
        ROS_INFO("Final feedback result: %dmm", result->finalHeight);
    }
    else {
        ROS_INFO("Task failed.");
    }
}

// Callback that gets called on transitions to Active
void activeCb() {
    ROS_INFO("The action has been activated...");
}

// Callback that gets called whenever feedback for this goal is received
void feedbackCb(const ebms_with_serial::adjustSeatHeightFeedbackConstPtr &feedback) {
    ROS_INFO("Current height is: %dmm", feedback->currentHeight);
}


void sendGoalOnButtonPressed(bool btnPressed, u_int8_t btnHeight, const boost::shared_ptr<Client> &actionClientPtr) {

    if (btnPressed) {
        // send a goal to the action server
        ebms_with_serial::adjustSeatHeightGoal goal;
        goal.wantedHeight = btnHeight;
        actionClientPtr->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

        //wait for the action to return
        bool finished_before_timeout = actionClientPtr->waitForResult(ros::Duration(30.0));

        if (finished_before_timeout)
        {
        actionlib::SimpleClientGoalState state = actionClientPtr->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
        ROS_INFO("Action did not finish before the time out.");
    }
}

void btnHighCallback(const std_msgs::Bool::ConstPtr& msg, const boost::shared_ptr<Client> &actionClientPtr)
{
    ROS_INFO("Button High is: [%d]", msg->data);
    sendGoalOnButtonPressed(msg->data, 140, actionClientPtr);
}

void btnMediumCallback(const std_msgs::Bool::ConstPtr& msg, const boost::shared_ptr<Client> &actionClientPtr)
{
    ROS_INFO("Button Medium is: [%d]", msg->data);
    sendGoalOnButtonPressed(msg->data, 70, actionClientPtr);
}

void btnLowCallback(const std_msgs::Bool::ConstPtr& msg, const boost::shared_ptr<Client> &actionClientPtr)
{
    ROS_INFO("Button Low is: [%d]", msg->data);
    sendGoalOnButtonPressed(msg->data, 10, actionClientPtr);
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
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ebms_with_serial/adjustSeatHeightAction.h>
#include "std_msgs/String.h"

/**
 * @brief the following numbers are status codes sent from the microcontroller
 * their main charactaristic is that they are between 151 and 255, because
 * before they were sent as a character array message, each number was stored
 * in a byte typed variable where numbers from 0 to 150 represent the current
 * seat height in millimeters and the rest are used as status codes.
 * 
 */
#define STALLED_CODE 255
#define RESTING_CODE 254
#define DONE_CODE 253
#define CANCEL_CODE 252
#define RAISE_CODE 251
#define LOWER_CODE 250

/* 
    It is impossible for the seat height to be above 150mm.
    Nevertheless, we use this constant to initialise the currentSeatHeight variable,
    because if we initialize it with a legal value between 0 and 150,
    it could coincidentally be the same as the goal.wantedHeight.
    This would cause the action to finish successfully before we have received
    feedback from the microcontroller with the actual measured seat height.
    The number 151 was chosen to comply with the rule that all values above 150
    are considered to be "special" i.e mean sommeting different than height.

    Everytime the active action is completed, we use this constant to reset
    the currentSeatHeight variable, because the new goal might be the same as the old
    value of currentSeatHeight. In that case, if we had not reset the currentSeatHeight
    variable, the action would be completed successfully before we've had the chance
    to read what the actual measured position of the seat is from the microcontroller.
*/
#define INITIAL_VALUE 151
#define INITIAL_COOLDOWN_TIME_VALUE 0

// Topic names for communication with the microcontroller and ROS Mobile
#define SEND_TO_ARDUINO_TOPIC "newSeatHeight"
#define READ_FROM_ARDUINO_TOPIC "currentSeatHeight"
#define COOLDOWN_TIME_TOPIC "cooldownTime"
#define SEND_TO_USER_TOPIC "log" // logger widget topic name

#define MAX_SEAT_HEIGHT 150
#define DIGITS_OF_MAX_SEAT_HEIGHT 3
#define WAIT_FOR_RESPONSE 1 // this time duration is in seconds


class SeatHeightAdjuster {

    protected:
    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<ebms_with_serial::adjustSeatHeightAction> actionServer;
    std::string actionName;
    ros::Publisher seatHeightPub;
    ros::Subscriber seatHeightSub;
    ros::Subscriber cooldownTimeSub;
    ros::Publisher notifyUserPub;
    ebms_with_serial::adjustSeatHeightFeedback feedback;
    u_int8_t currentSeatHeight; 
    unsigned long cooldownTime;
    bool cooldownTimeIsNew;

    /**
     * @brief This flag is used to ensure that every time we write feedback.currentValue to currentSeatHeight,
     * this value is a new measurement received from the microcontroller and not some old remembered value.
     * 
     * This is important in case the seat height has been changed manually, or by some other means,
     * because without this flag we would have no way of knowing if the currentSeatHeight value is up to date. 
     * When this flag is used in combination with reseting the currentSeatHeight value after every action is
     * completed, we can handle cases where the goal height is the same as the one before, but the seat height
     * has been changed by some other means.
     */
    bool feedbackIsNew;  
    ebms_with_serial::adjustSeatHeightResult result;

    public:
    SeatHeightAdjuster(std::string name) :
        actionServer(nodeHandle, name, boost::bind(&SeatHeightAdjuster::executeCB, this, _1), false),
        actionName(name)
        {
            actionServer.start();

            // initialize class members
            seatHeightPub = nodeHandle.advertise<std_msgs::String>(SEND_TO_ARDUINO_TOPIC, 1000);
            seatHeightSub = nodeHandle.subscribe(READ_FROM_ARDUINO_TOPIC, 1000, &SeatHeightAdjuster::onMcuFeedback, this);
            cooldownTimeSub = nodeHandle.subscribe(COOLDOWN_TIME_TOPIC, 1000, &SeatHeightAdjuster::onCooldownTimeMsg, this);
            notifyUserPub = nodeHandle.advertise<std_msgs::String>(SEND_TO_USER_TOPIC, 1000);
            feedback.currentValue = INITIAL_VALUE;
            currentSeatHeight = INITIAL_VALUE;
            cooldownTime = INITIAL_COOLDOWN_TIME_VALUE;
            feedbackIsNew = false;
            cooldownTimeIsNew = false;
        }
    
    ~SeatHeightAdjuster(void) { }

    /**
     * @brief The time it takes to execute any given seat adjustment action is
     * the sum of the time it takes to reach the given position and the time it
     * takes for the actuator to cool down. These are referred to as the adjustment
     * period and the resting period. The sum of the adjustment period and the resting
     * period, i.e the action execution time, should not be longer than 115s for
     * any given goal height.
     * 
     * @param goal this contains the wanted seat height sent to the microcontroller.
     */
    void executeCB(const ebms_with_serial::adjustSeatHeightGoalConstPtr &goal) {

        ROS_INFO("Executing %s, wanted height is %imm",
            actionName.c_str(),
            goal->wantedHeight
        );

        // send the goal height to the microcontroller by publishing it to the pubToArduinoTopic.
        publishNewHeight(goal->wantedHeight);

        bool cancelRequestSent = false;
        
        // This loop checks the latest feedback messages from the microcontroller
        // with a frequency of ros::Rate, which is currently set to 10 times a second.
        // If the action is cancelled by the client, preempted by another goal or the ROS server is not "ok"
        // (For example it is stopped with Ctrl+C), the goal state is set to PREEMPTED.
        // If the actuator stalls, the action is aborted.
        // If the action takes too long to execute, the Action Client cancels the current goal
        // which is detected by the Action Server as a preempt request.
        while (feedback.currentValue != DONE_CODE && ros::ok()) {

            if (actionServer.isPreemptRequested() && !cancelRequestSent) {
                
                // notify the microcontroller that the action has been cancelled
                // and wait for it to finish resting
                publishNewHeight(CANCEL_CODE);
                ROS_WARN("The action was cancelled.");
                publishFeedbackToMobile("The action was cancelled.");
                cancelRequestSent = true;

                // if we don't receive a cooldownTime response after cancelling the action,
                // then break the loop, because the microcontroller has probably been restarted
                // and it will not return DONE_CODE
                ros::Duration(WAIT_FOR_RESPONSE).sleep();
                if (cooldownTime == INITIAL_COOLDOWN_TIME_VALUE) {
                    break;
                }
            }

            if (feedbackIsNew) {

                feedbackIsNew = false;

                if (feedback.currentValue <= MAX_SEAT_HEIGHT) {
                    currentSeatHeight = feedback.currentValue;
                    publishFeedbackToAC(feedback);
                    publishFeedbackToMobile("Current seat height is: ", currentSeatHeight, "mm");
                }
                else if (feedback.currentValue == STALLED_CODE) {
                    ROS_ERROR("The actuator has stalled!");
                    publishFeedbackToMobile("The actuator has stalled!");
                }
                else if (feedback.currentValue == RESTING_CODE) {
                    ROS_INFO("Resting...");
                    publishFeedbackToMobile("The actuator is in cooldown");
                }
            }

            if (cooldownTimeIsNew) {
                ROS_INFO("Cooldown: %lums remaining.", cooldownTime);
                publishFeedbackToMobile("Cooldown: ", cooldownTime, "ms remaining.");
                cooldownTimeIsNew = false;
            }
        }

        result.finalHeight = currentSeatHeight;

        if (result.finalHeight == goal->wantedHeight) {
            actionServer.setSucceeded(result, "Success"); 
            ROS_INFO("%s Succeeded :) the final height is: %dmm", actionName.c_str(), result.finalHeight);
        } else {
            actionServer.setPreempted(result);
            ROS_INFO("%s was preempted. The final height is: %dmm", actionName.c_str(), result.finalHeight);
        }
        
        // reset the feedback's current value and flags to be ready for the next action
        feedback.currentValue = INITIAL_VALUE;
        cooldownTime = INITIAL_COOLDOWN_TIME_VALUE;
        feedbackIsNew = false;
        cancelRequestSent = false;

        ROS_INFO("Ready for a new action goal.");
        publishFeedbackToMobile("Done! The final seat height is: ", result.finalHeight, "mm. The buttons are active!");
        
        return;
    }

    private:

    /**
     * @brief send the new wanted height to the microcontroller
     * by publishing it to the appropriate topic.
     * 
     * @param goalHeight the newly requested seat height.
     * It is a number between 0 and 150 millimeters.
     */
    void publishNewHeight(uint8_t goalHeight) {
        std_msgs::String goalMsg;
        char goalHeightToStr[DIGITS_OF_MAX_SEAT_HEIGHT + sizeof(char)];

        // convert int to string
        std::sprintf(goalHeightToStr, "%d", goalHeight);
        goalMsg.data = goalHeightToStr;

        seatHeightPub.publish(goalMsg);
    }

    // publish feedback to the Action Client
    /**
     * @brief send the current seat height to the Action Client
     * by pulblishing it as a feedback message to the appropriate topic
     * and mark this feedback message as not being new anymore.
     * 
     * @param feedbackMsg contains the current seat height.
     */
    void publishFeedbackToAC(const ebms_with_serial::adjustSeatHeightFeedback &feedbackMsg) {
        actionServer.publishFeedback(feedbackMsg);
    }

    /**
     * @brief Publish a string message to the /log topic
     * which is visualized in ROS Mobile's "logger" widged.
     * Example messages:
     * - The actuator has stalled!
     * - The action has been cancelled!
     * - Ready for a new request.
     * 
     * @param userMsg 
     */
    void publishFeedbackToMobile(std::string userMsg){
        std_msgs::String msg;
        msg.data = userMsg;
        notifyUserPub.publish(msg);
    }

    /**
     * @brief Publish a message containing a number surrounded by two strings
     * to the /log topic which is visualized in ROS Mobile's "logger" widget.
     * Example messages:
     * - The current seat height is: 51mm
     * - Cooldown, 12s remaining.
     * 
     * @param leftString 
     * @param number 
     * @param rightString 
     */
    void publishFeedbackToMobile(std::string leftString, unsigned long number, std::string rightString){
        std_msgs::String msg;
        msg.data = leftString + std::to_string(number) + rightString;
        notifyUserPub.publish(msg);
    }

    /**
     * https://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
     * @brief Execute the following when we receive a new message from the microcontroller.
     * 
     * @param msg the message received from the microcontroller.
     * It contains a number from 0 to 255 where every digit is a seperate char element in a char array.
     * Numbers from 0 to 150 represent the seat height in millimeters. Numbers bigger than that are
     * considered "special", because the have a different meaning. These numbers are used as codes
     * that give us infomration about the current status of the microcontroller or actuator.
     * For example 255 means the actuator has stalled. 254 means the microcontroller is resting and
     * is not accepting new requets. For more information see the code definitions at the top of the page.
     */
    void onMcuFeedback(const std_msgs::String::ConstPtr& msg) {
        feedback.currentValue = atoi(msg->data.c_str());
        feedbackIsNew = true;
    }

    /**
     * @brief get the cooldown time sent from the microcontroller in milliseconds.
     * Convert it to seconds and store it in the class member cooldownTime.
     * 
     * @param msg the std_msgs::String message sent from the microcontroller
     * containing the remaining cooldown time in milliseconds.
     */
    void onCooldownTimeMsg(const std_msgs::String::ConstPtr& msg) {
        cooldownTime = std::stoul(msg->data.c_str(), nullptr, 10);
        cooldownTimeIsNew = true;
    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "ebmsRosNode");

    SeatHeightAdjuster seatHeightAdjuster("ebmsRosNode");
    ROS_INFO("EBMS Action Server has started");
    ros::spin();

    return 0;
}
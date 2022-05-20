#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ebms_with_serial/adjustSeatHeightAction.h>
#include "std_msgs/String.h"

// In the arduino IDE, a byte typed variable is used to store the height.
// That is equivalent to an uint8_t variable, i.e values range from 0 to 255.
// All values above 150 (which is the nominal height of the actuator) are reserved as "Special" values
// The following "Special" values represent some sort of condition
// which is transmitted via result message to the Action Client.
#define PREEMPTED_CODE 254
#define STALLED_CODE 255

// Topic names for communication with the microcontroller
#define SEND_TO_ARDUINO "newSeatHeight"
#define READ_FROM_ARDUINO "currentSeatHeight"

#define MAX_SEAT_HEIGHT 150
#define DIGITS_OF_MAX_SEAT_HEIGHT 3

class SeatHeightAdjuster {

    protected:
    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<ebms_with_serial::adjustSeatHeightAction> actionServer;
    std::string actionName;
    ros::Publisher seatHeightPub;
    ros::Subscriber seatHeightSub;
    ebms_with_serial::adjustSeatHeightFeedback feedback;
    // since feedback carries not only the current seat height but also status codes like STALLED_CODE,
    // this variable is needed to store the current seat height value.
    u_int8_t currentSeatHeight = 0; 
    ebms_with_serial::adjustSeatHeightResult result;

    public:
    SeatHeightAdjuster(std::string name) :
        actionServer(nodeHandle, name, boost::bind(&SeatHeightAdjuster::executeCB, this, _1), false),
        actionName(name)
        {
            actionServer.start();
            seatHeightPub = nodeHandle.advertise<std_msgs::String>(SEND_TO_ARDUINO, 1000);
            seatHeightSub = nodeHandle.subscribe(READ_FROM_ARDUINO, 1000, &SeatHeightAdjuster::onSeatFeedback, this);
        }
    
    ~SeatHeightAdjuster(void) { }

    void executeCB(const ebms_with_serial::adjustSeatHeightGoalConstPtr &goal) {
        ros::Rate rate(10);

        ROS_INFO("Executing %s, wanted height is %imm",
            actionName.c_str(),
            goal->wantedHeight
        );

        // -------------------------------------------------
        // TODO what happens when the arduino is "resting" ?
        // -------------------------------------------------
        // send the goal height to the microcontroller by publishing it to the pubToArduinoTopic.
        publishNewHeight(goal->wantedHeight);
        
        // Everytime the arduino publishes feedback message with the current height of the seat,
        // onSeatFeedback callback function is executed. This loop checks the latest feedback
        // with a frequency of ros::Rate currently set to 10 times a second.
        // If the action is cancelled by the client, preempted by another goal or the ROS server is not "ok"
        // (For example it is stopped with Ctrl+C), the goal state is set to PREEMPTED and
        // the resulting finalHeight is set to PREEMPTED_CODE and published.
        // If the actuator stalls, the action is aborted.
        // If the action takes too long to execute, the Action Client cancels the current goal
        // which is detected by the Action Server as a preempt request.
        // -------------------------------------------------
        // TODO notify the arduino when the goal has been cancelled.
        // -------------------------------------------------
        // TODO what if scenario:
        /*
            1. currentHeight != goal
            2. publish goal.wantedHeight
            3. currentHeight == goal
            4. rest
            5. change seat height manually
            6. publish the same goal.wantedHeight as before
            Question: do we read that the last feedback message is the same as the new goal so there is no need to move
            or do we get the latest result from the arduino and see that the seat has been manually moved and 
            execute properly?
        */
        while (currentSeatHeight != goal->wantedHeight) {

            if (actionServer.isPreemptRequested() || !ros::ok()) {
                if (feedback.currentHeight <= MAX_SEAT_HEIGHT) {
                    currentSeatHeight = feedback.currentHeight;
                    actionServer.publishFeedback(feedback);
                }

                result.finalHeight = currentSeatHeight;
                actionServer.setPreempted(result, "Action was cancelled or preempted.");
                ROS_INFO("Action was cancelled or preempted. Final seat height: %dmm", result.finalHeight);

                return;
            }

            // ------------------------------------------------------------
            // TODO rename feedback.currentHeight to feedback.currentValue
            // because currentValue can either be a seat height or a status code
            // ------------------------------------------------------------
            if (feedback.currentHeight > MAX_SEAT_HEIGHT) {
                switch (feedback.currentHeight){
                    case STALLED_CODE : {
                        // reset the currentHeight to the last stored seat height and notify the client.
                        feedback.currentHeight = currentSeatHeight;
                        actionServer.publishFeedback(feedback);

                        result.finalHeight = currentSeatHeight;
                        actionServer.setAborted(result, "Actuator has stalled.");
                        ROS_ERROR("The actuator has stalled!");
                        return;
                    }
                }
            }

            currentSeatHeight = feedback.currentHeight;
            ROS_INFO("The current seat height is %dmm", currentSeatHeight);
            actionServer.publishFeedback(feedback);
            rate.sleep();
        }

        if (currentSeatHeight == goal->wantedHeight) {
            result.finalHeight = currentSeatHeight;
            actionServer.setSucceeded(result, "Success");  
            ROS_INFO("%s Succeeded :) the final height is: %dmm", actionName.c_str(), result.finalHeight); 
            return;
        }
        
        result.finalHeight = currentSeatHeight;
        actionServer.setAborted(result, "Failed");  
        ROS_INFO("%s Failed. The final height is: %dmm", actionName.c_str(), result.finalHeight);
    }

    // publish the new wanted height to the arduino topic
    void publishNewHeight(uint8_t goalHeight) {
        std_msgs::String goalMsg;
        char goalHeightToStr[DIGITS_OF_MAX_SEAT_HEIGHT + sizeof(char)];

        // convert int to string
        std::sprintf(goalHeightToStr, "%d", goalHeight);
        goalMsg.data = goalHeightToStr;

        seatHeightPub.publish(goalMsg);
    }

    // https://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
    void onSeatFeedback(const std_msgs::String::ConstPtr& msg) {

        // TODO ignore messages received when action state is not active
        feedback.currentHeight = atoi(msg->data.c_str());
    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "ebmsRosNode");

    SeatHeightAdjuster seatHeightAdjuster("ebmsRosNode");
    ROS_INFO("EBMS Action Server has started");
    ros::spin();

    return 0;
}
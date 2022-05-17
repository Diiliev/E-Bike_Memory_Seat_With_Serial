#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ebms_with_serial/adjustSeatHeightAction.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

class SeatHeightAdjuster {

    protected:
    ros::NodeHandle nodeHandle;
    actionlib::SimpleActionServer<ebms_with_serial::adjustSeatHeightAction> actionServer;
    std::string actionName;
    ros::Publisher seatHeightPub;
    // ros::Subscriber seatHeightSub;
    ebms_with_serial::adjustSeatHeightFeedback feedback;
    ebms_with_serial::adjustSeatHeightResult result;

    public:
    SeatHeightAdjuster(std::string name) :
        actionServer(nodeHandle, name, boost::bind(&SeatHeightAdjuster::executeCB, this, _1), false),
        actionName(name)
        {
            actionServer.start();
            seatHeightPub = nodeHandle.advertise<std_msgs::Bool>("blinkLED", 1000);
            // seatHeightSub = nodeHandle.subscribe("", 1000, onSeatFeedback, this);
        }
    
    ~SeatHeightAdjuster(void) { }

    void executeCB(const ebms_with_serial::adjustSeatHeightGoalConstPtr &goal) {
        ros::Rate rate(10);

        ROS_INFO("Executing %s, wanted height is %i",
            actionName.c_str(),
            goal->wantedHeight
        );

        // send the goal height to the microcontroller by publishing it to the "changeHeight" topic.
        // TODO

        // if(!writeMessageToCan(goal->wantedHeight, CAN_ID_TO_SEND)) {
        //     ROS_ERROR("There was an error writting message to CAN bus.");
        //     actionServer.setAborted();
        //     return;
        // }
        
        // read the current height 
        // -1 means the function has failed to read the message
        // 255 means the actuator has stalled and the operation has failed.
        // In the arduino IDE, a byte typed variable is used to store the height.
        // That is equivalent to an uint8_t variable, i.e values range from 0 to 255.
        // Since we can't send -1 from the arduino, all values above 150
        // (which is the nominal height of the actuator) are reserved as "Special"
        // values and 255 means the actuator has stalled.
        // feedback.currentHeight = readMessageFromCanWithId(CAN_ID_TO_READ);
        // ROS_INFO("feedback.currentHeight=%d",feedback.currentHeight);


        // if there was an error reading message from the CAN bus, abort the action
        // ToDo what happens when reading is successful but the IDs don't match?
        // if (feedback.currentHeight == -1) {
        //     ROS_ERROR("There was an error reading message from CAN bus.");
        //     actionServer.setAborted();
        //     return;
        // }

        // if (feedback.currentHeight == 255) {
        //     ROS_ERROR("The actuator has stalled.");
        //     ROS_WARN("%s not completed", actionName.c_str());
        //     actionServer.setAborted();
        //     return;
        // }

        // keep reading and publishing the feedback messages sent from the microcontroller
        // until the feedback height is equal to the goal wanted height
        // or until the action is preempted
        // or until the microcontroller returns an error with a number higher than 150.
        // TODO add timer condition
        // TODO what if feedback overshoots goal and has to go in the other direction?
        // while (feedback.currentHeight != goal->wantedHeight) {
        //     if (actionServer.isPreemptRequested() || !ros::ok()) {
        //         ROS_WARN("%s was preempted", actionName.c_str());
        //         actionServer.setPreempted();
        //         break;
        //     }
        //     if (feedback.currentHeight == 255) {
        //         ROS_ERROR("The actuator has stalled.");
        //         ROS_WARN("%s not completed", actionName.c_str());
        //         actionServer.setAborted();
        //         return;
        //     }

        //     feedback.currentHeight = readMessageFromCanWithId(CAN_ID_TO_READ);
        //     actionServer.publishFeedback(feedback);
        //     rate.sleep();

            // TODO what happens if the arduino returns error during operation?
            // maybe include status info in the messages.
            // TODO what happens when the arduino is "resting" ?
        // }

        // if the current height is equal to the goal height,
        // set the action status to succeeded.
        // if (feedback.currentHeight == goal->wantedHeight) {
        //     result.finalHeight = feedback.currentHeight;
        //     ROS_INFO("%s Succeeded :)", actionName.c_str());
        //     actionServer.setSucceeded(result);
        // }     
        // else {
        //     ROS_WARN("%s not completed", actionName.c_str());
        //     result.finalHeight = feedback.currentHeight; // TODO not sure if I should set result if the action is aborted. Test this out.
        //     actionServer.setAborted();
        // } 
        

        

        publishNewHeight();

        ROS_INFO("%s Succeeded :)", actionName.c_str());
        actionServer.setSucceeded(result);

        rate.sleep();
    }

    void publishNewHeight() {
        std_msgs::Bool blinkLedMsg;
        blinkLedMsg.data = true;
        seatHeightPub.publish(blinkLedMsg);
    }

    // void onSeatFeedback(const std_msgs::String::ConstPtr& msg) {

    // }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "ebmsRosNode");

    SeatHeightAdjuster seatHeightAdjuster("ebmsRosNode");
    ROS_INFO("EBMS Action Server has started");
    ros::spin();

    return 0;
}
#include <ros.h>
#include <std_msgs/String.h>

#define MOTOR_F 5
#define MOTOR_B 6
#define FEEDBACK A0
#define MAX_WORK_TIME 60000
#define STALL_TIME 4000 // In theory this should be 26ms but in reality the time it takes to move the actuator with 1mm is ~135ms.
#define STALLED_CODE 255
#define RESTING_CODE 254
#define DONE_CODE 253
#define CANCEL_CODE 252
#define RAISE_CODE 251
#define LOWER_CODE 250
#define ACTUATOR_STOP_TIME 100
#define READ_FROM_ROS_TOPIC "newSeatHeight"
#define SEND_TO_ROS_TOPIC "currentSeatHeight"
#define COOLDOWN_TOPIC "cooldownTime"
#define HEIGHT_MAX_DIGITS 3
#define COOLDOWN_TIME_MAX_DIGITS 6

bool actionIsCancelled;

ros::NodeHandle  nh;

// setup ros subscriber. The arduino receives commands from ROS by subscribing to the "changeHeight" topic.
void messageCb( const std_msgs::String& wantedHeight){
  
  byte goalHeight = atoi(wantedHeight.data); 

  // if the action is cancelled stop the actuator immediatelly and raise the flag.
  // This will exit any raising or lowering loop and lead to the resting function.
  if (goalHeight == CANCEL_CODE) {
    stopTheActuator();
    actionIsCancelled = true;
    nh.loginfo("Cancelled.");
  } 

  else {
    actionIsCancelled = false;
    byte currentHeight = getCurrentHeight();
    sendFeedback(currentHeight);

    if (currentHeight == goalHeight) { stopTheActuator(); restTheActuator(0);}
    else if (goalHeight == RAISE_CODE) raiseUntilStopped();
    else if (goalHeight == LOWER_CODE) lowerUntilStopped();
    else if (currentHeight < goalHeight) raiseTheActuator(currentHeight, goalHeight, 0);
    else if (currentHeight > goalHeight) lowerTheActuator(currentHeight, goalHeight, 0);
  }
}

ros::Subscriber<std_msgs::String> rosCommandsTopic(READ_FROM_ROS_TOPIC, &messageCb );

// setup ros publisher. The arduino sends feedback to ROS by publishing it to the "arduinoFeedback" topic.
std_msgs::String feedbackMsg;
ros::Publisher feebackTopic(SEND_TO_ROS_TOPIC, &feedbackMsg);

std_msgs::String cooldownFeedbackMsg;
ros::Publisher cooldownTopic(COOLDOWN_TOPIC, &cooldownFeedbackMsg);



void setup() {

  nh.initNode();
  actionIsCancelled = false;
  nh.subscribe(rosCommandsTopic);
  nh.advertise(feebackTopic);
  nh.advertise(cooldownTopic);
  
  // initialise motor output pins and set to low.
  pinMode(MOTOR_F, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  pinMode(FEEDBACK, INPUT);
  digitalWrite(MOTOR_F, LOW);
  digitalWrite(MOTOR_B, LOW);
}


void loop() {

  /** We spin the node handler to: 
   *  - Receive new messages on the subscribed topic
   *  - Send messages in the queue of the published topic
   *  - Keep synchronized with rosserial. If 15 seconds go by
   *    without calling spinOnce(), we get the following
   *    error message: "Lost sync with device, restarting..."
   */
  nh.spinOnce();
  delay(1);
}

/* 
   *  Return the current height in mm instead of bytes
   *  1023b / 150 = 6.82
   *  1b = 0.147mm
   *  1mm = 6.82b
   *  when casting to byte the value is rounded
   *  and halves are zeroed which means   
   *  100.1 is rounded to 100
   *  100.5 is rounded to 101 
   *  100.9 is rounded to 101
   */
byte getCurrentHeight() {
  
  float analogReading = analogRead(FEEDBACK);
  byte roundedCurrentHeight = round(analogReading / 6.82);
  return roundedCurrentHeight;
}

/**
 * There are four safety mechanisms implemented in the raising function.
 * 1.) Overheat protection by limiting the maximum work time to 1min 
 * 2.) Overheat protection by maintaining 25% duty cycle by having rest time = 4 * work time.
 * 3.) Stall procetion by checking if the previous position feedback value
 *     is the same as the next. If the feedback value hasn't changed in 26ms,
 *     the actuator has stalled. V=5.7mm/s, S=0.147mm, t=0.026s.
 * 4.) Overshoot protection.
 *     If the current height overshots the goal and is now higher than the goal height, 
 *     lower the actuator and also continue tracking the work time of the actuator. 
 *     Otherwise if we have reached the goal height, send the final feedback message and let the actuator rest.
 *     Otherwise, if the actuator has stalled in a position where currentHeight > goalHeight
 *     we have already sent the feedback message that it has stalled.
 *     Now we only need to let the actuator rest without sending any more messages.
 * 
 * If this function is called after previously lowering the actuator,
 * then it has already worked for an "elapsedWorkTime" ammount of time.
 * This time needs to be accounted for in the time it takes to now raise the actuator
 * and the time it takes to rest after the operation has finished.
 * 
 * This function sends feedback messages only if the current height has changed and
 *  it is higher than the previously recorded current height. This is done for several reasons:
 *  - To prevent spamming messages with the same value.
 *  - To prevent us from sending flip-flop values, i.e when the actuator is in a position between
 *    two values and the reading changes from one to the other.
 *  - To prevent us from sending a feedback message where the current height
 *    is equal to the goal height before we have stopped the actuator in that position.
 *    Otherwise we risk notifying ROS that we have reached the goal height, overshoot it and
 *    send new feedback messages, while ROS is already accepting new goals.
 * 
 * TODO add a check if the while loop has ended because it reached the goal height
 *  or because the availableWorkTime was not sufficient to finish the operation.
 *  
 * TODO implement feedback messages with more detailed status such as:
 *  - stalled, timeout, resting, etc.
 *  
 *  -------------------------------------------------------------------------------------------------
 *  TODO find a way to combine raiseTheActuator and lowerTheActuator because it is waisting memory
 *  and it is annoying AND bad practice to Copy/Paste the same changes from one function to another.
 *  -------------------------------------------------------------------------------------------------
 */
void raiseTheActuator (byte currentHeight, byte goalHeight, unsigned long elapsedWorkTime) {
  
  nh.loginfo("Raising...");

  byte currentHeightPreviousValue = currentHeight; // this is in millimeters from 0 to 150
  unsigned long availableWorkTime = MAX_WORK_TIME - elapsedWorkTime; // Calculate the available work time the actuator has left.
  unsigned long startTime = millis(); // note the time we start raising the actuator
  unsigned long stallTimer = millis() + STALL_TIME;

  // begin raising the actuator
  digitalWrite(MOTOR_F, HIGH);
  digitalWrite(MOTOR_B, LOW);
  
  while (currentHeight < goalHeight && millis() < (startTime + availableWorkTime) && !actionIsCancelled) {

    // stall protection 
    if (millis() > stallTimer) {
      stopTheActuator();
      sendFeedback(STALLED_CODE); //This feedback message will abort the ROS action.
      nh.logwarn("Stalled!");
      break;
    }

    currentHeight = getCurrentHeight();

    // send feedback only if the current height has changed and it is not the goal height.
    // After that reset currentHeightPreviousValue
    if (currentHeight > currentHeightPreviousValue && currentHeight < goalHeight) {
      sendFeedback(currentHeight);
      currentHeightPreviousValue = currentHeight;

      // Everytime the current height has changed, reset the stall timer.
      stallTimer = millis() + STALL_TIME; 
    }

    // we spin the node handler to ensure we don't lose synch with rosserial.
    nh.spinOnce();
    
    // without this delay the function doesn't work.
    // I suspect the while loop spins too fast for for the microcontroller to handle.
    delay(10);
  }

  // Stop the actuator, wait for it to become completely still and get the final position
  stopTheActuator();
  delay(ACTUATOR_STOP_TIME);
  currentHeight = getCurrentHeight();

  // Calculate the time spent working by the actuator
  unsigned long workTime = millis() - startTime + elapsedWorkTime;

  // First condition is for Overshoot protection
  // Second condition is the successful case
  // Third condition is in case the actuator has stalled
  if (currentHeight > goalHeight && !actionIsCancelled) {
    lowerTheActuator(currentHeight, goalHeight, workTime);
  }
  else if(currentHeight == goalHeight && !actionIsCancelled){
    sendFeedback(currentHeight);
    nh.loginfo("Done!");
    restTheActuator(workTime);
  }
  else {
    /** 
     * TODO when the actuator has stalled all we do is rest and then accept new goals.
     * Maybe we need a better strategy? Try to un-stall the actuator by moving up/down 
     * or maybe just deny all incomming requests until a human has fixed the actuator
     * and restarted the microcontroller.
    */
    sendFeedback(currentHeight);
    restTheActuator(workTime);
  }
}


/**
 * There are four safety mechanisms implemented in the lower function.
 * 1.) Overheat protection by limiting the maximum work time to 1min 
 * 2.) Overheat protection by maintaining 25% duty cycle by having rest time = 4 * work time.
 * 3.) Stall procetion by checking if the previous position feedback value
 *     is the same as the next. If the feedback value hasn't changed in 26ms,
 *     the actuator has stalled. V=5.7mm/s, S=0.147mm, t=0.026s.
 * 4.) Overshoot protection.
 *     If the current height overshots the goal and is now lower than the goal height, 
 *     raise the actuator and also continue tracking the work time of the actuator. 
 *     Otherwise if we have reached the goal height, send the final feedback message and let the actuator rest.
 *     Otherwise, if the actuator has stalled in a position where currentHeight > goalHeight
 *     we have already sent the feedback message that it has stalled.
 *     Now we only need to let the actuator rest without sending any more messages.
 * 
 * If this function is called after previously raising the actuator,
 * then it has already worked for an "elapsedWorkTime" ammount of time.
 * This time needs to be accounted for in the time it takes to now lower the actuator
 * and the time it takes to rest after the operation has finished.
 * 
 * This function sends feedback messages only if the current height has changed and
 *  it is lower than the previously recorded current height. This is done for several reasons:
 *  - To prevent spamming messages with the same value.
 *  - To prevent us from sending flip-flop values, i.e when the actuator is in a position between
 *    two values and the reading changes from one to the other.
 *  - To prevent us from sending a feedback message where the current height
 *    is equal to the goal height before we have stopped the actuator in that position.
 *    Otherwise we risk notifying ROS that we have reached the goal height, overshoot it and
 *    send new feedback messages, while ROS is already accepting new goals.
 * 
 * TODO add a check if the while loop has ended because it reached the goal height
 *  or because the availableWorkTime was not sufficient to finish the operation.
 *  
 * TODO implement feedback messages with more detailed status such as:
 *  - stalled, timeout, resting, etc.
 *  
 *  -------------------------------------------------------------------------------------------------
 *  TODO find a way to combine raiseTheActuator and lowerTheActuator because it is waising memory
 *  and it is annoying AND bad practice to Copy/Paste the same changes from one function to another.
 *  -------------------------------------------------------------------------------------------------
 *  
 */
void lowerTheActuator (byte currentHeight, byte goalHeight, unsigned long elapsedWorkTime) {
  
  nh.loginfo("Lowering...");
  
  byte currentHeightPreviousValue = currentHeight; // this is in millimeters from 0 to 150
  unsigned long availableWorkTime = MAX_WORK_TIME - elapsedWorkTime; // Calculate the available work time the actuator has left.
  unsigned long startTime = millis(); // note the time we start raising the actuator
  unsigned long stallTimer = millis() + STALL_TIME;
  
  // begin lowering the actuator
  digitalWrite(MOTOR_F, LOW);
  digitalWrite(MOTOR_B, HIGH);

  while (currentHeight > goalHeight && millis() < (startTime + availableWorkTime) && !actionIsCancelled) {

    // stall protection
    if (millis() > stallTimer) {
      stopTheActuator();
      sendFeedback(STALLED_CODE); //This feedback message will abort the ROS action.
      nh.logwarn("Stalled!");
      break;
    }

    currentHeight = getCurrentHeight();
    
    // send feedback only if the current height has changed and it is not the goal height.
    // After that reset currentHeightPreviousValue and stallTimer
    if (currentHeight < currentHeightPreviousValue &&  currentHeight > goalHeight) {
      sendFeedback(currentHeight);
      currentHeightPreviousValue = currentHeight;

      // Everytime the current height has changed, reset the stall timer.
      stallTimer = millis() + STALL_TIME;
    }

    // we spin the node handler to ensure we don't lose synch with rosserial.
    nh.spinOnce();

    // without this delay the function doesn't work.
    // I suspect the loop spins too fast for for the microcontroller to handle.
    // TODO figure out why the fuction does not work without this delay.
    delay(10); 
  }

  // Stop the actuator, wait for it to become completely still and get the final position
  stopTheActuator();
  delay(ACTUATOR_STOP_TIME);
  currentHeight = getCurrentHeight();

  // Calculate the time spent working by the actuator
  unsigned long workTime = millis() - startTime + elapsedWorkTime;

  // First condition is for Overshoot protection
  // Second condition is the successful case
  // Third condition is in case the actuator has stalled
  if (currentHeight < goalHeight && !actionIsCancelled) {
    raiseTheActuator(currentHeight, goalHeight, workTime);
  }
  else if(currentHeight == goalHeight && !actionIsCancelled){ 
    sendFeedback(currentHeight);
    nh.loginfo("Done!");
    restTheActuator(workTime);
  }
  else {
    sendFeedback(currentHeight);
    restTheActuator(workTime);
  }
}


/**
 * When the user wants to raise the seat to an unspecified height, they
 * will hold down the raise button until they are satisfied with the seat height. 
 * When the button is pressed it publishes true, when released false.
 * 
 * The Action Client makes sure that only one request is being serviced at any given moment.
 * Button presses while the actuator is moving, or while it is in cooldown will be ignored.
 * 
 * The Action Server publishes the special codes: 
 * - RAISE_CODE when the raise button is pressed
 * - LOWER_CODE when the lower button is pressed 
 * - CANCEL_CODE when either button is released.
 * 
 * This function will raise the actuator until it receives STOP_CODE, until the actuator
 * stalls or until the timeout is reached. 
 */
void raiseUntilStopped() {
  nh.loginfo("Rаisе...");

  byte currentHeight = getCurrentHeight();
  byte currentHeightPreviousValue = currentHeight;
  unsigned long startTime = millis(); // note the time we start raising the actuator
  unsigned long stallTimer = millis() + STALL_TIME;

  // begin raising the actuator
  digitalWrite(MOTOR_F, HIGH);
  digitalWrite(MOTOR_B, LOW);
  
  while (millis() < (startTime + MAX_WORK_TIME) && !actionIsCancelled) {

    // stall protection 
    if (millis() > stallTimer) {
      stopTheActuator();
      sendFeedback(STALLED_CODE); //This feedback message will abort the ROS action.
      nh.logwarn("Stalled!");
      break;
    }

    currentHeight = getCurrentHeight();

    // send feedback only if the current height has changed.
    // Then reset currentHeightPreviousValue and the stall timer.
    if (currentHeight > currentHeightPreviousValue) {
      sendFeedback(currentHeight);
      currentHeightPreviousValue = currentHeight;
      stallTimer = millis() + STALL_TIME;
    }

    // we spin the node handler to ensure we don't lose synch with rosserial.
    // and to receive new messages from the subscriber.
    nh.spinOnce();
    
    // without this delay the function doesn't work.
    // I suspect the while loop spins too fast for for the microcontroller to handle.
    delay(10);
  }

  // Stop the actuator, wait for it to become completely still and get the final position
  stopTheActuator();
  delay(ACTUATOR_STOP_TIME);
  currentHeight = getCurrentHeight();

  // Calculate the time spent working by the actuator
  unsigned long workTime = millis() - startTime;

  sendFeedback(currentHeight);
  nh.loginfo("Done!");
  restTheActuator(workTime);
}

/**
 * When the user wants to lower the seat to an unspecified height, they
 * will hold down the lower button until they are satisfied with the seat height. 
 * When the button is pressed it publishes true, when released false.
 * 
 * The Action Client makes sure that only one request is being serviced at any given moment.
 * Button presses while the actuator is moving, or while it is in cooldown will be ignored.
 * 
 * The Action Server publishes the special codes: 
 * - RAISE_CODE when the raise button is pressed
 * - LOWER_CODE when the lower button is pressed 
 * - CANCEL_CODE when either button is released.
 * 
 * This function will lower the actuator until it receives STOP_CODE, until the actuator
 * stalls or until the timeout is reached. 
 */
void lowerUntilStopped() {
  nh.loginfo("Lower...");

  byte currentHeight = getCurrentHeight();
  byte currentHeightPreviousValue = currentHeight;
  unsigned long startTime = millis(); // note the time we start raising the actuator
  unsigned long stallTimer = millis() + STALL_TIME;

  // begin raising the actuator
  digitalWrite(MOTOR_F, LOW);
  digitalWrite(MOTOR_B, HIGH);
  
  while (millis() < (startTime + MAX_WORK_TIME) && !actionIsCancelled) {

    // stall protection 
    if (millis() > stallTimer) {
      stopTheActuator();
      sendFeedback(STALLED_CODE); //This feedback message will abort the ROS action.
      nh.logwarn("Stalled!");
      break;
    }

    currentHeight = getCurrentHeight();

    // send feedback only if the current height has changed.
    // Then reset currentHeightPreviousValue and the stall timer.
    if (currentHeight < currentHeightPreviousValue) {
      sendFeedback(currentHeight);
      currentHeightPreviousValue = currentHeight;
      stallTimer = millis() + STALL_TIME;
    }

    // we spin the node handler to ensure we don't lose synch with rosserial.
    // and to receive new messages from the subscriber.
    nh.spinOnce();
    
    // without this delay the function doesn't work.
    // I suspect the while loop spins too fast for for the microcontroller to handle.
    delay(10);
  }

  // Stop the actuator, wait for it to become completely still and get the final position
  stopTheActuator();
  delay(ACTUATOR_STOP_TIME);
  currentHeight = getCurrentHeight();

  // Calculate the time spent working by the actuator
  unsigned long workTime = millis() - startTime;

  sendFeedback(currentHeight);
  nh.loginfo("Done!");
  restTheActuator(workTime);
}


/**
 * Stop the actuator immediatelly.
 */
void stopTheActuator() {
  
  digitalWrite(MOTOR_F, LOW);
  digitalWrite(MOTOR_B, LOW);
}

/**
 * Calculate the appropriate ammount of rest time from the given work time.
 * Given that the duty cycle of the actuator is 25%, that means it must rest for
 * 4 times the ammount of time spent working.
 * Stop the execution of the program for "restTime" ammount of time
 * and print "restTime" to the Serial monitor for debugging purposes.
 * 
 * TODO replace this with a function which checks for incoming messages and
 *      returns the current height with status 'C' for cooldown. 
 *      Next version of this function should also return the time left to rest
 * 
 * TODO send feedback to ROS when resting is complete.
 * 
 * TODO send the time it will take to rest to the ROS Action Server so it can
 * notify the Client.
 * 
 * TODo log time left for resting on every spin
 * 
 * TODO new goals should be able to interrupt this resting period if the
 * duty cycle of the actuator allows for that to happen. For example 3 seconds
 * of work require 12 seconds of rest which is very annoying if you only want
 * to move the seat one last time for another few seconds.
 * We need to calculate how long the worst case scenario will take and
 * determine weather the actuator can move there without burning out.
 */
void restTheActuator(unsigned long workTime) {

  nh.loginfo("Resting...");
  sendFeedback(RESTING_CODE);
  
  unsigned long cooldownTime = 4 * workTime;
  unsigned long now = millis();
  unsigned long endTime = now + cooldownTime;
  
  while(now < endTime) {
    sendCooldownFeedback(endTime - now);
    nh.spinOnce();
    delay(100);
    now = millis();
  }

  sendFeedback(DONE_CODE);
  nh.loginfo("Ready");
}

/**
 *  Send feedback to the ROS Action Server.
 * 
 *  One char is used for every digit of the feedback number,
 *  which can have a value between 0 and 255,
 *  and one char is added for the null terminator.
 *  Numbers between 0 and 150 describe the current seat height
 *  in millimeters. Numbers between 151 and 255 are used to
 *  describe other conditions or states such as:
 *  "the actuator has stalled", or "the microcontroller is
 *  resting", etc.
 *  
 *  byte [0] stores the digit for the hundreds
 *  byte [1] stores the digit for the tens
 *  byte [2] stores the digit for the units
 *  byte [3] stores the Null-Terminator '\0'
 */
void sendFeedback(byte feedback) {

  char feedbackToStr[HEIGHT_MAX_DIGITS + sizeof(char)]; 
  itoa (feedback, feedbackToStr, 10); // convert the number into a character array
  feedbackMsg.data = feedbackToStr; // feed the std_msgs::String data parameter
  feebackTopic.publish(&feedbackMsg);
}

/**
 * Send the time in seconds left until the cooldown has finished.
 */
void sendCooldownFeedback(unsigned long cooldownTime) {
  char cooldownTimeToStr[COOLDOWN_TIME_MAX_DIGITS + sizeof(char)]; 
  ultoa (cooldownTime, cooldownTimeToStr, 10); // convert unsigned long into a character array
  cooldownFeedbackMsg.data = cooldownTimeToStr; // feed the std_msgs::String data parameter
  cooldownTopic.publish(&cooldownFeedbackMsg);
}

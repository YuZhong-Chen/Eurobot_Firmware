#ifndef ROS_MAINPP_H_
#define ROS_MAINPP_H_

#include "geometry_msgs/Twist.h"

#define ROS_CAR_PUB_FREQUENCY 10

namespace ROS {

void setup();
void loop();
void init();

// Publisher
void PubCarVnow();

// Subscriber
void GoalVel_CB(const geometry_msgs::Twist &msg);

}

#endif /* ROS_MAINPP_H_ */

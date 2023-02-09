#include "stm32h7xx_hal.h"

#include "ROS_mainpp.h"
#include "ros.h"

#include "Omni.h"

// For ROS::loop
extern TIM_HandleTypeDef htim7;

ros::NodeHandle nh;

geometry_msgs::Twist CarVnow;
static CAR_INFO NowCarInfo;

ros::Subscriber<geometry_msgs::Twist> CarVelSub("Omni_Vgoal", ROS::GoalVel_CB);

ros::Publisher CarVelPub("Omni_Vnow", &CarVnow);

void ROS::GoalVel_CB(const geometry_msgs::Twist &msg) {
	omni.SetGoalCarInfo(msg.linear.x, msg.linear.y, msg.angular.z);
}

void ROS::setup() {
	nh.initNode();

	nh.subscribe(CarVelSub);
	nh.advertise(CarVelPub);

	HAL_TIM_Base_Start_IT(&htim7);
}

void ROS::loop() {
	nh.spinOnce();
}

void ROS::PubCarVnow() {
	NowCarInfo = omni.GetNowCarInfo();

	CarVnow.linear.x = NowCarInfo.Vx;
	CarVnow.linear.y = NowCarInfo.Vy;
	CarVnow.angular.z = NowCarInfo.Omega;

	CarVelPub.publish(&CarVnow);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->reset_rbuf();
}

void ROS::init(void) {
//	nh.getHardware()->init();
}

#include "Omni.h"

#include "DC_Motor.h"
#include "geometry_msgs/Twist.h"

// Encoder
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

// Motor PWM
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim15;

Omni omni;

double CAR_RADIUS = 0.134163;

Omni::Omni() {
}

void Omni::Init() {
	this->motors[0].Init(0, &htim2, 3.7, 471.0);
	this->motors[1].Init(1, &htim5, 3.7, 471.0);
	this->motors[2].Init(2, &htim3, 3.7, 471.0);
	this->motors[3].Init(3, &htim4, 3.7, 471.0);

	SetCarRadius(CAR_RADIUS);

	DC_Motor::Init();
}

void Omni::UpdateCarLocation() {
	double m[4];
	for (int i = 0; i < 4; i++) {
		m[i] = this->motors[i].MoveDis() / 1000.0;
	}
	NowCarLocation.Vx += (m[3] - m[1]) / 2.0;
	NowCarLocation.Vy += (m[0] - m[2]) / 2.0;
	NowCarLocation.Omega += (m[0] + m[1] + m[2] + m[3]) / (4.0 * CarRadius);
}

void Omni::UpdateNowCarInfo() {
	// Get each motors' Vnow
	// Unit : m/s , rad/s
	this->UpdateMotorVnow();

	NowCarInfo.Vx = (motors[3].GetVnow() - motors[1].GetVnow()) / 2.0;
	NowCarInfo.Vy = (motors[0].GetVnow() - motors[2].GetVnow()) / 2.0;
	NowCarInfo.Omega = (motors[0].GetVnow() + motors[1].GetVnow() + motors[2].GetVnow() + motors[3].GetVnow()) / (CarRadius * 4.0);
}

// Set all motors' velocity base on Car Vgoal.
void Omni::SetGoalCarInfo(double Vx, double Vy, double Omega) {
	this->GoalCarInfo.Vx = Vx;
	this->GoalCarInfo.Vy = Vy;
	this->GoalCarInfo.Omega = Omega;

	SetMotorVgoal();
}

void Omni::Update_PID() {
	for (int i = 0; i < 4; i++) {
		this->motors[i].UpdatePI();
	}
}

void Omni::UpdateMotorVnow() {
	for (int i = 0; i < 4; i++) {
		this->motors[i].UpdateVnow();
	}
}

// TODO : Check for the DIR
void Omni::Move() {
	// DIR
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, (motors[0].u > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (motors[1].u > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, (motors[2].u > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (motors[3].u > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);

	// PWM
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, int(fabs(motors[0].u) * MOTOR_PWM_PULSE));
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, int(fabs(motors[1].u) * MOTOR_PWM_PULSE));
	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, int(fabs(motors[2].u) * MOTOR_PWM_PULSE));
	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, int(fabs(motors[3].u) * MOTOR_PWM_PULSE));
}

void Omni::SetMotorVgoal() {
	// Unit : m/s
	this->motors[0].SetVgoal(GoalCarInfo.Vy + CarRadius * GoalCarInfo.Omega);
	this->motors[1].SetVgoal(-GoalCarInfo.Vx + CarRadius * GoalCarInfo.Omega);
	this->motors[2].SetVgoal(-GoalCarInfo.Vy + CarRadius * GoalCarInfo.Omega);
	this->motors[3].SetVgoal(GoalCarInfo.Vx + CarRadius * GoalCarInfo.Omega);
}

CAR_INFO Omni::GetNowCarInfo() {
	return NowCarInfo;
}

CAR_INFO Omni::GetNowCarLocation() {
	return NowCarLocation;
}

void Omni::SetCarRadius(double CarRadius) {
	this->CarRadius = CarRadius;
}


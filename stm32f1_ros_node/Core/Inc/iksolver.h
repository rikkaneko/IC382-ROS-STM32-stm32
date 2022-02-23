/***
@Author: Vincent Chan
@About: Inverse Kinematic Solver
***/
#include "ros_main.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <utility>
/*** User defined variables ***/

// Resultant duty cycles of each motor
std::pair<int,int> twin_motors_duty_cycle;
std::pair<int,int> twin_motors_rotations;



// Wheel physical parameters in meters
double wheel_seperation = 0.47 * 10;
double max_velocity = 15;

// Wheel speed in m/s and rad/s
double motor1_v = 0, motor2_v = 0;
int motor1_rev = 0, motor2_rev = 0;

/*** Function Prototypes ***/
void twin_drive_ik(double linear_x,double angular_Z);
int clipping(int motor_pwm,int min_value,int max_value);


uint32_t __abs(float val) { return val>0? val: -val; }

/*** Function Definition ***/
// 1. Transform velocity to duty cycles
void twin_drive_ik(double linear_x,double angular_z)
{
	// self rolation
	float vl = linear_x - wheel_seperation * angular_z / 2;
	float vr = linear_x + wheel_seperation * angular_z / 2;
	
	// Output 
	twin_motors_duty_cycle.first = __abs((vl/max_velocity)*3200);
	twin_motors_duty_cycle.second = __abs((vr/max_velocity)*3200);
	
	twin_motors_rotations.first = (vl >= 0)? ((vl > 0)? 0: 2): 1;
	twin_motors_rotations.second =  (vr >= 0)? ((vr > 0)? 0: 2): 1;
}

// 2. Ensure the PWM is within reasonable range
int clipping(int motor_pwm,int min_value,int max_value)
{
	int output_pwm = 0;
	motor_pwm > max_value ? output_pwm = max_value : output_pwm=motor_pwm;
	motor_pwm < min_value ? output_pwm = min_value : output_pwm=motor_pwm;
	return output_pwm;
}
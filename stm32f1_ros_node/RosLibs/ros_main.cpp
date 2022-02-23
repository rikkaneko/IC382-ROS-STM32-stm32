#include "ros_main.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include "motor_manage.h"
#include "iksolver.h"
// Extern peripherals
extern IWDG_HandleTypeDef hiwdg;

// ROS Node Handler
ros::NodeHandle nh;

// ROS stm32_status Publisher
char stm32_cmd_vel_ready_msg[]= "STM32_cmd_vel_OK";
std_msgs::String str_msg;
ros::Publisher stm32_status_pub("/stm32/system_up", &str_msg);

// ROS motor encoders Publisher
long encoder_value_m1 = 0;
long encoder_value_m2 = 0;
geometry_msgs::Vector3 encoders_data;
ros::Publisher encoder_pub("/stm32/encoders",&encoders_data);


// Callback of cmd_vel
void cmd_vel_msg(const geometry_msgs::Twist& msg)
{
	double linear_x = msg.linear.x;
	double angular_z = msg.angular.z;
	// Transform cmd_vel to duty cycles
	twin_drive_ik(linear_x,angular_z);
	
	// Execute motor control
	double motor1_pwm = twin_motors_duty_cycle.first;
	double motor2_pwm = twin_motors_duty_cycle.second;
	motor_controller(MOTOR1,(MOTOR_DIRECTION) twin_motors_rotations.first,motor1_pwm,max_register_value);
	motor_controller(MOTOR2,(MOTOR_DIRECTION) twin_motors_rotations.second,motor2_pwm,max_register_value);
	// FILL IT IN BY YOURSELF (╯ ͡❛ ‿ ͡❛)╯┻━┻

	// LED Indicators
	HAL_GPIO_TogglePin(EXTENSION_LED4_GPIO_Port,EXTENSION_LED4_Pin);
	HAL_GPIO_TogglePin(EXTENSION_LED3_GPIO_Port,EXTENSION_LED3_Pin);
	HAL_GPIO_TogglePin(EXTENSION_LED2_GPIO_Port,EXTENSION_LED2_Pin);
	HAL_GPIO_TogglePin(EXTENSION_LED1_GPIO_Port,EXTENSION_LED1_Pin);
}


// ROS cmd_vel Subscriber
ros::Subscriber<geometry_msgs::Twist> velocity_sub("/cmd_vel", &cmd_vel_msg);

void setup(void)
{
	
	// Turn Off ALL Core Board LEDs (PULLED-HIGH)
	HAL_GPIO_WritePin(CORE_LED0_GPIO_Port,CORE_LED0_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CORE_LED1_GPIO_Port,CORE_LED1_Pin,GPIO_PIN_SET);
	
	// Turn On ALL Extension Board LEs (PULLED-HIGH)
	HAL_GPIO_WritePin(EXTENSION_LED4_GPIO_Port,EXTENSION_LED4_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EXTENSION_LED3_GPIO_Port,EXTENSION_LED3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EXTENSION_LED2_GPIO_Port,EXTENSION_LED2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EXTENSION_LED1_GPIO_Port,EXTENSION_LED1_Pin,GPIO_PIN_RESET);
	
	// Initialize ROS node
	nh.initNode();
	
	// Call ROS Master to keep a registry of this publisher
	nh.advertise(stm32_status_pub);
	nh.advertise(encoder_pub);
	
	// Subscribe cmd_vel
	nh.subscribe(velocity_sub);
	
	// Enable all motor related peripherals (ALL CORE LEDs should be ON)
	motor_peripherals_init(MOTOR1);
	motor_peripherals_init(MOTOR2);
	// FILL IT IN BY YOURSELF (╯ ͡❛ ‿ ͡❛)╯┻━┻
}


void loop(void)
{
  // Life LED	
	HAL_GPIO_TogglePin(CORE_LED0_GPIO_Port,CORE_LED0_Pin);
	HAL_GPIO_TogglePin(CORE_LED1_GPIO_Port,CORE_LED1_Pin);	
	
	// Get encoders values and publish
	// FILL IT IN BY YOURSELF (╯ ͡❛ ‿ ͡❛)╯┻━┻
	
	// Publish encoder vectors
	encoders_data.x = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim1));
	encoders_data.y = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim3));
	encoder_pub.publish(&encoders_data);

	// STM32 Virtual COM Port (VCP)
	unsigned char buffer[]= {"STM32 USB COM Port OK!\r\n"};
	CDC_Transmit_FS(buffer,sizeof(buffer));

	nh.spinOnce();
	HAL_Delay(200);
	
	// Reset IWDG within 13.104s.
	HAL_IWDG_Refresh(&hiwdg);

}

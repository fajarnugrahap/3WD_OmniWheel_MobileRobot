/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include <string.h>
#include "i2c-lcd.h"
#include "user.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim20;

/* Definitions for task_1 */
osThreadId_t task_1Handle;
const osThreadAttr_t task_1_attributes = {
  .name = "task_1",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for task_2 */
osThreadId_t task_2Handle;
const osThreadAttr_t task_2_attributes = {
  .name = "task_2",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for task_3 */
osThreadId_t task_3Handle;
const osThreadAttr_t task_3_attributes = {
  .name = "task_3",
  .priority = (osPriority_t) osPriorityBelowNormal1,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
uint32_t millis_tick;

char x_buffer[32], y_buffer[32], speed_buffer[32];

int16_t targetX = 0, targetY = 0, maxSpeed = 0;

char rx_buffer[11];
volatile uint8_t data_received = 0;

int count = 0;
int datasend = 0;
int datareceive = 0;

int B1;

uint32_t x = 0;

uint8_t RxData[5];
uint8_t TxData[40];

int PBleft;
int PBright;
int PBup;
int PBdown;

uint16_t menu = 0;

char *data = "Hello\n";

char *tx = "12";

//*********Odometry variables**********//
int32_t ma, mb, mc;
int32_t my, mx, mtheta;
int32_t na, nb, nc;

int kp_x = 2;
int kp_y = 2;

int error_x, error_y, error_theta;
int error_pos1, error_pos2, error_pos3;
float kp = 2.0;
int p_x, p_y, p_theta;
int p1, p2, p3;

//******Speed Setting variables*******//
float pwm_mtr1, pwm_mtr2, pwm_mtr3;
int d1,d2,d3;

int pwm1,pwm2,pwm3;
float a,b,c,
	  d,e,f,
	  g,h,i;
float a_inverse,b_inverse,c_inverse,
      d_inverse,e_inverse,f_inverse,
      g_inverse,h_inverse,i_inverse;
float det;
float L = 26.0;
float r = 15.0;

//*********Motor Variables**********//
int32_t tim1_cnt = 0, direction1= 0;
int32_t tim2_cnt = 0, direction2= 0;
int32_t tim8_cnt = 0, direction3= 0;

int32_t position1 = 0;
int32_t position2 = 0;
int32_t position3 = 0;

int32_t targetpos1 = 0;
int32_t targetpos2 = 0;
int32_t targetpos3 = 0;

int32_t counter1 = 0, counter2 = 0, counter8 = 0;
int32_t pengali1 = 0, pengali2 = 0, pengali8 = 0;

volatile uint32_t rise1= 0;
volatile float freq1 = 0;

volatile uint32_t rise2= 0;
volatile float freq2 = 0;

volatile uint32_t rise3= 0;
volatile float freq3 = 0;

float current_rpm1 = 0.0;
float current_rpm2 = 0.0;
float current_rpm3 = 0.0;

int sp1; //in RPM
int sp2; //in RPM
int sp3; //in RPM

float kp1 = 10;
float ki1 = 0.1;
float kd1 = 5;
int error1 = 0;
int last_error1 = 0, delta_error1  =  0, total_error1 = 0;
int sum_error1 = 0;
float motorSpeed1 = 0;
uint32_t  lastTime1 = 0;

float kp2 = 10;
float ki2 = 0.1;
float kd2 = 5;
int error2 = 0;
int last_error2 = 0, delta_error2  =  0, total_error2 = 0;
int sum_error2 = 0;
float motorSpeed2 = 0;
uint32_t  lastTime2 = 0;

float kp3 = 10;
float ki3 = 0.1;
float kd3 = 5;
int error3 = 0;
int last_error3 = 0, delta_error3  =  0, total_error3 = 0;
int sum_error3 = 0;
float motorSpeed3 = 0;
uint32_t  lastTime3 = 0;

int T = 30; //sample time in ms
int max_control = 1050; //PG45
int min_control = 0;

int complete = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM20_Init(void);
static void MX_I2C1_Init(void);
void task_1_function(void *argument);
void task_2_function(void *argument);
void task_3_function(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
  // Implement your write code here, this is used by puts and print for example
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

void get_millis(void) {
	millis_tick = HAL_GetTick();
}

void led_builtIn_blink(void){
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_Delay(500);
}

float deg2rad(float d)
{
   return d * 3.14 / 180.0 ;
}

float degtorad(float d){
	return d * 3.14 / 180.0 ;
}

void lcd_startUp(void){
	lcd_init();

	lcd_send_cmd (0x80|0x02);
	lcd_send_string("HOLONOMIC ROBOT");

	lcd_send_cmd (0x80|0x43);
	lcd_send_string("BASED ODOMETRY");

	HAL_Delay(1000);

	lcd_send_cmd (0x80|0x1D);
	lcd_send_string("by");

	lcd_send_cmd (0x80|0x57);
	lcd_send_string("fajarnugrahap");

	HAL_Delay(2000);

	lcd_clear();
}

void lcd_display_1 (void){
	PBleft = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	PBright = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
	PBup = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	PBdown = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	B1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

	if (menu == 0){
		lcd_send_cmd (0x80|0x42);
		lcd_send_string("Press Any Button");

		lcd_send_cmd (0x80|0x1A);
		lcd_send_string("to Begin");

		if (PBright == 0 || PBleft == 0 || PBup == 0 || PBdown == 0){
			HAL_Delay(250);
			lcd_clear();
			PBright = 1;
			PBleft = 1;
			PBup = 1;
			PBdown = 1;
			menu = 1;
		}
	}

	if (menu == 1) {
		lcd_send_cmd (0x80|0x00);
		lcd_send_string("Select Mode");

		lcd_send_cmd (0x80|0x40);
		lcd_send_string(">Remote");

		lcd_send_cmd (0x80|0x14);
		lcd_send_string(" Autonomous");

		if (PBdown == 0){
			HAL_Delay(250);
			lcd_clear();
			PBdown = 1;
			menu = 2;
		}

		if (PBright == 0){
			HAL_Delay(250);
			lcd_clear();
			PBright = 1;
			menu = 11;
		}
	}

	if (menu == 2) {
		lcd_send_cmd (0x80|0x00);
		lcd_send_string("Select Mode");

		lcd_send_cmd (0x80|0x40);
		lcd_send_string(" Remote");

		lcd_send_cmd (0x80|0x14);
		lcd_send_string(">Autonomous");

		if (PBup == 0){
			HAL_Delay(250);
			lcd_clear();
			PBup = 1;
			menu = 1;
		}

		if (PBright == 0){
			HAL_Delay(250);
			lcd_clear();
			PBright = 1;
			menu = 21;
		}

	}

	if (menu == 11 ){
		lcd_send_cmd (0x80|0x40);
		lcd_send_string("Wait to Connect");
		if(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 255 && rx_buffer[4] == 255 && rx_buffer[5] == 255){
			HAL_Delay(50);
			lcd_clear();
			menu = 111;
		}
	}

	if (menu == 111){
		lcd_send_cmd (0x80|0x40);
		lcd_send_string("Connected!");
		HAL_Delay(1000);

		lcd_send_cmd (0x80|0x00);
		lcd_send_string("Pos X : ");

		sprintf(x_buffer, "%li", mb);
		lcd_send_cmd (0x80|0x0B);
		lcd_send_string(x_buffer);

		lcd_send_cmd (0x80|0x40);
		lcd_send_string("Pos Y : ");

		sprintf(y_buffer, "%li", ma);
		lcd_send_cmd (0x80|0x4B);
		lcd_send_string(y_buffer);

		remote_mode1();

		if(B1 == 1){
			ma = 0;
			mb = 0;
			tim1_cnt = 0;
			tim2_cnt = 0;
			tim8_cnt = 0;
		}
		else{
		}
	}

	if (menu == 21){
		lcd_send_cmd (0x80|0x00);
		lcd_send_string("Target : ");

		sprintf(x_buffer, "%4i", targetX);
		lcd_send_cmd (0x80|0x09);
		lcd_send_string(x_buffer);

		lcd_send_cmd (0x80|0x0E);
		lcd_send_string(",");

		sprintf(y_buffer, "%4i", targetY);
		lcd_send_cmd (0x80|0x0F);
		lcd_send_string(y_buffer);

		lcd_send_cmd (0x80|0x40);
		lcd_send_string(">Set Target X");

		lcd_send_cmd (0x80|0x14);
		lcd_send_string(" Set Target Y");

		lcd_send_cmd (0x80|0x54);
		lcd_send_string(" Start");

		if (PBdown == 0){
			HAL_Delay(250);
			lcd_clear();
			PBdown = 1;
			menu = 22;
		}

		if (PBright == 0){
			HAL_Delay(250);
			lcd_clear();
			PBright = 1;
			menu = 211;
		}
	}

	if (menu == 22){
		lcd_send_cmd (0x80|0x00);
		lcd_send_string("Target : ");

		sprintf(x_buffer, "%4i", targetX);
		lcd_send_cmd (0x80|0x09);
		lcd_send_string(x_buffer);

		lcd_send_cmd (0x80|0x0E);
		lcd_send_string(",");

		sprintf(y_buffer, "%4i", targetY);
		lcd_send_cmd (0x80|0x0F);
		lcd_send_string(y_buffer);

		lcd_send_cmd (0x80|0x40);
		lcd_send_string(" Set Target X");

		lcd_send_cmd (0x80|0x14);
		lcd_send_string(">Set Target Y");

		lcd_send_cmd (0x80|0x54);
		lcd_send_string(" Start");

		if (PBdown == 0){
			HAL_Delay(250);
			lcd_clear();
			PBdown = 1;
			menu = 24;
		}

		if (PBup == 0){
			HAL_Delay(250);
			lcd_clear();
			PBup = 1;
			menu = 21;
		}

		if (PBright == 0){
			HAL_Delay(250);
			lcd_clear();
			PBright = 1;
			menu = 221;
		}
	}

	if (menu == 24){
		lcd_send_cmd (0x80|0x00);
		lcd_send_string("Target : ");

		sprintf(x_buffer, "%4i", targetX);
		lcd_send_cmd (0x80|0x09);
		lcd_send_string(x_buffer);

		lcd_send_cmd (0x80|0x0E);
		lcd_send_string(",");

		sprintf(y_buffer, "%4i", targetY);
		lcd_send_cmd (0x80|0x0F);
		lcd_send_string(y_buffer);

		lcd_send_cmd (0x80|0x40);
		lcd_send_string(" Set Target X");

		lcd_send_cmd (0x80|0x14);
		lcd_send_string(" Set Target Y");

		lcd_send_cmd (0x80|0x54);
		lcd_send_string(">Start");

		if (PBup == 0){
			HAL_Delay(250);
			lcd_clear();
			PBup = 1;
			menu = 22;
		}

		if (PBright == 0){
			HAL_Delay(250);
			lcd_clear();
			PBright = 1;
			menu = 241;
		}
	}

	if (menu == 211){
			lcd_send_cmd (0x80|0x42);
			lcd_send_string("Target X");
			if (PBup == 0){
				HAL_Delay(250);
				PBup = 1;
				targetX = targetX + 100;
			}
			if (PBdown == 0){
				HAL_Delay(250);
				PBdown = 1;
				targetX = targetX - 100;
			}
			if (PBleft == 0){
				HAL_Delay(250);
				PBleft = 1;
				menu = 21;
			}
			sprintf(x_buffer, "%4i", targetX);
			lcd_send_cmd (0x80|0x16);
			lcd_send_string(x_buffer);
			lcd_send_cmd (0x80|0x1B);
			lcd_send_string("cm");
		}

		if (menu == 221){
			lcd_send_cmd (0x80|0x42);
			lcd_send_string("Target Y");
			if (PBup == 0){
				HAL_Delay(250);
				PBup = 1;
				targetY = targetY + 100;
			}
			if (PBdown == 0){
				HAL_Delay(250);
				PBdown = 1;
				targetY = targetY - 100;
			}
			if (PBleft == 0){
				HAL_Delay(250);
				PBleft = 1;
				menu = 22;
			}
			sprintf(y_buffer, "%4i", targetY);
			lcd_send_cmd (0x80|0x16);
			lcd_send_string(y_buffer);
			lcd_send_cmd (0x80|0x1B);
			lcd_send_string("cm");
		}

		if (menu == 241){
			lcd_send_cmd (0x80|0x42);
			lcd_send_string("Running...");
			set_target(50, targetX, targetY, 0);
		}

		if (menu == 242){
	//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	//		HAL_Delay(100);
	//		lcd_clear();
	//		lcd_send_cmd (0x80|0x42);
	//		lcd_send_string("Complete");
	//		HAL_Delay(3000);
	//		complete = 0;
	//		menu = 1;
		}
}

void lcd_display (void){ //Use for monitoring position
	lcd_send_cmd (0x80|0x00);
	lcd_send_string("Pos Roda 1:");

	sprintf(x_buffer, "%li", counter1);
	lcd_send_cmd (0x80|0x0B);
	lcd_send_string(x_buffer);

	lcd_send_cmd (0x80|0x40);
	lcd_send_string("Pos Roda 2:");

	sprintf(y_buffer, "%li", counter2);
	lcd_send_cmd (0x80|0x4B);
	lcd_send_string(y_buffer);

	lcd_send_cmd (0x80|0x14);
	lcd_send_string("Pos Roda 3:");

	sprintf(speed_buffer, "%li", counter8);
	lcd_send_cmd (0x80|0x1F);
	lcd_send_string(speed_buffer);
}

void remote_mode(void){
	if (data_received) {
		// Check if the received data is the expected command
		while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 255 && rx_buffer[4] == 128 && rx_buffer[5] == 255) {
			set_speeds(0,70,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
			// Toggle LED
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			//HAL_Delay(100);  // Delay for half a second
		}

		while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 255 && rx_buffer[4] == 127 && rx_buffer[5] == 255) {
			set_speeds(0,-70,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
			// Toggle LED
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			//HAL_Delay(300);  // Delay for half a second
		}

		while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 127 && rx_buffer[4] == 255 && rx_buffer[5] == 255) {
			set_speeds(70,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
			// Toggle LED
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			//HAL_Delay(500);  // Delay for half a second
		}

		while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 128 && rx_buffer[4] == 255 && rx_buffer[5] == 255) {
			set_speeds(-70,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
			// Toggle LED
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			//HAL_Delay(700);  // Delay for half a second
		}

		while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 255 && rx_buffer[4] == 255 && rx_buffer[5] == 127) {
			set_speeds(0,0,3);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
			// Toggle LED
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			//HAL_Delay(700);  // Delay for half a second
		}

		while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 255 && rx_buffer[4] == 255 && rx_buffer[5] == 128) {
			set_speeds(0,0,-3);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
			// Toggle LED
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			//HAL_Delay(700);  // Delay for half a second
		}

		while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 255 && rx_buffer[4] == 255 && rx_buffer[5] == 255) {
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}

		// Reset flag and clear buffer
		data_received = 0;
		memset(rx_buffer, 0, sizeof(rx_buffer));
	}
}

void remote_mode1(void){
	while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 255 && rx_buffer[4] == 128 && rx_buffer[5] == 255) {
		set_speeds(0,70,0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
		// Toggle LED
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//HAL_Delay(100);  // Delay for half a second
	}

	while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 255 && rx_buffer[4] == 127 && rx_buffer[5] == 255) {
		set_speeds(0,-70,0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
		// Toggle LED
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//HAL_Delay(300);  // Delay for half a second
	}

	while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 127 && rx_buffer[4] == 255 && rx_buffer[5] == 255) {
		set_speeds(70,0,0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
		// Toggle LED
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//HAL_Delay(500);  // Delay for half a second
	}

	while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 128 && rx_buffer[4] == 255 && rx_buffer[5] == 255) {
		set_speeds(-70,0,0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
		// Toggle LED
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//HAL_Delay(700);  // Delay for half a second
	}

	while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 255 && rx_buffer[4] == 255 && rx_buffer[5] == 127) {
		set_speeds(0,0,3);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
		// Toggle LED
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//HAL_Delay(700);  // Delay for half a second
	}

	while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 255 && rx_buffer[4] == 255 && rx_buffer[5] == 128) {
		set_speeds(0,0,-3);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
		// Toggle LED
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//HAL_Delay(700);  // Delay for half a second
	}

	while(rx_buffer[0]==12 && rx_buffer[1] == 24 && rx_buffer[2] == 36 && rx_buffer[3] == 255 && rx_buffer[4] == 255 && rx_buffer[5] == 255) {
		set_speeds(0,0,0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
	}
}

void set_speeds(float x_speed,float y_speed,float w_speed){
	a = cos(deg2rad(120)); b = cos(deg2rad(240)); c = cos(deg2rad(360));
	d = sin(deg2rad(120)); e = sin(deg2rad(240)); f = sin(deg2rad(360));
	g = 1/L;       	 	   h = 1/L;          	  i=1/L;
	det = a*e*i + b*f*g + c*d*h - c*e*g - a*f*h - b*d*i;
	a_inverse = (e*i - f*h)/det;  b_inverse = (h*c - i*b)/det;  c_inverse = (b*f - c*e)/det;
	d_inverse = (g*f - d*i)/det;  e_inverse = (a*i - g*c)/det;  f_inverse = (d*c - a*f)/det;
	g_inverse = (d*h - g*e)/det;  h_inverse = (g*b - a*h)/det;  i_inverse = (a*e - d*b)/det;
	pwm_mtr1 = a_inverse*x_speed + b_inverse*y_speed + c_inverse*w_speed;
	pwm_mtr2 = d_inverse*x_speed + e_inverse*y_speed + f_inverse*w_speed;
	pwm_mtr3 = g_inverse*x_speed + h_inverse*y_speed + i_inverse*w_speed;
	d1 = 0;
	d2 = 0;
	d3 = 0;
	d1 = pwm_mtr1 < 0 ? -1 : 1;
	d2 = pwm_mtr2 < 0 ? -1 : 1;
	d3 = pwm_mtr3 < 0 ? -1 : 1;
	pwm1=(int)pwm_mtr1;
	pwm2=(int)pwm_mtr2;
	pwm3=(int)pwm_mtr3;

	Mleft(pwm1,d1);
	Mright(pwm2,d2);
	Mback(pwm3,d3);
}

void set_speeds2(float x_speed,float y_speed,float w_speed){
	a = cos(deg2rad(0)); 	b = sin(deg2rad(0));	c = L;
	d = cos(deg2rad(120)); 	e = sin(deg2rad(120)); 	f = L;
	g = cos(deg2rad(240));  h = sin(deg2rad(240));  i = L;

	pwm_mtr1 = (a*x_speed + b*y_speed + c*w_speed)/r;
	pwm_mtr2 = (d*x_speed + e*y_speed + f*w_speed)/r;
	pwm_mtr3 = (g*x_speed + h*y_speed + i*w_speed)/r;
	d1 = 0;
	d2 = 0;
	d3 = 0;
	d1 = pwm_mtr1 < 0 ? -1 : 1;
	d2 = pwm_mtr2 < 0 ? -1 : 1;
	d3 = pwm_mtr3 < 0 ? -1 : 1;
	pwm1=(int)pwm_mtr1; //kecepatan motor depan
	pwm2=(int)pwm_mtr2; //kecepatan motor kiri
	pwm3=(int)pwm_mtr3; //kecepatan motor kanan

	Mleft(pwm2,d2);
	Mright(pwm3,d3);
	Mback(pwm1,d1);
}

void Mleft(int rpm1,int dir1){
	tim1_cnt = htim1.Instance -> CNT;
	direction1 = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1));
	if (tim1_cnt == 1000 && direction1 == 1) {
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		pengali1++;
	}
	else if (tim1_cnt == 1000 && direction1 == 0){
		__HAL_TIM_SET_COUNTER(&htim1, 999);
		pengali1--;
	}
	counter1 = (1000*pengali1) + tim1_cnt;
	position1 = (float) counter1 * (-1) / 38.666677;

	current_rpm1 = (freq1 * 60.0) / (200.0 * 50.9);  //motor PG45

	if (dir1 == 1){
		if (rpm1!=0){
			sp1=abs(rpm1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
			TIM20->CCR1=motorSpeed1;
		}
		if (rpm1==0){
			sp1=0;
			freq1=0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
			TIM20->CCR1=motorSpeed1;
		}
  }
  else if (dir1 == -1){
	  if (rpm1!=0){
		  sp1=abs(rpm1);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		  TIM20->CCR1=motorSpeed1;
	  }
	  if (rpm1==0){
		  sp1=0;
		  freq1=0;
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		  TIM20->CCR1=motorSpeed1;
	  }
  }

  //PID Motor Left
  uint32_t currentTime1 = HAL_GetTick();
  int deltaTime1 = currentTime1 - lastTime1;
  if (deltaTime1 >= T){
	  error1 = sp1 - current_rpm1;
  	  total_error1 += error1;
  	  if (total_error1>=max_control) total_error1 = max_control;
  	  else if (total_error1<=min_control) total_error1 = min_control;
  	  int delta_error1 = error1-last_error1;
  	  motorSpeed1 = kp1*error1 + (ki1*T)*total_error1 + (kd1/T)*delta_error1;
  	  if(motorSpeed1 >= max_control)motorSpeed1 = max_control;
  	  else if(motorSpeed1 <=  min_control) motorSpeed1 = min_control;
  	  last_error1 = error1;
  	  lastTime1 = currentTime1;
  }
}

void Mback(int rpm3,int dir3){
	tim2_cnt = htim2.Instance -> CNT;
	direction2 = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2));
	if (tim2_cnt == 1000 && direction2 == 1) {
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		pengali2++;
	}
	else if (tim2_cnt == 1000 && direction2 == 0){
		__HAL_TIM_SET_COUNTER(&htim2, 999);
		pengali2--;
	}
	counter2 = (1000*pengali2) + tim2_cnt;
	position2 = (float) counter2 * (-1) / 38.666677;

	current_rpm2 = (freq2 * 60.0) / (200.0 * 50.9); //Motor PG45

	if (dir3 == 1){
		if (rpm3 != 0){
			sp2=abs(rpm3);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);
			TIM20->CCR2=motorSpeed2;
		}
		if (rpm3 == 0){
			sp2=0;
			freq2=0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET);
			TIM20->CCR2=motorSpeed2;
		}
	}
	else if (dir3 == -1){
		if (rpm3!=0){
			sp2=abs(rpm3);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET);
			TIM20->CCR2=motorSpeed2;
		}
		if (rpm3==0){
			sp2=0;
			freq2=0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET);
			TIM20->CCR2=motorSpeed2;
		}
	}

	//PID Motor Front
	uint32_t currentTime2 = HAL_GetTick();
	int deltaTime2 = currentTime2 - lastTime2;
	if (deltaTime2 >= T){
		error2 = sp2 - current_rpm2;
		total_error2 += error2;
		if (total_error2>=max_control) total_error2 = max_control;
		else if (total_error2<=min_control) total_error2 = min_control;
		int delta_error2 = error2-last_error2;
		motorSpeed2 = kp2*error2 + (ki2*T)*total_error2 + (kd2/T)*delta_error2;
		if(motorSpeed2 >= max_control)motorSpeed2 = max_control;
		else if(motorSpeed2 <=  min_control) motorSpeed2 = min_control;
		last_error2 = error2;
		lastTime2 = currentTime2;
	}
}

void Mright(int rpm2,int dir2){
	tim8_cnt = htim8.Instance -> CNT;
	direction3 = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8));
	if (tim8_cnt == 1000 && direction3 == 1) {
		__HAL_TIM_SET_COUNTER(&htim8, 0);
		pengali8++;
	}
	else if (tim8_cnt == 1000 && direction3 == 0){
		__HAL_TIM_SET_COUNTER(&htim8, 999);
		pengali8--;
	}
	counter8 = (1000*pengali8) + tim8_cnt;
	position3 = (float) counter8 * (-1) / 38.666677;

	current_rpm3 = (freq3 * 60.0) / (200.0 * 50.9); // Motor PG45

	if (dir2 == 1){
		if (rpm2!=0){
			sp3=abs(rpm2);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
			TIM20->CCR3=motorSpeed3;
		}
		if (rpm2==0){
			sp3=0;
			freq3 = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
			TIM20->CCR3=motorSpeed3;
		}
	}
	else if (dir2 == -1){
		if (rpm2!=0){
			sp3=abs(rpm2);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
			TIM20->CCR3=motorSpeed3;
		}
		if (rpm2==0){
			sp3=0;
			freq3=0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
			TIM20->CCR3=motorSpeed3;
		}
	}

  //PID Motor Right
  uint32_t currentTime3 = HAL_GetTick();
  int deltaTime3 = currentTime3 - lastTime3;
  if (deltaTime3 >= T){
	  error3 = sp3 - current_rpm3;
  	  total_error3 += error3;
  	  if (total_error3>=max_control) total_error3 = max_control;
  	  else if (total_error3<=min_control) total_error3 = min_control;
  	  int delta_error3 = error3-last_error3;
  	  motorSpeed3 = kp3*error3 + (ki3*T)*total_error3 + (kd3/T)*delta_error3;
  	  if(motorSpeed3 >= max_control)motorSpeed3 = max_control;
  	  else if(motorSpeed3 <=  min_control) motorSpeed3 = min_control;
  	  last_error3 = error3;
  	  lastTime3 = currentTime3;
  }
}

void odometry(void){
	ma = (position1*cos(degtorad(30))) - (position3*cos(degtorad(30)));
	mb = position2 - (position1*sin(degtorad(30))) - (position3*sin(degtorad(30)));
	mc = (position1+position3+position2)/26;

	my = (cos(mc))*ma - sin(mc)*mb;
	mx = (sin(mc))*ma + (cos(mc))*mb;
	mtheta = mc;
}

void odometryv2 (void){
	ma = position1/5;
	mb = position2/5 ;
	mc = (position1+position3+position2)/26;

	my = (cos(mc))*ma - sin(mc)*mb;
	mx = (sin(mc))*ma + (cos(mc))*mb;
	mtheta = mc;

//	my = (position1*(sin(deg2rad(120)))) + (position2*(sin(degtorad(240)))) + (position3*(sin(deg2rad(360))));
//	mx = (position2*(cos(deg2rad(120)))) + (position2*(cos(degtorad(240)))) + (position3*(cos(deg2rad(360))));
//	mtheta = mc;
}

void set_target (int max_speed, int x_target, int y_target, int theta_target){
	while (x_target == 0 && y_target == 100 && theta_target == 0){ // kontrol posisi Y 100
		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ( error_y>=5 || error_y<=-5) set_speeds(0, p_y, 0);
		else if ( error_y<5 && error_y>-5) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 0 && y_target == 200 && theta_target == 0){ // kontrol posisi Y 200
		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if (error_y>=30 || error_y<=-30) set_speeds(0, p_y, 0);
		else if ( error_y<30 && error_y>-30) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 0 && y_target == 300 && theta_target == 0){ // kontrol posisi Y 300
		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if (error_y>=35 || error_y<=-35) set_speeds(0, p_y, 0);
		else if (error_y<35 && error_y>-35) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 0 && y_target >= 400 && theta_target == 0){ // kontrol posisi Y 400 ke atas
		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if (error_y>=28 || error_y<=-28) set_speeds(0, p_y, 0);
		else if ( error_y<28 && error_y>-28) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 0 && y_target == -100 && theta_target == 0){ // kontrol posisi Y (-100)
		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if (error_y>=5 || error_y<=-5) set_speeds(0, p_y, 0);
		else if ( error_y<5 && error_y>-5) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 0 && y_target == -200 && theta_target == 0){ // kontrol posisi Y (-200)
		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if (error_y>=5 || error_y<=-5) set_speeds(0, p_y, 0);
		if ( error_y<5 && error_y>-5) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 0 && y_target <= -300 && y_target >= -400 && theta_target == 0){ // kontrol posisi Y (-300) - (-400)
		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if (error_y>=22 || error_y<=-22) set_speeds(0, p_y, 0);
		else if ( error_y<22 && error_y>-22) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 0 && y_target <= -500 && y_target >= -600 && theta_target == 0){ // kontrol posisi Y (-500) - (-600)
		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if (error_y>=32 || error_y<=-32) set_speeds(0, p_y, 0);
		else if ( error_y<32 && error_y>-32) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 100 && y_target == 0 && theta_target == 0){ // kontrol posisi X 100
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		if (error_x>=5 || error_x<=-5) set_speeds(p_x, 0, 0);
		else if ( error_x<5 && error_x>-5) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 200 && y_target == 0 && theta_target == 0){ // kontrol posisi X 200
			error_x = x_target - mb;
			p_x = kp*error_x;
			if(p_x > max_speed) p_x = max_speed;
			else if (p_x < -max_speed) p_x = -max_speed;

			if (error_x>=18 || error_x<=-18) set_speeds(p_x, 0, 0);
			else if ( error_x<18 && error_x>-18) {
				kp = 0;
				set_speeds(0,0,0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
			}
		}

	while (x_target >= 300 && y_target == 0 && theta_target == 0){ // kontrol posisi X
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		if (error_x>=33 || error_x<=-33) set_speeds(p_x, 0, 0);
		else if (error_x<33 && error_x>-33) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == -100 && y_target == 0 && theta_target == 0){ // kontrol posisi -X (-100)
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		if (error_x>=8 || error_x<=-8) set_speeds(p_x, 0, 0);
		else if (error_x<8 && error_x>-8) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == -200 && y_target == 0 && theta_target == 0){ // kontrol posisi -X (-200)
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		if (error_x>=26 || error_x<=-26) set_speeds(p_x, 0, 0);
		else if (error_x<26 && error_x>-26) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target <= -300 && x_target >= -600 && y_target == 0 && theta_target == 0){ // kontrol posisi -X -300 sampai -600
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		if (error_x>=42 || error_x<=-42) set_speeds(p_x, 0, 0);
		else if (error_x<42 && error_x>-42) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 0 && y_target == 0 && theta_target != 0){ // kontrol posisi theta
		error_theta = theta_target - mc;
		p_theta = kp*error_theta;
		if(p_theta > max_speed) p_theta = max_speed;
		else if (p_theta < -max_speed) p_theta = -max_speed;

		if (error_theta>=15 || error_theta<=-15) set_speeds(0, 0, p_theta);
		else if (error_theta<15 && error_theta>-15) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == -100 && y_target == 100){ // (-x , y) -100,100
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ((error_x>=6 || error_x<=-6) || (error_y>=6 || error_y<=-6)) set_speeds(p_x, p_y, 0);
		if ((error_x<6 && error_x>-6) || (error_y<6 && error_y>-6)) {
			kp = 0;
			error_x = 0;
			error_y = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target < -100 && y_target > 100 && x_target == (-1)*y_target){ // (-x , y) di atas 200
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ((error_x>=12 || error_x<=-12) || (error_y>=12 || error_y<=-12))set_speeds(p_x, p_y, 0);
		if ((error_x<12 && error_x>-12) || (error_y<12 && error_y>-12)) {
			kp = 0;
			error_x = 0;
			error_y = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 100 && y_target == -100){ // (x , -y) 100,-100
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ((error_x>=13 || error_x<=-13) || (error_y>=13 || error_y<=-13))set_speeds(p_x, p_y, 0);
		if ((error_x<13 && error_x>-13) || (error_y<13 && error_y>-13)) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target > 100 && y_target < -100 && x_target == (-1)*y_target){ // (x , -y) diatas 200
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ((error_x>=22 || error_x<=-22) || (error_y>=22 || error_y<=-22)) set_speeds(p_x, p_y, 0);
		if ((error_x<22 && error_x>-22) || (error_y<22 && error_y>-22)) {
			kp = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 100 && y_target == 100){ //(+x, +y) sama 100,100
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ((error_x>=22 || error_x<=-22) || (error_y>=22 || error_y<=-22)) set_speeds(p_x, p_y, 0);
		if ((error_x<22 && error_x>-22) || (error_y<22 && error_y>-22)) {
			kp = 0;
			error_x = 0;
			error_y = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target > 100 && y_target > 100 && x_target == y_target){ //(+x, +y) sama diatas 200
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ((error_x>=32 || error_x<=-32) || (error_y>=32 || error_y<=-32)) set_speeds(p_x, p_y, 0);
		if ((error_x<32 && error_x>-32) || (error_y<32 && error_y>-32)) {
			kp = 0;
			error_x = 0;
			error_y = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target < -100 && y_target < -100 && x_target == y_target){ //(-x, -y) sama dibawah -200
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ((error_x>=28 || error_x<=-28) || (error_y>=28 || error_y<=-28)) set_speeds(p_x, p_y, 0);
		if ((error_x<28 && error_x>-28) || (error_y<28 && error_y>-28)) {
			kp = 0;
			error_x = 0;
			error_y = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == -100 && y_target == -100){ // (-X, -Y) sama -100,-100
		error_x = x_target - mb;
		p_x = kp*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		error_y = y_target - ma;
		p_y = kp*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ((error_x>=13 || error_x<=-13) || (error_y>=13 || error_y<=-13)) set_speeds(p_x, p_y, 0);
		if ((error_x<13 && error_x>-13) || (error_y<13 && error_y>-13)) {
			kp = 0;
			error_x = 0;
			error_y = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 100 && y_target == 200){ // contoh (100, 200)
		error_x = x_target - mb;
		p_x = kp_x*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		error_y = y_target - ma;
		p_y = kp_y*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ((error_x>=11 || error_x<=-11) || (error_y>=42 || error_y<=-42)) set_speeds(p_x, p_y, 0);
		if ((error_x<11 && error_x>-11) && (error_y<42 && error_y>-42)) {
			kp = 0;
			error_x = 0;
			error_y = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == 100 && y_target == -200){ // contoh (100, -200)
		error_x = x_target - mb;
		p_x = kp_x*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		error_y = y_target - ma;
		p_y = kp_y*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ((error_x>=20 && error_x<=-20) || (error_y>=35 || error_y<=-35)) set_speeds(p_x, p_y, 0);
		if ((error_x<20 && error_x>-20) && (error_y<35 && error_y>-35)) {
			kp = 0;
			error_x = 0;
			error_y = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == -100 && y_target == 200){ // contoh (-100, 200)
		error_x = x_target - mb;
		p_x = kp_x*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		error_y = y_target - ma;
		p_y = kp_y*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ((error_x>=20 || error_x<=-20) || (error_y>=40 || error_y<=-40)) set_speeds(p_x, p_y, 0);
		if ((error_x<20 && error_x>-20) && (error_y<40 && error_y>-40)) {
			kp = 0;
			error_x = 0;
			error_y = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	while (x_target == -100 && y_target == -200){ // contoh (-100, -200)
		error_x = x_target - mb;
		p_x = kp_x*error_x;
		if(p_x > max_speed) p_x = max_speed;
		else if (p_x < -max_speed) p_x = -max_speed;

		error_y = y_target - ma;
		p_y = kp_y*error_y;
		if(p_y > max_speed) p_y = max_speed;
		else if (p_y < -max_speed) p_y = -max_speed;

		if ((error_x>=24 || error_x<=-24) || (error_y>=35 || error_y<=-35)) set_speeds(p_x, p_y, 0);
		if ((error_x<24 && error_x>-24) && (error_y<35 && error_y>-35)) {
			kp = 0;
			error_x = 0;
			error_y = 0;
			set_speeds(0,0,0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
		}
	}

	//kinematika(p_x, p_y, p_theta);
//	set_speeds(p_x, p_y, p_theta);
//	//HAL_Delay(30);
//	if (error_x<15 && error_x>-15 && error_y<15 && error_y>-15 && error_theta<15 && error_theta>-15) set_speeds(0,0,0);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM15_Init();
  MX_TIM20_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
  HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_buffer, sizeof(rx_buffer));
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_3);
  lcd_startUp();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of task_1 */
  task_1Handle = osThreadNew(task_1_function, NULL, &task_1_attributes);

  /* creation of task_2 */
  task_2Handle = osThreadNew(task_2_function, NULL, &task_2_attributes);

  /* creation of task_3 */
  task_3Handle = osThreadNew(task_3_function, NULL, &task_3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  set_speeds(0, 70, 0);
//	  remote_mode();
//	  odometry();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30A0A7FB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 16-1;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 16-1;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 16-1;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 16-1;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65534;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65534;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 16-1;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 16-1;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 16-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65534;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR5;
  if (HAL_TIM_SlaveConfigSynchro(&htim15, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM20 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM20_Init(void)
{

  /* USER CODE BEGIN TIM20_Init 0 */

  /* USER CODE END TIM20_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM20_Init 1 */

  /* USER CODE END TIM20_Init 1 */
  htim20.Instance = TIM20;
  htim20.Init.Prescaler = 170-1;
  htim20.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim20.Init.Period = 1049;
  htim20.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim20.Init.RepetitionCounter = 0;
  htim20.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim20) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim20, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim20) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim20, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim20, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim20, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim20, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim20, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM20_Init 2 */

  /* USER CODE END TIM20_Init 2 */
  HAL_TIM_MspPostInit(&htim20);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|MLEFT_A_Pin|MLEFT_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MBACK_A_Pin|MBACK_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MRIGHT_A_Pin|MRIGHT_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin MLEFT_A_Pin MLEFT_B_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|MLEFT_A_Pin|MLEFT_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MBACK_A_Pin MBACK_B_Pin */
  GPIO_InitStruct.Pin = MBACK_A_Pin|MBACK_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MRIGHT_A_Pin MRIGHT_B_Pin */
  GPIO_InitStruct.Pin = MRIGHT_A_Pin|MRIGHT_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PBRight_Pin PBLeft_Pin PBUp_Pin */
  GPIO_InitStruct.Pin = PBRight_Pin|PBLeft_Pin|PBUp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PBDown_Pin */
  GPIO_InitStruct.Pin = PBDown_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PBDown_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    data_received = 1;
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_task_1_function */
/**
  * @brief  Function implementing the task_1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_task_1_function */
void task_1_function(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	  odometry();
//	  inverse_odometry(0, 100, 0);
	  odometryv2();
//	  Mleft(18, -1);
//	  Mright(18, -1);
//	  Mback(36, 1);
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task_2_function */
/**
* @brief Function implementing the task_2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_2_function */
void task_2_function(void *argument)
{
  /* USER CODE BEGIN task_2_function */
  /* Infinite loop */
  for(;;)
  {
//	  wheelKiri();
//	  remote_mode();
//	  set_target(50, 100, 200, 0);
//	  lcd_display();
	  lcd_display_1();
//	  set_targetv2(0, 100, 0);
	  osDelay(1);
  }
  /* USER CODE END task_2_function */
}

/* USER CODE BEGIN Header_task_3_function */
/**
* @brief Function implementing the task_3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_3_function */
void task_3_function(void *argument)
{
  /* USER CODE BEGIN task_3_function */
  /* Infinite loop */
  for(;;)
  {
//	  wheelKanan();
//	  lcd_display();
//	  while(rx_buffer[0]!=12 && rx_buffer[1]!=24 && rx_buffer[2]!=36){
//		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
//		  NVIC_SystemReset();
//	  }
	  osDelay(1);
  }
  /* USER CODE END task_3_function */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

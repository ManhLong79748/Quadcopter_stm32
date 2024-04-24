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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "MY_NRF24.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MPU6050_ADDR 			0x68<<1
#define SMPLRT_DIV_REG 		0x19
#define GYRO_CONFIG_REG 	0x1B
#define ACCEL_CONFIG_REG 	0x1C
#define ACCEL_XOUT_H_REG  0x3B
#define TEMP_OUT_H_REG 		0x41
#define GYRO_XOUT_H_REG 	0x43
#define PWR_MGMT_1_REG 		0x6B
#define CONFIG       			0x1A


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t data;
uint32_t looptimer;
uint8_t buffer[2], tuffer[6], cuffer[6];
int16_t gyro_raw[3], acc_raw[3];
float gyro_cal[3];
float acc_cal[3];
int16_t acc_total_vector;
float angle_pitch_gyro, angle_roll_gyro;
float angle_pitch_acc, angle_roll_acc;
float angle_pitch, angle_roll;

int16_t raw_temp; float temp;
int i; 
float prevtime, prevtime1,time1,elapsedtime1,prevtime2, time2,elapsedtime2;

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;

// kalman 
float KalmanAngleRoll  =0,  Kalman_ERR_AngleRoll  = 2*2 ; // gia su sai so theo moi truc la 2
float KalmanAnglePitch =0,  Kalman_ERR_AnglePitch = 2*2 ;
float Kalman_ouput[2] = {0,0}; // angle prediction, uncertainly of the predictiobn 

// PID va
//float PIDReturn[9]= {0 };
//float DesiredRateRoll, DesiredRatePitch,
//DesiredRateYaw;
//float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
//float InputRoll, InputThrottle, InputPitch, InputYaw;
//float PrevErrorRateRoll, PrevErrorRatePitch,
//PrevErrorRateYaw;
//float PrevItermRateRoll, PrevItermRatePitch,
//PrevItermRateYaw;


//float PRateRoll=2 ; 
//float PRatePitch =2;
//float PRateYaw=2;

//float IRateRoll=0.05 ;
//float IRatePitch=0.05;
//float IRateYaw=12;

//float DRateRoll=0.03 ; 
//float DRatePitch=0.03;
//float DRateYaw=0;

											//PID

float pid_set_points_roll,pid_set_points_pitch,pid_set_points_yaw; // roll, pitch, yaw
int pid_max[3] = { 400, 400, 400 };		 // roll, pitch, yaw
// Errors
float errors[3]; // Measured errors (compared to instructions) : [roll, pitch, yaw]
float delta_err[3] = { 0, 0, 0 }; // Error deltas in that order   : roll, pitch, yaw
float error_sum[3] = { 0, 0, 0 }; // Error sums (used for integral component) : [roll, pitch, yaw]
float previous_error[3] = { 0, 0, 0 }; // Last errors (used for derivative component) : [roll, pitch, yaw]
// PID coefficients

float Kp[3] = { 1.5, 1.5, 2 };
float Ki[3] = {0.03, 0.03, 0.02}; 	 	// I: roll, pitch, yaw
float Kd[3] = { 0.01, 0.01, 0 };         // D: roll, pitch, yaw

float pid_roll = 0;
float pid_pitch = 0;
float pid_yaw = 0;

#define roll     0
#define pitch    1
#define yaw      2

float roll_level_adjust, pitch_level_adjust;

float capture1,capture2,capture3,capture4;
// Esc and active

uint8_t active, first_angle = 0;
int16_t esc1, esc2, esc3, esc4,throttle;

// NRF24 transmit
uint64_t TxpipeAddrs = 0x11223344AA;
float myTxData[32]; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Input capture callback
uint32_t channel_1_start, channel_1;
uint32_t channel_2_start, channel_2;
uint32_t channel_3_start, channel_3;
uint32_t channel_4_start, channel_4;

volatile uint32_t diff1 = 0, diff2 = 0, diff3 = 0, diff4 = 0;

uint8_t firt_capture1 = 0, firt_capture2 = 0, firt_capture3 = 0, firt_capture4 =0;	//fist_capture? 0-no 1-yes
		

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) 
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		if (firt_capture1 == 0) {
			channel_1_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			firt_capture1 = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
		} else if (firt_capture1 == 1) {
			channel_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			if (channel_1 > channel_1_start)
				diff1 = channel_1 - channel_1_start;
			else if (channel_1 < channel_1_start)
				diff1 = (0xffff - channel_1_start) + channel_1;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);		
			firt_capture1 = 0;
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		if (firt_capture2 == 0) {					//read
			channel_2_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			firt_capture2 = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		} else if (firt_capture2 == 1) {
			channel_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			if (channel_2 > channel_2_start)
				diff2 = channel_2 - channel_2_start;
			else if (channel_2 < channel_2_start)
				diff2 = (0xffff - channel_2_start) + channel_2;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2,
					TIM_INPUTCHANNELPOLARITY_RISING);
			firt_capture2 = 0;
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		if (firt_capture3 == 0) {					//read
			channel_3_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			firt_capture3 = 1;
			//Change the input capture mode to the falling edge of the pulse
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		} else if (firt_capture3 == 1) {
			channel_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			if (channel_3 > channel_3_start)
				diff3 = channel_3 - channel_3_start;
			else if (channel_3 < channel_3_start)
				diff3 = (0xffff - channel_3_start) + channel_3;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3,
					TIM_INPUTCHANNELPOLARITY_RISING);
			firt_capture3 = 0;
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		if (firt_capture4 == 0) {					//read
			channel_4_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			firt_capture4 = 1;
			//Change the input capture mode to the falling edge of the pulse
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		} else if (firt_capture4 == 1) {
			channel_4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			if (channel_4 > channel_4_start)
				diff4 = channel_4 - channel_4_start;
			else if (channel_4 < channel_4_start)
				diff4 = (0xffff - channel_4_start) + channel_4;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4,
					TIM_INPUTCHANNELPOLARITY_RISING);
			firt_capture4 = 0;
		}
	}
}


// KALMAN
void Kalman (float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)  // kalma angle predictio, error, RateRoll, angle_roll
{
    KalmanState = KalmanState + 0.004 * KalmanInput; // 1/250
    KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 *4 ; 				//( T^2 * ERR^2)
    float KalmanGain = KalmanUncertainty* 1/ (1*KalmanUncertainty + 3*3) ;// err in measure
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);  // current estimate
    KalmanUncertainty = (1- KalmanGain) * KalmanUncertainty;   // error in estimate

    Kalman_ouput[0] = KalmanState;
    Kalman_ouput[1] = KalmanUncertainty; // error estimate
}

// MPUinit
void MPU6050_Init()
{
  // power management register 0X6B we should write all 0's to wake the sensor up
  data =  0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG,1 , &data, 1, HAL_MAX_DELAY);

	data =  0x05;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG,1 , &data, 1, HAL_MAX_DELAY);
	
  // Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=1 -> ± 500 °/s
  data =  0x08;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG,1 , &data, 1, HAL_MAX_DELAY);

  // Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=3 -> ± 8g
  data =  0x10;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG,1 , &data, 1, HAL_MAX_DELAY);
}

void Calibrate()
  {
    for ( i=0;i < 2000; i++)
    {
			if (i % 125 ==0) HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//      prevtime2 = time2;
//      time2 = HAL_GetTick();
//      elapsedtime2 = (time2 - prevtime2) * 1000;
      cuffer[0] = 0x43 ;
      HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR, cuffer,1, HAL_MAX_DELAY);
      HAL_I2C_Master_Receive (&hi2c1,MPU6050_ADDR,cuffer, 6, HAL_MAX_DELAY);

      gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]);
      gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]);
      gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]);

      gyro_cal[0] += gyro_raw[0];
      gyro_cal[1] += gyro_raw[1];
      gyro_cal[2] += gyro_raw[2];
      HAL_Delay(4);
    }
      gyro_cal[0] /= 2000;
      gyro_cal[1] /= 2000;
      gyro_cal[2] /= 2000;
    
		// calibrate value for acc
      acc_cal[0] = -80;
      acc_cal[1] = 0;
      acc_cal[2] = -580;
  }

  void Read_mpu_all()
  {
    tuffer[0] = 0x3B;
    HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,tuffer,1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,tuffer,6, HAL_MAX_DELAY);

    acc_raw[0] = (tuffer[0] << 8 | tuffer[1]) ; // roll
    acc_raw[1] = (tuffer[2] << 8 | tuffer[3]) ;//  pitch
    acc_raw[2] = (tuffer[4] << 8 | tuffer[5]) ;//  yaw
    
    acc_raw[0] += acc_cal[0];
    acc_raw[1] += acc_cal[1];
    acc_raw[2] += acc_cal[2];
    
    AccX= (float) acc_raw[0] / 4096;
    AccY= (float) acc_raw[1] / 4096;
    AccZ= (float) acc_raw[2] / 4096;

    angle_roll  =  atan(AccY / sqrt(AccX * AccX + AccZ* AccZ)) * 1/ (3.142/180);
    angle_pitch = -atan(AccX / sqrt(AccY * AccY + AccZ* AccZ)) * 1/ (3.142/180);
    
    buffer[0] = 0x41;
     HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR, buffer, 1,HAL_MAX_DELAY);
     HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR, buffer, 2,HAL_MAX_DELAY);

    //  _temp
    raw_temp = (buffer[0] << 8  | buffer[1]);
    temp = (raw_temp /340.0) + 36.53;

    cuffer[0] = 0x43;
    HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR, cuffer, 1,HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR, cuffer, 6,HAL_MAX_DELAY);
		
    gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]);
    gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]);
    gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]);

    gyro_raw[0] -= gyro_cal[0];
    gyro_raw[1] -= gyro_cal[1];
    gyro_raw[2] -= gyro_cal[2];

    RateRoll = (float) gyro_raw[0] /65.5;
    RatePitch = (float) gyro_raw[1] /65.5;
    RateYaw = (float) gyro_raw[2]  /65.5;

    // RateRoll = 0.7 * RateRoll + ((float) gyro_raw[0] /65.5) * 0.3;   // dec/s
    // RatePitch = 0.7 * RatePitch + ((float) gyro_raw[1] /65.5) * 0.3;
    // RateYaw = 0.7 * RateYaw + ((float) gyro_raw[2]  /65.5) * 0.3;
  }

//  void pid_equation(float Error, float P , float I, float D,float PrevError, float PrevIterm) 
//  {

//      float Pterm = P*Error;  // Kp * error

//      float Iterm = PrevIterm + I*(Error+PrevError)* 0.004/2;  // Ts = 0.004= 4ms // giá trị của Ki bằng sai số hieenjn tại cộng với trước chia 2

//      if (Iterm > 400) Iterm=400;  // limit the Iterm
//      else if (Iterm <-400) Iterm=-400;

//      float Dterm= D*(Error-PrevError)/0.004;  // gia tri cua bộ D // khau  vi phan
//      float PIDOutput= Pterm+Iterm+Dterm;
//      if (PIDOutput>400) PIDOutput=400;
//      else if (PIDOutput <-400)
//       PIDOutput=-400;

//      PIDReturn[0]=PIDOutput;
//      PIDReturn[1]=Error;
//      PIDReturn[2]=Iterm;
//  }
void reset_pid(void) 
  {
      errors[0] = 0;
			errors[1] = 0;
			errors[2] = 0;
			delta_err[0] = 0;
			delta_err[1] = 0;
			delta_err[2] = 0;
			error_sum[0] = 0;
			error_sum[1] = 0;
			error_sum[2] = 0;
			previous_error[0] = 0;
			previous_error[1] = 0;
			previous_error[2] = 0;
			pid_roll = 0;
			pid_pitch = 0;
			pid_yaw = 0;
  }

//  void caculate_pid()
//  {
//      DesiredRateRoll=0.15*(diff1-1500); // limit the roll rate in 75"/s  500 * 0.15 = 75
//      DesiredRatePitch=0.15*(diff2-1500); // RatePitch: tốc dộ goc trục pitch mong muốn
//      InputThrottle=diff3;
//      DesiredRateYaw=0.15*(diff4-1500);

//      ErrorRateRoll=DesiredRateRoll-RateRoll; // sai số = toc do goc mong muon - toc do goc do dac
//      ErrorRatePitch=DesiredRatePitch-RatePitch;
//      ErrorRateYaw=DesiredRateYaw-RateYaw;

//      pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll,PrevItermRateRoll);
//      InputRoll=PIDReturn[0];
//      PrevErrorRateRoll=PIDReturn[1];
//      PrevItermRateRoll=PIDReturn[2];

//      pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
//      InputPitch=PIDReturn[0];
//      PrevErrorRatePitch=PIDReturn[1];
//      PrevItermRatePitch=PIDReturn[2];

//      pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw,PrevItermRateYaw);
//      InputYaw=PIDReturn[0];
//      PrevErrorRateYaw=PIDReturn[1];
//      PrevItermRateYaw=PIDReturn[2];

//      if (InputThrottle > 2000) InputThrottle = 2000;

//}

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_Init();
  Calibrate();
	NRF24_begin(GPIOB, GPIO_PIN_11, GPIO_PIN_10, hspi1);
	//NRF24_begin(GPIOB, CSNpin_Pin, CEpin_Pin, hspi1);	
	NRF24_stopListening(); 
	NRF24_openWritingPipe(TxpipeAddrs);	
	NRF24_setAutoAck(true);
	NRF24_setChannel(52);
	NRF24_setPayloadSize(32);
	NRF24_enableDynamicPayloads();
	NRF24_enableAckPayload();

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	//wait for connection tx-rx
	while (diff1 < 990 || diff2 < 990 || diff3 < 990 || diff4 < 990);
	//while (diff3 < 990 || diff3 > 1050);
	

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	//------------------------------------------------------
	//esc calibration
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2000);	//fl
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000);	//fr
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 2000);	//rr
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 2000); //rl

	HAL_Delay(50);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);	//fl
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);	//fr
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1000);	//rr
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1000); //rl

	HAL_Delay(300);
	
	//prevtime = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // prevtime1 = time1;
    // time1 = HAL_GetTick();
    // elapsedtime1 = (time1 - prevtime1) * 1000;
		capture1 = diff1;
		capture2 = diff2;
    capture3 = diff3;
		capture4 = diff4;
		
    Read_mpu_all();
    Kalman(KalmanAngleRoll,Kalman_ERR_AngleRoll, RateRoll, angle_roll);
    KalmanAngleRoll = Kalman_ouput[0];
    Kalman_ERR_AngleRoll = Kalman_ouput[1];

    Kalman(KalmanAnglePitch,Kalman_ERR_AnglePitch, RatePitch, angle_pitch);
    KalmanAnglePitch = Kalman_ouput[0];
    Kalman_ERR_AnglePitch = Kalman_ouput[1];

    // active   
		pitch_level_adjust =  KalmanAnglePitch* 15;                                           //Calculate the pitch angle correction.
		roll_level_adjust = KalmanAngleRoll * 15;     

    if (capture3 < 1050 && capture4 < 1300)
			active = 1;
		if (active == 1 && capture3 < 1050 && capture4 > 1550) 
    {
			active = 2;
			//Reset the PID controllers for a bumpless start.
		  reset_pid();
		}
     if (active == 2 && capture3 < 1050 && capture4 > 1700) active = 0;

    pid_set_points_roll = 0;
		pid_set_points_pitch = 0;
		pid_set_points_yaw = 0;

		if (capture1 > 1510)
			pid_set_points_roll = (capture1 - 1510)/3;
		else if (capture1 < 1490)
			pid_set_points_roll = (capture1 - 1490)/3;
		
		pid_set_points_roll = pid_set_points_roll - roll_level_adjust;
		
		//pid_set_points[roll] = pid_set_points[roll]/3;
		
		if (capture2 > 1510)
			pid_set_points_pitch = (capture2 - 1510)/3;
		else if (capture2 < 1490)
			pid_set_points_pitch = (capture2 - 1490)/3;
		
	    pid_set_points_pitch = pid_set_points_pitch - pitch_level_adjust;

		if (capture3 > 1150) {  //Do not yaw when turning off the motors.
			if (capture4 > 1510)
				pid_set_points_yaw = (capture4 - 1510)/3;
			else if (capture4 < 1490)
				pid_set_points_yaw = (capture4 - 1490) /3;
			
		}

		// Calculate current errors
		errors[roll] = RateRoll - pid_set_points_roll;
		errors[pitch] = RatePitch - pid_set_points_pitch;
		errors[yaw] = RateYaw - pid_set_points_yaw;

		// Calculate sum of errors : Integral coefficients
		error_sum[roll]  += errors[roll];
		error_sum[pitch] += errors[pitch];
		error_sum[yaw]   += errors[yaw];

		// Keep values in acceptable range
		if (error_sum[roll] < (-180 / Ki[roll]))
			error_sum[roll] = -180 / Ki[roll];
		if (error_sum[roll] > 180 / Ki[roll])
			error_sum[roll] = 180 / Ki[roll];
		if (error_sum[pitch] < (-180 / Ki[pitch]))
			error_sum[pitch] = -180 / Ki[pitch];
		if (error_sum[pitch] > 180 / Ki[pitch])
			error_sum[pitch] = 180 / Ki[pitch];
		if (error_sum[yaw] < (-180 / Ki[yaw]))
			error_sum[yaw] = -180 / Ki[yaw];
		if (error_sum[yaw] > 180 / Ki[yaw])
			error_sum[yaw] = 180 / Ki[yaw];

		// Calculate error delta : Derivative coefficients
		delta_err[roll] = errors[roll] - previous_error[roll];
		delta_err[pitch] = errors[pitch] - previous_error[pitch];
		delta_err[yaw] = errors[yaw] - previous_error[yaw];

		// Save current error as previous_error for next time
		previous_error[roll] = errors[roll];
		previous_error[pitch] = errors[pitch];
		previous_error[yaw] = errors[yaw];

		throttle = capture3;

		if (throttle > 1050) {
			// PID = e.Kp + ∫e.Ki.t + Δe.Kd/t
			pid_roll = (errors[roll] * Kp[roll])
					+ (error_sum[roll] * Ki[roll]) * 0.004
					+ (delta_err[roll] * Kd[roll]) / 0.004;
			pid_pitch = (errors[pitch] * Kp[pitch])
					+ (error_sum[pitch] * Ki[pitch]) * 0.004
					+ (delta_err[pitch] * Kd[pitch]) / 0.004;
			pid_yaw = (errors[yaw] * Kp[yaw])
					+ (error_sum[yaw] * Ki[yaw]) * 0.004
					+ (delta_err[yaw] * Kd[yaw]) / 0.004;

			if (pid_roll > pid_max[roll])
				pid_roll = pid_max[roll];
			if (pid_roll < (-pid_max[roll]))
				pid_roll = -pid_max[roll];
			if (pid_pitch > pid_max[pitch])
				pid_pitch = pid_max[pitch];
			if (pid_pitch < (-pid_max[pitch]))
				pid_pitch = -pid_max[pitch];
			if (pid_yaw > pid_max[yaw])
				pid_yaw = pid_max[yaw];
			if (pid_yaw < (-pid_max[yaw]))
				pid_yaw = -pid_max[yaw];
		}
    if (active == 2) 
    {
			if (throttle > 1800)
				throttle = 1800;

			esc1 = throttle + pid_roll + pid_pitch + pid_yaw; //Calculate the pulse for esc 1 (front-right - CCW).
			esc2 = throttle - pid_roll + pid_pitch - pid_yaw; //Calculate the pulse for esc 2 (front-left - CW).
			esc3 = throttle - pid_roll - pid_pitch + pid_yaw; //Calculate the pulse for esc 3 (rear-left - CCW).
			esc4 = throttle + pid_roll - pid_pitch - pid_yaw; //Calculate the pulse for esc 4 (rear-right - CW).

			if (esc1 < 1100)
				esc1 = 1100;  //Keep the motors running.
			if (esc2 < 1100)
				esc2 = 1100;  //Keep the motors running.
			if (esc3 < 1100)
				esc3 = 1100;  //Keep the motors running.
			if (esc4 < 1100)
				esc4 = 1100;  //Keep the motors running.

			if (esc1 > 2000)
				esc1 = 2000;  //Limit the esc-1 pulse to 2000us.
			if (esc2 > 2000)
				esc2 = 2000;  //Limit the esc-2 pulse to 2000us.
			if (esc3 > 2000)
				esc3 = 2000;  //Limit the esc-3 pulse to 2000us.
			if (esc4 > 2000)
				esc4 = 2000;  //Limit the esc-4 pulse to 2000us.
		} 
    else {
			esc1 = 1000;
			esc2 = 1000;
			esc3 = 1000;
			esc4 = 1000;
		  }
                                                
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, esc1);			//fr
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, esc2);			//fl
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, esc4);			//br
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, esc3);			//bl
			
			
		myTxData[0] =KalmanAngleRoll;
		myTxData[1] =KalmanAnglePitch;
		myTxData[2] =esc1;
		myTxData[3] =esc2;
	
		NRF24_write(myTxData, 32);


	while ((HAL_GetTick() - prevtime) * 1000 < 4000);
	prevtime = HAL_GetTick();

  }
	
	
		// Transmit data
	

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <imu_complementary_filter.h>
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dshot300.h"
#include "mpu6050.h"
#include "stdio.h"//printf function
#include "pid_controller.h"
#include <string.h>
#include <stdbool.h>
#include "bno055_stm32.h"
#include "imu_kalman_filter.h"
#include "imu_complementary_filter.h"
#include "esp32_cmd.h"
#include "hcsr04.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESP32_UART_HANDLE &huart2
#define ESP32_MSG_LENGTH 6
#define BNO_I2C_HANDLE &hi2c3

#define SAMPLE_TIME 0.01f

#define PID_KP_MIN 4.0f
#define PID_KP_MAX 12.0f
#define PID_KI_MIN 0.1f
#define PID_KI_MAX 5.0f
#define PID_KD_MIN 4.0f
#define PID_KD_MAX 20.0f
#define PID_TAU_MIN 0.1f
#define PID_TAU_MAX 0.2f

#define ROLL_REF_MIN -15.0f
#define ROLL_REF_MAX 15.0f

#define PITCH_REF_MIN -15.0f
#define PITCH_REF_MAX 15.0f

#define YAW_REF_MIN 100.0f
#define YAW_REF_MAX 200.0f

#define Z_REF_MIN 3.0f
#define Z_REF_MAX 20.0f

#define YAW_BIAS 150.0f

#define SPEED_MIN 48
#define MAX_SPEED 1900

#define SPEED_OFFSET 600.0f
#define ADC_TIMEOUT 1   // us

#define speed 200

//Kalman filter parameters

#define Qa_roll  0.006955685f
#define Qa_pitch  0.012531481f

#define Qb_roll  0.00006955685f
#define Qb_pitch  0.00006955685f

#define R_roll 0.00708008f
#define R_pitch 0.005146486f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Podstawowe dane i parametry */
uint16_t speed_ref[DSHOT_NUM_MOTORS] = {speed, speed, speed, speed};
volatile uint8_t white_button_flag = 0;

/* PID controllery */
PID_t pid_pitch;
PID_t pid_roll;
PID_t pid_yaw;
PID_t pid_z;

/* Wzmocnienia regulatorów */
float kp = 4.5f;
float ki = 2.6f;
float kd = 14.0f;
float tau = 0.09f;

float kp_y = 5.0f;
float ki_y = 1.5f;
float kd_y = 12.0f;
float tau_y = 0.1f;

float kp_z = 22.0f;
float ki_z = 6.0f;
float kd_z = 8.0f;
float tau_z = 0.1f;

/* Liczniki i sygnały odniesienia */
volatile float pid_z_counter = 1.0f;
volatile float ref_signal_counter = -1000.0f;
volatile uint8_t num_ref = 0;
volatile uint8_t num_ref_prev = 0;

/* Wartości zadane */
float REF_PITCH_ANGLE = 0.0f;
float REF_ROLL_ANGLE  = 0.0f;
float REF_YAW_ANGLE   = 100.0f;
float REF_Z_DISTANCE  = 10.0f;

/* Sekwencje sygnałów odniesienia */
static const float ref_signals[][4] = {
    // pitch, roll, yaw, z
    [0]  = {0.0, 0.0, 100.0, 10.0},
    [1]  = {0.0, 0.0, 100.0, 20.0},
    [2]  = {0.0, 0.0, 100.0, 10.0},
    [3]  = {0.0, 0.0, 200.0, 10.0},
    [4]  = {0.0, 0.0, 100.0, 10.0},
    [5]  = {10.0, 0.0, 100.0, 10.0},
    [6]  = {-10.0, 0.0, 100.0, 10.0},
    [7]  = {0.0, 10.0, 100.0, 10.0},
    [8]  = {0.0, -10.0, 100.0, 10.0},
    [9]  = {0.0, 0.0, 100.0, 10.0},
    [10] = {0.0, 0.0, 100.0, 3.0},
    [11] = {0.0, 0.0, 100.0, 10.0},
};

/* Flagi i stany systemowe */
volatile uint8_t SS = 0;
volatile uint8_t emergency_stop_flag = 0;

/* Dane z czujników i filtru */
volatile bno055_vector_t bno_vector;
volatile float copter_pitch_angle;
volatile float copter_roll_angle;
volatile float copter_yaw_angle;
volatile float copter_z_distance;

volatile float copter_pitch_angle_kal;
volatile float copter_roll_angle_kal;

/* Wyjścia regulatorów */
volatile float speed_pitch_ref;
volatile float speed_roll_ref;
volatile float speed_yaw_ref;
volatile float speed_z_ref;

/* Sygnały prędkości dla ESC */
volatile uint16_t speed_1_ref;
volatile uint16_t speed_2_ref;
volatile uint16_t speed_3_ref;
volatile uint16_t speed_4_ref;

/* Komunikacja z ESP32 */
uint8_t esp32_data_received_flag = 0;
uint8_t rx_esp32_data[ESP32_MSG_LENGTH];

/* Debug UART */
uint8_t uart_line[64];
int uart_line_length;
uint32_t UartDebugSoftTimer;

/* IMU i filtry */
MPU6050_t MPU6050;
IMU_Angles imu_angles;
KalmanFilter Roll;
KalmanFilter Pitch;

/* HC-SR04 – struktura drivera */
static HCSR04_t hcsr;

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t compute_crc8(const uint8_t *data, uint8_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float AngX, AngY;
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
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  //MPU6050_Init(&hi2c3);
  //MPU6050_Calibrate_Gyro(&hi2c3, &MPU6050, 500);

  Kalman_Init(&Roll, Qa_roll, Qb_roll, R_roll);
  Kalman_Init(&Pitch, Qa_pitch, Qb_pitch, R_pitch);

  HAL_TIM_Base_Start(&htim2); //Czas do czujników odległościowych
  HAL_TIM_Base_Start_IT(&htim15); //Timer od częstotliwości regulatora i wysyłania prędkości do drona
  //HAL_TIM_Base_Start_IT(&htim1); // control loop interrupt


  // PID controllers
  	PID_Init_Bartek_s_Lab(&pid_pitch, kp, ki, kd,
  	tau, -250.0f, 250.0f, SAMPLE_TIME);

  	PID_Init_Bartek_s_Lab(&pid_roll, kp, ki, kd,
  	tau, -250.0f, 250.0f, SAMPLE_TIME);

  	PID_Init_Bartek_s_Lab(&pid_yaw, kp_y, ki_y, kd_y,
  	tau_y, -300.0f, 300.0f, SAMPLE_TIME);

  	PID_Init_Bartek_s_Lab(&pid_z, kp_z, ki_z, kd_z,
  	  	tau_z, -150.0f, 400.0f, SAMPLE_TIME);

  	UartDebugSoftTimer = HAL_GetTick();

	HAL_Delay(2000); // let the motor stop after uC RST

	dshot_arm_all_esc();

//	SS=2;

	// AHRS
		bno055_assignI2C(BNO_I2C_HANDLE);
		bno055_setup();
		bno055_setOperationModeNDOF();

	static ESP32_CmdCtx esp_ctx = {
		    .pid_pitch = &pid_pitch,
		    .pid_roll  = &pid_roll,
		    .pid_yaw   = &pid_yaw,
		    .pid_z     = &pid_z,

		    .kp = &kp, .ki = &ki, .kd = &kd, .tau = &tau,

		    .kp_y = &kp_y, .ki_y = &ki_y, .kd_y = &kd_y, .tau_y = &tau_y,
		    .kp_z = &kp_z, .ki_z = &ki_z, .kd_z = &kd_z, .tau_z = &tau_z,

		    .ref_roll  = &REF_ROLL_ANGLE,
		    .ref_pitch = &REF_PITCH_ANGLE,
		    .ref_yaw   = &REF_YAW_ANGLE,
		    .ref_z     = &REF_Z_DISTANCE,

		    .kp_min = PID_KP_MIN,   .kp_max = PID_KP_MAX,
		    .ki_min = PID_KI_MIN,   .ki_max = PID_KI_MAX,
		    .kd_min = PID_KD_MIN,   .kd_max = PID_KD_MAX,
		    .tau_min = PID_TAU_MIN, .tau_max = PID_TAU_MAX,

		    .roll_min  = ROLL_REF_MIN,   .roll_max  = ROLL_REF_MAX,
		    .pitch_min = PITCH_REF_MIN,  .pitch_max = PITCH_REF_MAX,
		    .yaw_min   = YAW_REF_MIN,    .yaw_max   = YAW_REF_MAX,
		    .z_min     = Z_REF_MIN,      .z_max     = Z_REF_MAX,
		};

	/* HC-SR04: TIM2 jako licznik czasu ECHO, TRIG/ECHO jak w CubeMX */
	HCSR04_Init(&hcsr,
	            &htim2,
	            1000000u,                  /* timer_hz (tu 1 MHz) */
	            TRIG_PIN_GPIO_Port, TRIG_PIN_Pin,
	            ECHO_PIN_Pin);

	/* Opcjonalnie: co 6 wywołań timera sterującego zrób trigger (jak Twój Trig_counter) */
	HCSR04_SetPeriodicDivider(&hcsr, 6);
	/* Opcjonalnie: długość impulsu TRIG (domyślnie ~50 µs) */
	HCSR04_SetTrigPulse(&hcsr, 400);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  	memset(rx_esp32_data, 0x00, ESP32_MSG_LENGTH); //Zeruje bufor rx_esp32_data
  	HAL_UART_Receive_IT(ESP32_UART_HANDLE, rx_esp32_data,ESP32_MSG_LENGTH); // uruchamia odbieranie danych przez UART w trybie przerwań. Kiedy dane przyjdą, zostanie wywołane HAL_UART_RxCpltCallback.
	while (1)
	{

		if (white_button_flag == 1)
		{
			white_button_flag = 0;
			__NOP();
		}
	    (void)ESP32_PollAndProcess(&esp32_data_received_flag,
	                               rx_esp32_data,
	                               ESP32_MSG_LENGTH,    // = 6
	                               &esp_ctx);



//		PID_Controller_Update_Gains(&pid_pitch, kp, ki, kd, tau);
//		PID_Controller_Update_Gains(&pid_roll, kp, ki, kd, tau);
//		PID_Controller_Update_Gains(&pid_yaw, kp_y, ki_y, kd_y, tau_y);
//		PID_Controller_Update_Gains(&pid_z, kp_z, ki_z, kd_z, tau_z);
		if (num_ref!=num_ref_prev){
			REF_SIGNAL(num_ref);
			num_ref_prev=num_ref;
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

//////////////////FUNKCJA ODPOWIADAJĄCA ZA PRZELICZENIE WYCHYLENIA NA PRĘDKOŚCI REFERENCYJNE SILNIKÓW/////////////////////////

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //wejście w przerwanie
{
	uint16_t speeds[4];
	uint16_t base[4];
	//float U_vec[3];

	if (htim->Instance == TIM15) //sprawdzenie od którego timera jest przerwanie
	{
		/* Periodyczne wyzwalanie pomiaru odległości */
		(void)HCSR04_OnPeriodic(&hcsr);

		/* Jeśli przyszła nowa próbka – pobierz i przypisz do zmiennej systemowej */
		if (HCSR04_HasNew(&hcsr)) {
		    copter_z_distance = HCSR04_GetDistanceCm(&hcsr);
		}


		/*
		/////////////POBIERANIE WYCHYLENIA Z CZUJNIKA MPU6050/////////////
		MPU6050_Read_All(&hi2c3, &MPU6050);
		copter_pitch_angle = MPU6050.KalmanAngleX;
		copter_roll_angle = MPU6050.KalmanAngleY;
		copter_yaw_angle+=MPU6050.Gz*SAMPLE_TIME;
		*/
		//////////////DANE Z BNO(SUROWE+WEWNĘTRZNA FUZJA)//////////
		bno055_vector_t acc= bno055_getVectorAccelerometer();

//		ac_x=acc.x;
//	    ac_x  = atan2f(acc.y, acc.z) * 57.2958;
//	    ac_x_f= 0.7f*acc.x+0.3f*ac_x_f;
//	    ac_y=acc.y;
//	    ac_y = atan2f(-acc.x, sqrtf(acc.y*acc.y + acc.z*acc.z)) * 57.2958;
//	    ac_y_f= 0.7f*acc.y+0.3f*ac_y_f;
		bno055_vector_t gyro= bno055_getVectorGyroscope();
//		gyr_x=0.8f*gyro.x+0.2f*gyr_x;
//		gyr_y=0.8f*gyro.y+0.2f*gyr_y;;
//		IMU_Fusion_Update(&imu_angles, acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, SAMPLE_TIME);
//
//		copter_pitch_angle_komp = -imu_angles.pitch;
//		copter_roll_angle_komp = -imu_angles.roll;

		Kalman_Update(&copter_roll_angle_kal, &copter_pitch_angle_kal, &Roll, &Pitch, acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z);

		////////////////DANE Z BNO(ZEWNĘTRZNA FUZJA)///////////////////


		bno_vector = bno055_getVectorEuler();
		copter_pitch_angle = copter_pitch_angle_kal;
		copter_roll_angle = copter_roll_angle_kal;

//		copter_pitch_angle = bno_vector.y;
//		copter_roll_angle = bno_vector.z;
		copter_yaw_angle = bno_vector.x;

		//////////////////OBLICZANIE WYJŚCIA REGULATORA WYKORZYSTUJĄC ERROR ORAZ REF ANGLE//////////////////
		if(SS==1){
			speed_pitch_ref = PID_Controller_Bartek_s_Lab(&pid_pitch,REF_PITCH_ANGLE,
					copter_pitch_angle);
			speed_roll_ref = PID_Controller_Bartek_s_Lab(&pid_roll, REF_ROLL_ANGLE,
					copter_roll_angle);
			speed_yaw_ref = PID_Controller_Bartek_s_Lab(&pid_yaw, REF_YAW_ANGLE,
					copter_yaw_angle);


			////////////////3DOF/////////////////////

			if(pid_z_counter!=0){
				speed_1_ref = (uint16_t) (SPEED_OFFSET + speed_roll_ref
						+ speed_pitch_ref +speed_yaw_ref);
				speed_2_ref = (uint16_t) (SPEED_OFFSET + speed_roll_ref
						- speed_pitch_ref -speed_yaw_ref);
				speed_3_ref = (uint16_t) (SPEED_OFFSET - speed_roll_ref
						- speed_pitch_ref +speed_yaw_ref);
				speed_4_ref = (uint16_t) (SPEED_OFFSET - speed_roll_ref
						+ speed_pitch_ref -speed_yaw_ref);
			}
//
//
//
//			////////////////4DOF/////////////////////
			else {
				speed_z_ref = PID_Controller_Bartek_s_Lab(&pid_z, REF_Z_DISTANCE,
										copter_z_distance);

//				speed_1_ref = (uint16_t) (SPEED_OFFSET + speed_roll_ref
//						+ speed_pitch_ref +speed_yaw_ref+speed_z_ref);
//				speed_2_ref = (uint16_t) (SPEED_OFFSET + speed_roll_ref
//						- speed_pitch_ref -speed_yaw_ref+speed_z_ref);
//				speed_3_ref = (uint16_t) (SPEED_OFFSET - speed_roll_ref
//						- speed_pitch_ref +speed_yaw_ref+speed_z_ref);
//				speed_4_ref = (uint16_t) (SPEED_OFFSET - speed_roll_ref
//						+ speed_pitch_ref -speed_yaw_ref+speed_z_ref);

				speed_1_ref = (uint16_t) (SPEED_OFFSET + speed_roll_ref
						+ speed_pitch_ref +speed_yaw_ref);
				speed_2_ref = (uint16_t) (SPEED_OFFSET + speed_roll_ref
						- speed_pitch_ref -speed_yaw_ref);
				speed_3_ref = (uint16_t) (SPEED_OFFSET - speed_roll_ref
						- speed_pitch_ref +speed_yaw_ref);
				speed_4_ref = (uint16_t) (SPEED_OFFSET - speed_roll_ref
						+ speed_pitch_ref -speed_yaw_ref);
			}

//
			if (ref_signal_counter==500){
				num_ref++;
				ref_signal_counter=0;
			}

			////////////////2DOF/////////////////////

//			speed_1_ref = (uint16_t) (SPEED_OFFSET + speed_roll_ref
//					+ speed_pitch_ref);
//			speed_2_ref = (uint16_t) (SPEED_OFFSET + speed_roll_ref
//					- speed_pitch_ref);
//			speed_3_ref = (uint16_t) (SPEED_OFFSET - speed_roll_ref
//					- speed_pitch_ref);
//			speed_4_ref = (uint16_t) (SPEED_OFFSET - speed_roll_ref
//					+ speed_pitch_ref);


			/////////////1DOF///////////////////////////

//			speed_1_ref = (uint16_t) (SPEED_OFFSET + speed_roll_ref);
//			speed_2_ref = (uint16_t) (SPEED_OFFSET + speed_roll_ref);
//			speed_3_ref = (uint16_t) (SPEED_OFFSET - speed_roll_ref);
//			speed_4_ref = (uint16_t) (SPEED_OFFSET - speed_roll_ref);

			///////////////SPRAWDZENIE CZY PRĘDKOŚCI MIESZCZĄ SIĘ W ZAKRESIE//////////
			// Double-check :))
			if (speed_1_ref < 48)
			{
				speed_1_ref = 48;
			}
			else if (speed_1_ref > MAX_SPEED)
			{
				speed_1_ref = MAX_SPEED;
			}
			else
			{
				__NOP();
			}

			if (speed_2_ref < 48)
			{
				speed_2_ref = 48;
			}
			else if (speed_2_ref > MAX_SPEED)
			{
				speed_2_ref = MAX_SPEED;
			}
			else
			{
				__NOP();
			}

			if (speed_3_ref < 48)
			{
				speed_3_ref = 48;
			}
			else if (speed_3_ref > MAX_SPEED)
			{
				speed_3_ref = MAX_SPEED;
			}
			else
			{
				__NOP();
			}

			if (speed_4_ref < 48)
			{
				speed_4_ref = 48;
			}
			else if (speed_4_ref > MAX_SPEED)
			{
				speed_4_ref = MAX_SPEED;
			}
			else
			{
				__NOP();
			}
			/////////WYSYŁANIE PRĘDKOŚCI DO ESC////////////
			speeds[0] = speed_1_ref;
			speeds[1] = speed_2_ref;
			speeds[2] = speed_3_ref;
			speeds[3] = speed_4_ref;
			dshot_send_all_ref_speeds(speeds);
//			base[0]=speed_test;
//			base[1]=speed_test;
//			base[2]=speed_test;
//			base[3]=speed_test;
//			dshot_send_all_ref_speeds(base);
			ref_signal_counter++;
			if(pid_z_counter<700 && pid_z_counter!=0)
			{
				pid_z_counter++;
			}
			else{
				pid_z_counter=0;
			}

		}



		else if (SS==2){
			__NOP();
		}


		else{
			base[0]=speed;
			base[1]=speed;
			base[2]=speed;
			base[3]=speed;
			dshot_send_all_ref_speeds(base);
		}

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		esp32_data_received_flag = 1;
		HAL_UART_Receive_IT(huart, rx_esp32_data, ESP32_MSG_LENGTH);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* Obsługa HC-SR04 (ECHO) */
    HCSR04_ProcessExti(&hcsr, GPIO_Pin);

}


void REF_SIGNAL(volatile uint8_t num) {
    if (num < sizeof(ref_signals)/sizeof(ref_signals[0])) {
        if (num == 11) {
            SS = 0;
        } else {
            REF_PITCH_ANGLE = ref_signals[num][0];
            REF_ROLL_ANGLE = ref_signals[num][1];
            REF_YAW_ANGLE = ref_signals[num][2];
            REF_Z_DISTANCE = ref_signals[num][3];
        }
    } else {
        num_ref = 12;
        num_ref_prev = 12;
    }
}
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
#ifdef USE_FULL_ASSERT
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

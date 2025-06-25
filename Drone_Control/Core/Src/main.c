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
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dshot150.h"
#include "mpu6050.h"
#include "stdio.h"//printf function
#include "pid_controller.h"
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESP32_UART_HANDLE &huart1
#define ESP32_MSG_LENGTH 6

#define SAMPLE_TIME 0.01f
#define PID_KP_MIN 7.0f
#define PID_KP_MAX 15.0f
#define PID_KI_MIN 7.0f
#define PID_KI_MAX 15.0f
#define PID_KD_MIN 5.0f
#define PID_KD_MAX 10.0f
#define PID_TAU_MIN 0.02f
#define PID_TAU_MAX 0.06f
#define REF_PITCH_ANGLE 0.0f
#define REF_ROLL_ANGLE 0.0f

#define SPEED_MIN 48
#define MAX_SPEED 1500

#define SPEED_OFFSET 1200.0f
#define ADC_TIMEOUT 1   // us

#define speed 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_reading;
uint16_t speed_ref[DSHOT_NUM_MOTORS]={speed,speed,speed,speed};
uint16_t accel=0;
uint16_t kali[3]={0,0,0};

volatile uint8_t white_button_flag = 0;

//PID
PID_t pid_pitch;
PID_t pid_roll;

float kp = PID_KP_MIN;
float ki = PID_KI_MIN;
float kd = PID_KD_MIN;
float tau = PID_TAU_MIN;

volatile uint8_t emergency_stop_flag = 0;
volatile float copter_pitch_angle;
volatile float copter_roll_angle;
volatile float copter_yaw_angle;
volatile float copter_yaw_angle_intergral;
volatile float speed_pitch_ref;
volatile float speed_roll_ref;

volatile uint8_t Trig_counter=0;

volatile uint16_t speed_1_ref;
volatile uint16_t speed_2_ref;
volatile uint16_t speed_3_ref;
volatile uint16_t speed_4_ref;

uint8_t calculated_crc = 0;

uint8_t uart_line[64];
int uart_line_length;

uint8_t esp32_data_received_flag = 0;
uint8_t rx_esp32_data[ESP32_MSG_LENGTH];
uint32_t UartDebugSoftTimer;

MPU6050_t MPU6050;

volatile uint32_t echo_end = 0;
volatile uint8_t echo_captured = 0;
volatile uint32_t last_trigger = 0;
float distance_cm=0;
float gyro_z_offset = 0;

float R[3][3] = {
		{0.9191, 0.3896, -0.0455},
		   {-0.3900, 0.9200, 0},
		    {0.0419, 0.0177, 0.9990}
};
float ToDrone[3]={0,0,0};
void HCSR04_Trigger(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

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
  MPU6050_Init(&hi2c3);


  HAL_TIM_Base_Start(&htim2); //Czas do czujników odległościowych
  HAL_TIM_Base_Start_IT(&htim15); //Timer od częstotliwości regulatora i wysyłania prędkości do drona
  HAL_TIM_Base_Start_IT(&htim1); // control loop interrupt


  // PID controllers
  	PID_Init_Bartek_s_Lab(&pid_pitch, PID_KP_MIN, PID_KI_MIN, PID_KD_MIN,
  	PID_TAU_MIN, -300.0f, 300.0f, SAMPLE_TIME);

  	PID_Init_Bartek_s_Lab(&pid_roll, PID_KP_MIN, PID_KI_MIN, PID_KD_MIN,
  	PID_TAU_MIN, -300.0f, 300.0f, SAMPLE_TIME);



  	UartDebugSoftTimer = HAL_GetTick();

  	/////////////////////gyro offset calculation///////////////

  	for (int i = 0; i < 1500; i++) {
  		MPU6050_Read_All(&hi2c3, &MPU6050);
  	    gyro_z_offset += MPU6050.Gz;
  	    HAL_Delay(2); // np. 2 ms między pomiarami
  	}
  	gyro_z_offset /= 1500;
  	copter_yaw_angle_intergral=0;


	HAL_Delay(2000); // let the motor stop after uC RST

	dshot_arm_all_esc();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  	memset(rx_esp32_data, 0x00, ESP32_MSG_LENGTH); //Zeruje bufor rx_esp32_data
  	HAL_UART_Receive_IT(ESP32_UART_HANDLE, rx_esp32_data,ESP32_MSG_LENGTH); // uruchamia odbieranie danych przez UART w trybie przerwań. Kiedy dane przyjdą, zostanie wywołane HAL_UART_RxCpltCallback.
	while (1)
	{
		/*
		if (HAL_GetTick() - last_trigger > 60) { // max 15 Hz
		        last_trigger = HAL_GetTick();
		        HCSR04_Trigger();
		}

		if (echo_captured) {
		    echo_captured = 0;
		    uint32_t diff = (echo_end >= echo_start) ? (echo_end - echo_start) : (0xFFFF - echo_start + echo_end);
		    distance_cm = diff * 0.0343f / 2.0f; // 343 m/s → 0.0343 cm/µs
		    printf("Dystans: %.2f cm\n", distance_cm);
		}
		*/

		if (white_button_flag == 1)
		{
			white_button_flag = 0;
			__NOP();
		}
		if (esp32_data_received_flag == 1)
		{
			esp32_data_received_flag = 0;
			calculated_crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) rx_esp32_data,
					(uint32_t) 6); //Liczenie CRC8 z SAEJ1850 standardu (0x1D poli oraz 0xFF ini)
			if (calculated_crc == 0)
			{
				switch (rx_esp32_data[0])
				{
				case 0x01: // kp
					kp = PID_KP_MIN
							+ (PID_KP_MAX - PID_KP_MIN) * rx_esp32_data[1]
									/ 100;
					break;
				case 0x02: // ki
					ki = PID_KI_MIN
							+ (PID_KI_MAX - PID_KI_MIN) * rx_esp32_data[1]
									/ 100;
					break;
				case 0x03: // kd
					kd = PID_KD_MIN
							+ (PID_KD_MAX - PID_KD_MIN) * rx_esp32_data[1]
									/ 100;
					break;
				case 0x04: // tau
					tau = PID_TAU_MIN
							+ (PID_TAU_MAX - PID_TAU_MIN) * rx_esp32_data[1]
									/ 100;
					break;
				default:
					;
				}

				PID_Controller_Update_Gains(&pid_pitch, kp, ki, kd, tau);
				PID_Controller_Update_Gains(&pid_roll, kp, ki, kd, tau);
			}


		}

		PID_Controller_Update_Gains(&pid_pitch, kp, ki, kd, tau);
		PID_Controller_Update_Gains(&pid_roll, kp, ki, kd, tau);

		//dshot_send_all_ref_speeds(speed_ref);

		MPU6050_Read_All(&hi2c3, &MPU6050);
		AngX= MPU6050.KalmanAngleX;
		AngY= MPU6050.KalmanAngleY;

		printf("Roll: %.2f", AngX);
		printf("Roll\r\n");

		kali[0]= MPU6050.Accel_X_RAW;
		kali[1]= MPU6050.Accel_Y_RAW;
		kali[2]= MPU6050.Accel_Z_RAW;

		ToDrone[0]=0;
		ToDrone[1]=0;
		ToDrone[2]=0;

		for (int i=0; i<3;i++){
			for(int j=0; j<3; j++){
				ToDrone[i]+=R[i][j]*kali[j];
			}
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

	if (htim->Instance == TIM15) //sprawdzenie od którego timera jest przerwanie
	{
		if (Trig_counter>5){
			HCSR04_Trigger();
			distance_cm = echo_end * 0.0343f / 2.0f;
			printf("Dystans: %.2f cm\n", distance_cm);
			Trig_counter=0;
		}
		Trig_counter++;

		/////////////POBIERANIE WYCHYLENIA Z CZUJNIKA MPU6050/////////////
		copter_pitch_angle = MPU6050.KalmanAngleX;
		copter_roll_angle = MPU6050.KalmanAngleY;
		copter_yaw_angle = MPU6050.Gz-gyro_z_offset;
		copter_yaw_angle_intergral+=copter_yaw_angle*SAMPLE_TIME;
		//////////////////OBLICZANIE WYJŚCIA REGULATORA WYKORZYSTUJĄC ERROR ORAZ REF ANGLE//////////////////
		speed_pitch_ref = PID_Controller_Bartek_s_Lab(&pid_pitch,
		REF_PITCH_ANGLE, copter_pitch_angle);
		speed_roll_ref = PID_Controller_Bartek_s_Lab(&pid_roll, REF_ROLL_ANGLE,
				copter_roll_angle);
		////////////////Z OBLICZONYCH PRĘDKOŚCI WZGLĘDEM
		speed_1_ref = (uint16_t) (SPEED_OFFSET + speed_roll_ref
				- speed_pitch_ref);
		speed_2_ref = (uint16_t) (SPEED_OFFSET + speed_roll_ref
				+ speed_pitch_ref);
		speed_3_ref = (uint16_t) (SPEED_OFFSET - speed_roll_ref
				+ speed_pitch_ref);
		speed_4_ref = (uint16_t) (SPEED_OFFSET - speed_roll_ref
				- speed_pitch_ref);
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
	}

}

uint8_t compute_crc8(uint8_t *data, uint8_t length) {
    uint8_t crc = 0xFF;
    uint8_t poly = 0x1D;

    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		esp32_data_received_flag = 1;
		HAL_UART_Receive_IT(huart, rx_esp32_data, ESP32_MSG_LENGTH);
	}
}

void HCSR04_Trigger(void) {
    HAL_GPIO_WritePin(TRIG_PIN_GPIO_Port, TRIG_PIN_Pin, GPIO_PIN_SET); // TRIG high
    for (volatile int i = 0; i < 400; i++) __NOP(); // ~50 µs
    HAL_GPIO_WritePin(TRIG_PIN_GPIO_Port, TRIG_PIN_Pin, GPIO_PIN_RESET); // TRIG low
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) { // ECHO
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) {
            // Zbocze narastające
        	__HAL_TIM_SET_COUNTER(&htim2, 0);
        	HAL_TIM_Base_Start(&htim2);
        } else {
            // Zbocze opadające
            echo_end = __HAL_TIM_GET_COUNTER(&htim2);
            HAL_TIM_Base_Stop(&htim2);
        }
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

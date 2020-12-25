/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "u8g2.h"
#include <stdio.h>

#include "display_prints.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define BOILER_TEMP_CAN_ID 0x0d0
#define BOILER_TEMP_CAN_ID 0x0d0
//#define RELAY_CONTROL_CAN_ID 0x080
#define RELAY_CONTROL_CAN_ID 0x090
//#define RELAY_STATUS_CAN_ID 0x081
#define RELAY_STATUS_CAN_ID 0x091


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static u8g2_t u8g2;

volatile uint32_t testCounter = 0;


struct display_screen1_data screen1_data;


int32_t temperature;
uint32_t pressure;
uint32_t humidity;
float input_packet_boiler_temperature;

volatile uint8_t input_packet_boiler_timeout = 0;

volatile uint8_t temperature1timeout = 0;
volatile uint8_t temperature2timeout = 0;
volatile uint8_t humidity1timeout = 0;
volatile uint8_t humidity2timeout = 0;

#define DATA_WAIT_TIMEOUT 10

CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

volatile uint16_t micro_delay_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len)
{
 HAL_UART_Transmit(&huart3, (uint8_t *) ptr, len, HAL_MAX_DELAY);
 return len;
}

void display_update(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* ******************************************************************************* */
void user_delay_ms(uint32_t period);


void user_delay_ms(uint32_t period)
{
	HAL_Delay(period);
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
}

void send_8080_data_delay_100u(uint16_t delay) {
//	HAL_Delay(1);
	micro_delay_counter = delay+1;
	while(micro_delay_counter > 0) {
	}
}

void send_8080_data(uint8_t arg_int, void *arg_ptr) {
	uint8_t quantity = arg_int;
	uint8_t * data_buffer = arg_ptr;
	for (uint8_t i = 0; i < quantity; i++) {
		uint8_t byte = data_buffer[i];

		HAL_GPIO_WritePin(DISPLAY_D0_GPIO_Port, DISPLAY_D0_Pin, byte & 0x01);
		HAL_GPIO_WritePin(DISPLAY_D1_GPIO_Port, DISPLAY_D1_Pin, byte & 0x02);
		HAL_GPIO_WritePin(DISPLAY_D2_GPIO_Port, DISPLAY_D2_Pin, byte & 0x04);
		HAL_GPIO_WritePin(DISPLAY_D3_GPIO_Port, DISPLAY_D3_Pin, byte & 0x08);

		HAL_GPIO_WritePin(DISPLAY_D4_GPIO_Port, DISPLAY_D4_Pin, byte & 0x10);
		HAL_GPIO_WritePin(DISPLAY_D5_GPIO_Port, DISPLAY_D5_Pin, byte & 0x20);
		HAL_GPIO_WritePin(DISPLAY_D6_GPIO_Port, DISPLAY_D6_Pin, byte & 0x40);
		HAL_GPIO_WritePin(DISPLAY_D7_GPIO_Port, DISPLAY_D7_Pin, byte & 0x80);

		HAL_GPIO_WritePin(DISPLAY_WR_GPIO_Port, DISPLAY_WR_Pin, GPIO_PIN_RESET);

		send_8080_data_delay_100u(1);
		HAL_GPIO_WritePin(DISPLAY_WR_GPIO_Port, DISPLAY_WR_Pin, GPIO_PIN_SET);
		send_8080_data_delay_100u(1);
	}
}

uint8_t u8x8_byte_8080(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
    	send_8080_data(arg_int, arg_ptr);

      break;
    case U8X8_MSG_BYTE_INIT:
      break;
    case U8X8_MSG_BYTE_SET_DC:
    	HAL_GPIO_WritePin(DISPLAY_A0_GPIO_Port, DISPLAY_A0_Pin, arg_int);

      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
    	HAL_GPIO_WritePin(DISPLAY_CS_GPIO_Port, DISPLAY_CS_Pin, GPIO_PIN_RESET);

      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
    	HAL_GPIO_WritePin(DISPLAY_CS_GPIO_Port, DISPLAY_CS_Pin, GPIO_PIN_SET);

      break;
    default:
      return 0;
  }
  return 1;
}

uint8_t u8x8_gpio_and_delay_8080(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:	// called once during init phase of u8g2/u8x8
    	  HAL_GPIO_WritePin(DISPLAY_RD_GPIO_Port, DISPLAY_RD_Pin, GPIO_PIN_SET);
    	  HAL_GPIO_WritePin(DISPLAY_CS_GPIO_Port, DISPLAY_CS_Pin, GPIO_PIN_SET);
    	  HAL_Delay(1);

    	break;							// can be used to setup pins
    case U8X8_MSG_DELAY_NANO:			// delay arg_int * 1 nano second
      break;
    case U8X8_MSG_DELAY_100NANO:		// delay arg_int * 100 nano seconds
      break;
    case U8X8_MSG_DELAY_10MICRO:		// delay arg_int * 10 micro seconds
      break;
    case U8X8_MSG_DELAY_MILLI:			// delay arg_int * 1 milli second
    	HAL_Delay(arg_int);

    	break;
    case U8X8_MSG_DELAY_I2C:				// arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
      break;							// arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
    case U8X8_MSG_GPIO_D0:				// D0 or SPI clock pin: Output level in arg_int
    	HAL_GPIO_WritePin(DISPLAY_D0_GPIO_Port, DISPLAY_D0_Pin, arg_int);

    	break;
    case U8X8_MSG_GPIO_D1:				// D1 or SPI data pin: Output level in arg_int
    	HAL_GPIO_WritePin(DISPLAY_D1_GPIO_Port, DISPLAY_D1_Pin, arg_int);

    //case U8X8_MSG_GPIO_SPI_DATA:
      break;
    case U8X8_MSG_GPIO_D2:				// D2 pin: Output level in arg_int
    	HAL_GPIO_WritePin(DISPLAY_D2_GPIO_Port, DISPLAY_D2_Pin, arg_int);

      break;
    case U8X8_MSG_GPIO_D3:				// D3 pin: Output level in arg_int
    	HAL_GPIO_WritePin(DISPLAY_D3_GPIO_Port, DISPLAY_D3_Pin, arg_int);

      break;
    case U8X8_MSG_GPIO_D4:				// D4 pin: Output level in arg_int
    	HAL_GPIO_WritePin(DISPLAY_D4_GPIO_Port, DISPLAY_D4_Pin, arg_int);

      break;
    case U8X8_MSG_GPIO_D5:				// D5 pin: Output level in arg_int
    	HAL_GPIO_WritePin(DISPLAY_D5_GPIO_Port, DISPLAY_D5_Pin, arg_int);

    	break;
    case U8X8_MSG_GPIO_D6:				// D6 pin: Output level in arg_int
    	HAL_GPIO_WritePin(DISPLAY_D6_GPIO_Port, DISPLAY_D6_Pin, arg_int);

      break;
    case U8X8_MSG_GPIO_D7:				// D7 pin: Output level in arg_int
    	HAL_GPIO_WritePin(DISPLAY_D7_GPIO_Port, DISPLAY_D7_Pin, arg_int);

      break;
    case U8X8_MSG_GPIO_E:				// E/WR pin: Output level in arg_int
    	HAL_GPIO_WritePin(DISPLAY_WR_GPIO_Port, DISPLAY_WR_Pin, arg_int);

      break;
    case U8X8_MSG_GPIO_CS:				// CS (chip select) pin: Output level in arg_int
    	HAL_GPIO_WritePin(DISPLAY_CS_GPIO_Port, DISPLAY_CS_Pin, arg_int);

      break;
    case U8X8_MSG_GPIO_DC:				// DC (data/cmd, A0, register select) pin: Output level in arg_int
    	HAL_GPIO_WritePin(DISPLAY_A0_GPIO_Port, DISPLAY_A0_Pin, arg_int);

    	break;
    case U8X8_MSG_GPIO_RESET:			// Reset pin: Output level in arg_int
    	HAL_GPIO_WritePin(DISPLAY_RESET_GPIO_Port, DISPLAY_RESET_Pin, arg_int);

    	break;
    case U8X8_MSG_GPIO_CS1:				// CS1 (chip select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_CS2:				// CS2 (chip select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_I2C_CLOCK:		// arg_int=0: Output low at I2C clock pin
      break;							// arg_int=1: Input dir with pullup high for I2C clock pin
    case U8X8_MSG_GPIO_I2C_DATA:			// arg_int=0: Output low at I2C data pin
      break;							// arg_int=1: Input dir with pullup high for I2C data pin
    case U8X8_MSG_GPIO_MENU_SELECT:
//      u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_NEXT:
//      u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_PREV:
//      u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_HOME:
//      u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
      break;
    default:
//      u8x8_SetGPIOResult(u8x8, 1);			// default return value
      break;
  }
  return 1;
}

//uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
//    U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
//    U8X8_UNUSED void *arg_ptr)
//{
//  switch (msg)
//  {
//  case U8X8_MSG_GPIO_AND_DELAY_INIT:
//    HAL_Delay(1);
//    break;
//  case U8X8_MSG_DELAY_MILLI:
//    HAL_Delay(arg_int);
//    break;
//  case U8X8_MSG_GPIO_DC:
////    HAL_GPIO_WritePin(DISPLAY_DC_GPIO_Port, DISPLAY_DC_Pin, arg_int);
//    HAL_GPIO_WritePin(DISPLAY_A0_GPIO_Port, DISPLAY_A0_Pin, arg_int);
//    break;
//  case U8X8_MSG_GPIO_RESET:
//    HAL_GPIO_WritePin(DISPLAY_RESET_GPIO_Port, DISPLAY_RESET_Pin, arg_int);
//    break;
//  }
//  return 1;
//}
//
//uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
//    void *arg_ptr)
//{
//  switch (msg)
//  {
//  case U8X8_MSG_BYTE_SEND:
//    HAL_SPI_Transmit(&hspi1, (uint8_t *) arg_ptr, arg_int, 10000);
//    break;
//  case U8X8_MSG_BYTE_INIT:
//    break;
//  case U8X8_MSG_BYTE_SET_DC:
//    HAL_GPIO_WritePin(DISPLAY_DC_GPIO_Port, DISPLAY_DC_Pin, arg_int);
//    break;
//  case U8X8_MSG_BYTE_START_TRANSFER:
//		HAL_GPIO_WritePin(CS1_DISPLAY_GPIO_Port, CS1_DISPLAY_Pin, GPIO_PIN_RESET);
//    break;
//  case U8X8_MSG_BYTE_END_TRANSFER:
//		HAL_GPIO_WritePin(CS1_DISPLAY_GPIO_Port, CS1_DISPLAY_Pin, GPIO_PIN_SET);
//    break;
//  default:
//    return 0;
//  }
//  return 1;
//}

void display_update(void) {

	int16_t tempInt;
	uint32_t tempUInt;
	int16_t tempDec;

	u8g2_SetFont(&u8g2, u8g2_font_logisoso18_tr );

	char tmp_string[100];
	u8g2_FirstPage(&u8g2);
	do
	 {

		////// boiler temp
		if(input_packet_boiler_timeout == DATA_WAIT_TIMEOUT){
			sprintf(tmp_string, "B xxx");
		}else{
//			tempInt = (int16_t)(input_packet_boiler_temperature);
//			tempDec = (uint16_t)(input_packet_boiler_temperature%1);
			sprintf(tmp_string, "B %.2f", input_packet_boiler_temperature);
		}
		u8g2_DrawStr(&u8g2, 0, 20, tmp_string);

		//////t1
		if(temperature1timeout == DATA_WAIT_TIMEOUT){
			sprintf(tmp_string, "T xxx");
		}else{
			tempInt = (int16_t)(temperature/100);
			tempDec = (uint16_t)(temperature%100);
			sprintf(tmp_string, "T %d,%u", tempInt, tempDec);
		}
		u8g2_DrawStr(&u8g2, 0, 45, tmp_string);


		//////h1
		if(humidity1timeout == DATA_WAIT_TIMEOUT){
			sprintf(tmp_string, "H xxx");
		}else{
			tempUInt = (uint32_t)(humidity/1024);
			tempDec = (uint16_t)(humidity%1024);
			sprintf(tmp_string, "H %lu,%u", tempUInt, tempDec);
			printf(tmp_string);
		}
		u8g2_DrawStr(&u8g2, 0, 70, tmp_string);


		//////t2
		if(temperature2timeout == DATA_WAIT_TIMEOUT){
			sprintf(tmp_string, "T xxx");
		}else{
			tempInt = (int16_t)(temperature/100);
			tempDec = (uint16_t)(temperature%100);
			sprintf(tmp_string, "T %u,%u", tempInt, tempDec);
		}
		u8g2_DrawStr(&u8g2, 0, 100, tmp_string);


		//////h2
		if(humidity2timeout == DATA_WAIT_TIMEOUT){
			sprintf(tmp_string, "H xxx");
		}else{
			tempUInt = (uint32_t)(humidity/1024);
			tempDec = (uint16_t)(humidity%1024);
		}
		u8g2_DrawStr(&u8g2, 0, 125, tmp_string);

	 } while (u8g2_NextPage(&u8g2));
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
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* off realays */

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
	  /* Filter configuration Error */
	  Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
	  /* Start Error */
	  Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
	  /* Notification Error */
	  Error_Handler();
  }


  TxHeader.StdId = BOILER_TEMP_CAN_ID;
  TxHeader.ExtId = 0x00;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 4;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxData[0] = 1;
  TxData[1] = 2;
  TxData[2] = 3;
  TxData[3] = 4;


  /* display init >> */

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);




  /* turn on backlight */
  HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(DISPLAY_RD_GPIO_Port, DISPLAY_RD_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DISPLAY_CS_GPIO_Port, DISPLAY_CS_Pin, GPIO_PIN_SET);

  // --->>>

    u8g2_Setup_st7565_ea_dogm128_1(&u8g2, U8G2_R2, u8x8_byte_8080, u8x8_gpio_and_delay_8080);
//  u8g2_Setup_st7565_zolen_128x64_1(&u8g2, U8G2_R2, u8x8_byte_8080, u8x8_gpio_and_delay_8080);// bad and mirrored
//  u8g2_Setup_st7565_lx12864_1(&u8g2, U8G2_R2, u8x8_byte_8080, u8x8_gpio_and_delay_8080); //bad
//  u8g2_Setup_st7565_erc12864_1(&u8g2, U8G2_R2, u8x8_byte_8080, u8x8_gpio_and_delay_8080);// not good and white
//  u8g2_Setup_st7565_erc12864_alt_1(&u8g2, U8G2_R2, u8x8_byte_8080, u8x8_gpio_and_delay_8080); //bad
//  u8g2_Setup_st7565_nhd_c12864_1(&u8g2, U8G2_R2, u8x8_byte_8080, u8x8_gpio_and_delay_8080); //bad but in the right position
//  u8g2_Setup_st7565_jlx12864_1(&u8g2, U8G2_R2, u8x8_byte_8080, u8x8_gpio_and_delay_8080);//bad but in the right position

//  u8g2_Setup_st7565_ea_dogm128_1
//  /* default_x_offset = */ 0, fix -> to 4
//  /* flipmode_x_offset = */ 4, fix -> to 0

//  u8g2_Setup_st7565_nhd_c12864_1
  //  /* default_x_offset = */ 4,
  //  /* flipmode_x_offset = */ 0,


//  u8g2_Setup_ssd1306_128x64_noname_1(&u8g2,
//									 U8G2_R1,
//									 u8x8_byte_4wire_hw_spi,
//									 u8x8_stm32_gpio_and_delay);

    // ---<<<
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_SetContrast(&u8g2, 95);

  /* display init << */

	char tmp_string[20];

	memset(tmp_string, 0, 20);

  u8g2_FirstPage(&u8g2);
  do
	 {
	    u8g2_SetFont(&u8g2, u8g2_font_luBS08_tf);
//	    u8g2_font_luRS08_tf

	    //	    u8g2_font_ncenB14_tr


		u8g2_DrawStr(&u8g2, 0, 14, "Hello world ");
	 } while (u8g2_NextPage(&u8g2));

  HAL_GPIO_WritePin(CAN_ENABLE_GPIO_Port, CAN_ENABLE_Pin, GPIO_PIN_RESET);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_Delay(2000);
//	  send_8080_data_delay_100u(50000);
//	  struct bme280_data comp_data;
//	  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
//      sprintf(tmp_string, "%ld, %ld, %ld\r\n",comp_data.temperature, comp_data.pressure, comp_data.humidity);

	  HAL_Delay(100);
//	    HAL_GPIO_TogglePin(RELAY0_GPIO_Port, RELAY0_Pin);
//	    HAL_GPIO_TogglePin(GPIOB, DISPLAY_RESET_Pin);

	  testCounter++;
	  memset(tmp_string, 0, 20);


	  u8g2_FirstPage(&u8g2);
	  do
	 {
//		u8g2_font_luRS08_tf
//		u8g2_font_luBS08_tf
//		u8g2_font_luIS08_tf
		  sprintf(tmp_string, "testCounter:%lu", (uint32_t)testCounter);
		  u8g2_SetFont(&u8g2, u8g2_font_luBS08_tf);
		  u8g2_DrawStr(&u8g2, 0, 10, tmp_string);

		  u8g2_SetFont(&u8g2, u8g2_font_luRS08_tf);

//		  u8g2_font_5x7_tf
//		  u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
		  u8g2_SetFont(&u8g2, u8g2_font_courR08_tf);

		  sprintf(tmp_string, "temp:%lu%sC", testCounter,"\xb0");
		  u8g2_DrawStr(&u8g2, 0, 24, tmp_string);
//		  u8g2_get("\xb0");
		  sprintf(tmp_string, "hm:%lu", testCounter);
		  u8g2_DrawStr(&u8g2, 0, 40, tmp_string);
	 } while (u8g2_NextPage(&u8g2));




	    HAL_GPIO_TogglePin(GPIOC, LED_Pin);
	  	 printf("OK2!\r\n");
//	  display_update();

//	  sprintf(screen1_data.str_boiler_value, "ddd1");
//
//	  sprintf(screen1_data.str_tempreture_value_1, "ddd2");
//	  sprintf(screen1_data.str_humidity_value_1, "ddd3");
//
//	  sprintf(screen1_data.str_tempreture_value_2, "ddd4");
//	  sprintf(screen1_data.str_humidity_value_2, "ddd5");
//
//	  sprintf(screen1_data.str_pump_status_1, "ddd6");
//	  sprintf(screen1_data.str_pump_status_2, "ddd7");

//	  display_update2(&u8g2, &screen1_data);
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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL10;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 5;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1440;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 40;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISPLAY_D0_Pin|DISPLAY_D1_Pin|DISPLAY_D2_Pin|BACKLIGHT_Pin
                          |DISPLAY_A0_Pin|DISPLAY_RESET_Pin|DISPLAY_CS_Pin|DISPLAY_D3_Pin
                          |DISPLAY_D4_Pin|DISPLAY_D5_Pin|DISPLAY_D6_Pin|DISPLAY_D7_Pin
                          |DISPLAY_RD_Pin|DISPLAY_WR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_ENABLE_GPIO_Port, CAN_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_0_Pin BUTTON_1_Pin BUTTON_2_Pin BUTTON_3_Pin */
  GPIO_InitStruct.Pin = BUTTON_0_Pin|BUTTON_1_Pin|BUTTON_2_Pin|BUTTON_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DISPLAY_D0_Pin DISPLAY_D1_Pin DISPLAY_D2_Pin BACKLIGHT_Pin
                           DISPLAY_A0_Pin DISPLAY_RESET_Pin DISPLAY_CS_Pin DISPLAY_D3_Pin
                           DISPLAY_D4_Pin DISPLAY_D5_Pin DISPLAY_D6_Pin DISPLAY_D7_Pin
                           DISPLAY_RD_Pin DISPLAY_WR_Pin */
  GPIO_InitStruct.Pin = DISPLAY_D0_Pin|DISPLAY_D1_Pin|DISPLAY_D2_Pin|BACKLIGHT_Pin
                          |DISPLAY_A0_Pin|DISPLAY_RESET_Pin|DISPLAY_CS_Pin|DISPLAY_D3_Pin
                          |DISPLAY_D4_Pin|DISPLAY_D5_Pin|DISPLAY_D6_Pin|DISPLAY_D7_Pin
                          |DISPLAY_RD_Pin|DISPLAY_WR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_ENABLE_Pin */
  GPIO_InitStruct.Pin = CAN_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_ENABLE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan_)
{
//	HAL_GPIO_TogglePin(GPIOC, LED_Pin);
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_)
{
	HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
//	HAL_GPIO_TogglePin(GPIOC, LED_Pin);

	 uint32_t u;
	  u = RxData[0];
	  u += (uint32_t)(RxData[1] << 8);
	  u += (uint32_t)(RxData[2] << 16);
	  u += (uint32_t)(RxData[3] << 24);




	if(RxHeader.StdId==0xf0){
		pressure = u;
	}else if(RxHeader.StdId==0xf0+1){
		temperature = u;
		temperature1timeout = 0;
		temperature2timeout = 0;
	}else if(RxHeader.StdId==0xf0+2){
		humidity = u;
		humidity1timeout = 0;
		humidity2timeout = 0;
	}else if(RxHeader.StdId==RELAY_CONTROL_CAN_ID){
//		Control_peripheral_relays(RxData[0]);
	}else if(RxHeader.StdId==BOILER_TEMP_CAN_ID){
		input_packet_boiler_temperature = u;
		for (int i=0; i<4 ;++i) {
			((uint8_t*)&input_packet_boiler_temperature)[i] = RxData[i];
		}
		input_packet_boiler_timeout = 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if( htim->Instance == TIM1 ){
//	HAL_GPIO_TogglePin(GPIOC, LED_Pin);
	++temperature1timeout;
	++temperature2timeout;
	++humidity1timeout;
	++humidity2timeout;
	++input_packet_boiler_timeout;

	if(temperature1timeout>= DATA_WAIT_TIMEOUT){
		temperature1timeout = DATA_WAIT_TIMEOUT;
	}

	if(temperature2timeout>= DATA_WAIT_TIMEOUT){
		temperature2timeout = DATA_WAIT_TIMEOUT;
	}

	if(humidity1timeout>= DATA_WAIT_TIMEOUT){
		humidity1timeout = DATA_WAIT_TIMEOUT;
	}

	if(humidity2timeout>= DATA_WAIT_TIMEOUT){
		humidity2timeout = DATA_WAIT_TIMEOUT;
	}

	if(input_packet_boiler_timeout>= DATA_WAIT_TIMEOUT){
		input_packet_boiler_timeout = DATA_WAIT_TIMEOUT;
	}
 } else if ( htim->Instance == TIM2 ) {
	 if (micro_delay_counter > 0 ) {
		 micro_delay_counter--;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

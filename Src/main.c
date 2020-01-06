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
#include "bme280.h"
#include "u8g2.h"
#include <stdio.h>

#include "OneWire.h"
#include "DallasTemperature.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BOILER_TEMP_CAN_ID 0x0d0
#define RELAY_CONTROL_CAN_ID 0x080
#define RELAY_STATUS_CAN_ID 0x081


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
static u8g2_t u8g2;

int32_t temperature;
uint32_t pressure;
uint32_t humidity;

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

struct bme280_data comp_data;

volatile uint8_t relays_status = 0x00;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len)
{
 HAL_UART_Transmit(&huart3, (uint8_t *) ptr, len, HAL_MAX_DELAY);
 return len;
}

// function to print a device address
void printAddress(CurrentDeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
 char buf[4];
 sprintf(buf, "%02X ", deviceAddress[i]);
 printf(buf);
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* ******************************************************************************* */
struct bme280_dev dev;

int8_t setup_bme280(void);
void user_delay_ms(uint32_t period);
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);



int8_t setup_bme280(void) {
	int8_t rslt = BME280_OK;
	uint8_t settings_sel;

	/* Sensor_0 interface over SPI with native chip select line */
	dev.dev_id = 0;
	dev.intf = BME280_SPI_INTF;
	dev.read = user_spi_read;
	dev.write = user_spi_write;
	dev.delay_ms = user_delay_ms;

	rslt = bme280_init(&dev);


	/* Recommended mode of operation: Indoor navigation */
	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, &dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);

	return rslt;
}

void user_delay_ms(uint32_t period)
{
	HAL_Delay(period);
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
}

int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    uint32_t Timeout = 50;

    uint8_t temp_buffer_size = len+1;
    uint8_t temp_buffer_tx[temp_buffer_size];
    uint8_t temp_buffer_rx[temp_buffer_size];

    /* setting values for tx buffer */
    memset(temp_buffer_tx, 0, sizeof(temp_buffer_tx));
    temp_buffer_tx[0] = reg_addr;

    HAL_GPIO_WritePin(CS0_BME280_GPIO_Port, CS0_BME280_Pin, GPIO_PIN_RESET);
    rslt = HAL_SPI_TransmitReceive(&hspi1, temp_buffer_tx, temp_buffer_rx, temp_buffer_size, Timeout);
    HAL_GPIO_WritePin(CS0_BME280_GPIO_Port, CS0_BME280_Pin, GPIO_PIN_SET);

    /* copy received data into origin buffer */
    memcpy(reg_data, &temp_buffer_rx[1], len);

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |----------------+---------------------+-------------|
     * | MOSI           | MISO                | Chip Select |
     * |----------------+---------------------|-------------|
     * | (don't care)   | (don't care)        | HIGH        |
     * | (reg_addr)     | (don't care)        | LOW         |
     * | (don't care)   | (reg_data[0])       | LOW         |
     * | (....)         | (....)              | LOW         |
     * | (don't care)   | (reg_data[len - 1]) | LOW         |
     * | (don't care)   | (don't care)        | HIGH        |
     * |----------------+---------------------|-------------|
     */

    return rslt;
}

int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    uint32_t Timeout = 50;

    uint8_t temp_buffer_size = len+1;
    uint8_t temp_buffer[temp_buffer_size];

    /* setting values for tx buffer */
    temp_buffer[0] = reg_addr;
    memcpy(&temp_buffer[1], reg_data, len);

    HAL_GPIO_WritePin(CS0_BME280_GPIO_Port, CS0_BME280_Pin, GPIO_PIN_RESET);
    rslt = HAL_SPI_Transmit(&hspi1, temp_buffer, temp_buffer_size, Timeout);
    HAL_GPIO_WritePin(CS0_BME280_GPIO_Port, CS0_BME280_Pin, GPIO_PIN_SET);

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |---------------------+--------------+-------------|
     * | MOSI                | MISO         | Chip Select |
     * |---------------------+--------------|-------------|
     * | (don't care)        | (don't care) | HIGH        |
     * | (reg_addr)          | (don't care) | LOW         |
     * | (reg_data[0])       | (don't care) | LOW         |
     * | (....)              | (....)       | LOW         |
     * | (reg_data[len - 1]) | (don't care) | LOW         |
     * | (don't care)        | (don't care) | HIGH        |
     * |---------------------+--------------|-------------|
     */

    return rslt;
}

uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
    U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
    U8X8_UNUSED void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    HAL_Delay(1);
    break;
  case U8X8_MSG_DELAY_MILLI:
    HAL_Delay(arg_int);
    break;
  case U8X8_MSG_GPIO_DC:
    HAL_GPIO_WritePin(DISPLAY_DC_GPIO_Port, DISPLAY_DC_Pin, arg_int);
    break;
  case U8X8_MSG_GPIO_RESET:
    HAL_GPIO_WritePin(DISPLAY_RESET_GPIO_Port, DISPLAY_RESET_Pin, arg_int);
    break;
  }
  return 1;
}

uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
    void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_BYTE_SEND:
    HAL_SPI_Transmit(&hspi1, (uint8_t *) arg_ptr, arg_int, 10000);
    break;
  case U8X8_MSG_BYTE_INIT:
    break;
  case U8X8_MSG_BYTE_SET_DC:
    HAL_GPIO_WritePin(DISPLAY_DC_GPIO_Port, DISPLAY_DC_Pin, arg_int);
    break;
  case U8X8_MSG_BYTE_START_TRANSFER:
		HAL_GPIO_WritePin(CS1_DISPLAY_GPIO_Port, CS1_DISPLAY_Pin, GPIO_PIN_RESET);
    break;
  case U8X8_MSG_BYTE_END_TRANSFER:
		HAL_GPIO_WritePin(CS1_DISPLAY_GPIO_Port, CS1_DISPLAY_Pin, GPIO_PIN_SET);
    break;
  default:
    return 0;
  }
  return 1;
}


void Control_peripheral_relays(uint8_t flag_holder) {
	uint8_t flag = 0;

	/* control relays */
	flag = flag_holder & 0x01;
	if (flag != 0) {
		/* on */
		HAL_GPIO_WritePin(RELAY0_GPIO_Port, RELAY0_Pin, GPIO_PIN_RESET);
	} else {
		/* off */
		HAL_GPIO_WritePin(RELAY0_GPIO_Port, RELAY0_Pin, GPIO_PIN_SET);
	}

	relays_status = flag_holder;
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
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* off realays */
  HAL_GPIO_WritePin(RELAY0_GPIO_Port, RELAY0_Pin, GPIO_PIN_SET);



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


  HAL_GPIO_WritePin(CS0_BME280_GPIO_Port, CS0_BME280_Pin, GPIO_PIN_SET);
  setup_bme280();


  HAL_GPIO_WritePin(CS1_DISPLAY_GPIO_Port, CS1_DISPLAY_Pin, GPIO_PIN_SET);
  u8g2_Setup_ssd1306_128x64_noname_1(&u8g2, U8G2_R1, u8x8_byte_4wire_hw_spi,
      u8x8_stm32_gpio_and_delay);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
//  int8_t rslt;
  HAL_TIM_Base_Start_IT(&htim1);



  	 uint8_t ds_error = 0;

  	 printf("Debug UART is OK!\r\n");

//	if(OW_Reset() == OW_OK)
//	{
//		printf("OneWire devices are present :)\r\n");
//	}
//	else
//	{
//		printf("OneWire no devices :(\r\n");
//	}

	// arrays to hold device address
	CurrentDeviceAddress insideThermometer;

	// locate devices on the bus
	char buf[30];

	printf("Locating devices...");
	ds_error = DT_Begin();
	if (ds_error != DS_OK) {
		sprintf(buf, "DT_Begin err %d", ds_error);
		printf(buf);
	}

	printf("Found ");
	sprintf(buf, "%d", DT_GetDeviceCount());
	printf(buf);
	printf(" devices.\r\n");

	// report parasite power requirements
	printf("Parasite power is: ");
	if (DT_IsParasitePowerMode()) printf("ON\r\n");
	else printf("OFF\r\n");

	if (!DT_GetAddress(insideThermometer, 0)) printf("Unable to find address for Device 0\r\n");

	printf("Device 0 Address: ");
	printAddress(insideThermometer);
	printf("\r\n");

	// set the resolution to 12 bit (Each Dallas/Maxim device is capable of several different resolutions)
	DT_SetResolution(insideThermometer, 12, true);

	printf("Device 0 Resolution: ");
	sprintf(buf, "%d", DT_GetResolution(insideThermometer));
	printf(buf);
	printf("\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	char tmp_string[20];

	memset(tmp_string, 0, 20);

    u8g2_FirstPage(&u8g2);
    do
	 {
	    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
		u8g2_DrawStr(&u8g2, 5, 30, "Hello world ");
	 } while (u8g2_NextPage(&u8g2));
	u8g2_SetFont(&u8g2, u8g2_font_logisoso18_tr );

	int16_t tempInt;
	uint32_t tempUInt;
	int16_t tempDec;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // call sensors.requestTemperatures() to issue a global temperature
	  // request to all devices on the bus
	  printf("Requesting temperatures...");
	  DT_RequestTemperatures(); // Send the command to get temperatures
	  printf("DONE\r\n");
	  // After we got the temperatures, we can print them here.
	  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
	  printf("Temperature for the device 1 (index 0) is: ");
	  float ds_temperature = DT_GetTempCByIndex(0);
	  sprintf(buf, "%.2f\r\n", ds_temperature);
	  printf(buf);

	  TxHeader.StdId = BOILER_TEMP_CAN_ID;
	  TxHeader.DLC = 4;

	  for (int i=0; i<4 ;++i) {
		  TxData[i] = ((uint8_t*)&ds_temperature)[i];
	  }

	  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

	  HAL_Delay(2500);
	  /* send relays status */
	  TxHeader.StdId = RELAY_STATUS_CAN_ID;
	  TxHeader.DLC = 1;
	  TxData[0] = relays_status;
	  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
	  HAL_Delay(2500);


//	  struct bme280_data comp_data;
//	  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
//      sprintf(tmp_string, "%ld, %ld, %ld\r\n",comp_data.temperature, comp_data.pressure, comp_data.humidity);

	  uint8_t test_uart_data[2] = { 1, 8 };
	  HAL_UART_Transmit(&huart3, test_uart_data, sizeof(test_uart_data), 50);

	  HAL_Delay(1000);
//	  TxData[7] = TxData[7] + 1;

//	    HAL_GPIO_TogglePin(GPIOC, LED_Pin);
		char tmp_string[100];
		u8g2_FirstPage(&u8g2);
		do
		 {
			//////t1
			if(temperature1timeout == DATA_WAIT_TIMEOUT){
			    sprintf(tmp_string, "T xxx");
			}else{
				tempInt = (int16_t)(temperature/100);
				tempDec = (uint16_t)(temperature%100);
				sprintf(tmp_string, "T %d,%u", tempInt, tempDec);
			}
			u8g2_DrawStr(&u8g2, 0, 20, tmp_string);


			//////h1
			if(humidity1timeout == DATA_WAIT_TIMEOUT){
			    sprintf(tmp_string, "H xxx");
			}else{
				tempUInt = (uint32_t)(humidity/1024);
				tempDec = (uint16_t)(humidity%1024);
				sprintf(tmp_string, "H %lu,%u", tempUInt, tempDec);
				printf(tmp_string);
			}
			u8g2_DrawStr(&u8g2, 0, 45, tmp_string);


			//////t2
			if(temperature2timeout == DATA_WAIT_TIMEOUT){
			    sprintf(tmp_string, "T xxx");
			}else{
				tempInt = (int16_t)(temperature/100);
				tempDec = (uint16_t)(temperature%100);
				sprintf(tmp_string, "T %u,%u", tempInt, tempDec);
			}
			u8g2_DrawStr(&u8g2, 0, 80, tmp_string);


			//////h2
			if(humidity2timeout == DATA_WAIT_TIMEOUT){
			    sprintf(tmp_string, "H xxx");
			}else{
				tempUInt = (uint32_t)(humidity/1024);
				tempDec = (uint16_t)(humidity%1024);
			}
			u8g2_DrawStr(&u8g2, 0, 105, tmp_string);

		 } while (u8g2_NextPage(&u8g2));
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hcan.Init.Prescaler = 9;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS1_DISPLAY_Pin|CS0_BME280_Pin|RELAY0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISPLAY_RESET_Pin|DISPLAY_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_DISPLAY_Pin CS0_BME280_Pin */
  GPIO_InitStruct.Pin = CS1_DISPLAY_Pin|CS0_BME280_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DISPLAY_RESET_Pin DISPLAY_DC_Pin */
  GPIO_InitStruct.Pin = DISPLAY_RESET_Pin|DISPLAY_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY0_Pin */
  GPIO_InitStruct.Pin = RELAY0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY0_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan_)
{
//	HAL_GPIO_TogglePin(GPIOC, LED_Pin);
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_)
{
	HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	//HAL_GPIO_TogglePin(GPIOC, LED_Pin);

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
		Control_peripheral_relays(RxData[0]);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if( htim->Instance == TIM1 ){
	HAL_GPIO_TogglePin(GPIOC, LED_Pin);
	++temperature1timeout;
	++temperature2timeout;
	++humidity1timeout;
	++humidity2timeout;

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

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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "stdlib.h"
#include "lcd_config.h"
#include "States.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP280_DATA_INDEX 1
#define BMP280_ADDRESS_INDEX 2
const int SPI_BUFFER_LEN= 28;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Received[4];
double TempRef=24;
char buffer[100];
uint8_t size;
uint8_t error;
static double TempDif;
static unsigned short heater_state = 0;
static unsigned short fan_state = 0;
static unsigned short ld1ex_state = 0;
static unsigned short ld2ex_state = 0;
static unsigned short ld3ex_state = 0;
double TempRecived;
float TempSVW=0;
double temp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/**
 *
 * @brief HAL_UART_RxCpltCallback it's job is to return information
 * if data sent via UART is incorrect,
 * or to set received data as TempRef if data is correct.
 * Also at the same time it's shows information on second line of LCD.
 * To make it easier for user to notice we put LCD screen to sleep for 1s,
 * it doesn't change anything, because with current parts even if we take
 * each read from sensor in 1ms, any change will be noticeable after 1s.
 *
 * @param huart it's a UART3 handler
 */

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
	TempRecived=atof(&Received);
	if (TempRecived < 18.0)
	{
		error = sprintf(buffer,"Incorrect value\n\r ");
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, error, 200);
		LCD_SetCursor(&hlcd1, 1, 0);
		LCD_printf(&hlcd1, "Incorrect value");
		lcd_delay_us(&hlcd1, 1000000);
		LCD_Clear(&hlcd1);

	}
	else if (TempRecived > 26.0)
	{
		error = sprintf(buffer,"Incorrect value\n\r ");
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, error, 200);
		LCD_SetCursor(&hlcd1, 1, 0);
		LCD_printf(&hlcd1, "Incorrect value");
		lcd_delay_us(&hlcd1, 1000000);
		LCD_Clear(&hlcd1);
	}
	else
	{
		TempRef = TempRecived;
		LCD_SetCursor(&hlcd1, 1, 0);
		LCD_printf(&hlcd1, "Set %4.2f[degC]", TempRef);
		lcd_delay_us(&hlcd1, 1000000);
//		LCD_Clear(&hlcd1);
	}

}

int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
        {
    		 /**Transmitting the temperature via UART*/
    		 size = sprintf(buffer,"Temperature %4.2f [degC]\n\r ", temp);
    		 HAL_UART_Transmit(&huart3, (uint8_t*)buffer, size, 100);
        }

    if (htim->Instance == TIM2)
            {
    		 /** States_Run function callback */
    	     States_Run(heater_state,fan_state,ld1ex_state,ld2ex_state,ld1ex_state);

    	     /** States_Run function callback*/
    	     States_Change(TempRef, temp);


            }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int8_t rslt;
	struct bmp280_dev bmp;
	struct bmp280_config conf;
	struct bmp280_uncomp_data ucomp_data;
	int32_t temp32;
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
  MX_USART3_UART_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  bmp.delay_ms=HAL_Delay;
  bmp.dev_id = 0;
  bmp.read = spi_reg_read;
  bmp.write = spi_reg_write;
  bmp.intf = BMP280_SPI_INTF;

 rslt = bmp280_init(&bmp);


 /* Always read the current settings before writing, especially when
  * all the configuration is not modified
  */
 rslt = bmp280_get_config(&conf, &bmp);


 /* configuring the temperature oversampling, filter coefficient and output data rate */
 /* Overwrite the desired settings */
 conf.filter = BMP280_FILTER_COEFF_2;

 /* Temperature oversampling set at 4x */
 conf.os_temp = BMP280_OS_4X;

 /* Pressure over sampling none (disabling pressure measurement) */
 conf.os_pres = BMP280_OS_NONE;

 /* Setting the output data rate as 1HZ(1000ms) */
 conf.odr = BMP280_ODR_1000_MS;
 rslt = bmp280_set_config(&conf, &bmp);


 /* Always set the power mode after setting the configuration */
 rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);

 HAL_TIM_Base_Start_IT(&htim4);
 HAL_TIM_Base_Start_IT(&htim2);

 LCD_Init(&hlcd1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /** Reading the raw data from sensor */
     rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

     /** Getting the compensated temperature as floating point value */
     rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);

     /** Sleep time between measurements = BMP280_ODR_2000_MS */
     bmp.delay_ms(100);

     /** States_Run function callback */
     //States_Run(heater_state,fan_state,ld1ex_state,ld2ex_state,ld1ex_state);

     /** Receiving Referenced temperature*/
     HAL_UART_Receive_IT(&huart3, Received, 4);

     /** States_Run function callback*/
     //States_Change(TempRef, temp);

     /** Displaying the temperature on LCD */
     LCD_SetCursor(&hlcd1, 0, 0);
     LCD_printf(&hlcd1, "Temp %4.2f[degC]", temp);

     HAL_Delay(1);

     /** Conversion to float in case of Serial Wire Viewer use*/
     TempSVW= (float)temp;

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//Functions bellow are provided by the manufacturer of the sensor

/*!
 * @brief Function for writing the sensor 's registers through SPI bus.
 *
 * @param [in] cs : Chip select to enable the sensor .
 * @param [in] reg_addr : Register address .
 * @param [in] reg_data : Pointer to the data buffer whose data has to be written .
 * @param [in] length : No of bytes to write .
 *
 * @return Status of execution
 * @retval 0 -> Success
 * @retval >0 -> Failure Info
 */
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	/* Implement the SPI write routine according to the target machine . */
	HAL_StatusTypeDef status = HAL_OK ;
	int32_t iError = BMP280_OK ;
	uint8_t txarray [ SPI_BUFFER_LEN * BMP280_ADDRESS_INDEX ];
	uint8_t stringpos ;


	txarray [0] = reg_addr ;
	for ( stringpos = 0; stringpos < length ; stringpos ++) {
	txarray [ stringpos + BMP280_DATA_INDEX ] = reg_data [ stringpos ];
	 }

	HAL_GPIO_WritePin ( BMP280_CS1_GPIO_Port , BMP280_CS1_Pin , GPIO_PIN_RESET );
	status = HAL_SPI_Transmit ( &hspi4 , ( uint8_t *)(& txarray ), length *2, 100);
	while ( hspi4 . State == HAL_SPI_STATE_BUSY ) {};
	HAL_GPIO_WritePin ( BMP280_CS1_GPIO_Port , BMP280_CS1_Pin , GPIO_PIN_SET );

	if ( status != HAL_OK )
	{
	iError = ( -1);
	}
	return ( int8_t ) iError ;
}
/*!
 * @brief Function for reading the sensor 's registers through SPI bus.
 *
 * @param [in] cs : Chip select to enable the sensor .
 * @param [in] reg_addr : Register address .
 * @param [ out] reg_data : Pointer to the data buffer to store the read data .
 * @param [in] length : No of bytes to read .
 *
 * @return Status of execution
 * @retval 0 -> Success
 * @retval >0 -> Failure Info
 */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	/* Implement the SPI read routine according to the target machine . */
	HAL_StatusTypeDef status = HAL_OK ;
	int32_t iError = BMP280_OK ;
	uint8_t txarray [ 28 ] = {0 ,};
	uint8_t rxarray [ 28 ] = {0 ,};
	uint8_t stringpos ;

	txarray [0] = reg_addr ;

	HAL_GPIO_WritePin ( BMP280_CS1_GPIO_Port , BMP280_CS1_Pin , GPIO_PIN_RESET );
	status = HAL_SPI_TransmitReceive ( &hspi4 , ( uint8_t *)(& txarray ),( uint8_t *)(& rxarray ), length +1, 5);
	while ( hspi4 . State == HAL_SPI_STATE_BUSY ) {};
	HAL_GPIO_WritePin ( BMP280_CS1_GPIO_Port , BMP280_CS1_Pin , GPIO_PIN_SET );

	for ( stringpos = 0; stringpos < length ; stringpos ++)
	{
		*( reg_data + stringpos ) = rxarray [ stringpos + BMP280_DATA_INDEX ];
	}

	if ( status != HAL_OK )
	{
	iError = ( -1);
	}
	return ( int8_t ) iError ;
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
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

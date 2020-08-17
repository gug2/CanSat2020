
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdio.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include "bmp280.h"
#include "bmi160.h"
// LORA
//#include "SX1278.h"
#include "lora_sx1276.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char buf[128];
FATFS fs;
FIL file;
uint32_t byteswritten;
BMP280_HandleTypedef bmp280;
float temp, pres, humi, alti, s_pres;
struct bmi160_dev bmi160;
struct bmi160_sensor_data bmi160Accel, bmi160Gyro;
int usePhotoresist = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
char gpsBuffer[650];
uint8_t h, m, s, ms;
// GPS
char rxCharBuf[700];
char *rxChars;
uint8_t rxBuffer[512];
uint16_t rxIndex;
uint8_t rxTmp;

// LORA
//uint16_t message,message_length;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if(huart == &huart4) {
	if(rxIndex < 512) {
	  rxBuffer[rxIndex] = rxTmp;
	  rxIndex++;
	}
    HAL_UART_Receive_IT(&huart4, &rxTmp, 1);
  }
}

int8_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, 1, (uint8_t*)data, len, 1000);
	return status;
}

int8_t i2cRead(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, 1, (uint8_t*)data, len, 1000);
	return status;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SDIO_SD_Init();
  MX_UART4_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  //rxIndex = 0;
  //HAL_UART_Receive_IT(&huart4, &rxTmp, 1);

  /*bmi160.id = BMI160_I2C_ADDR;
  bmi160.interface = BMI160_I2C_INTF;
  bmi160.read = i2cRead;
  bmi160.write = i2cWrite;
  bmi160.delay_ms = HAL_Delay;
  int8_t result = BMI160_E_DEV_NOT_FOUND;
  result = bmi160_init(&bmi160);

  bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
  bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
  bmi160.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
  bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
  bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
  bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
  bmi160.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
  bmi160.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
  result = bmi160_set_sens_conf(&bmi160);

  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c1;
  bmp280_init(&bmp280, &bmp280.params);

  HAL_Delay(100);*/

  //bmp280_read_float(&bmp280, NULL, &s_pres, NULL);
  //s_pres /= 133.322F; // to mm Hg

  // LORA
  /*lora_sx1276 lora;
  lora.spi = &hspi1;
  lora.nss_port = GPIOA;
  lora.nss_pin = GPIO_PIN_4;
  lora.frequency = 434000000;
  lora.pa_mode = LORA_PA_OUTPUT_PA_BOOST;
  lora.tx_base_addr = LORA_DEFAULT_TX_ADDR;
  lora.rx_base_addr = LORA_DEFAULT_RX_ADDR;
  lora.spi_timeout = LORA_DEFAULT_SPI_TIMEOUT;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(250);

  uint8_t mode = read_register(&lora, 0x01);
  HAL_Delay(250);
  uint8_t init = 0xAA;
  init = lora_init(&lora, &hspi1, GPIOA, GPIO_PIN_4, 434000000);*/
  // end

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    //if(rxIndex == 512) {
	  	/*result = bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL, &bmi160Accel, &bmi160Gyro, &bmi160);
    	bmp280_read_float(&bmp280, &temp, &pres, &humi);

    	pres /= 133.322F; // to mm Hg
    	alti = 44330.0F * (1.0F - powf(pres / s_pres, 1.0F / 5.255F));
    	pres *= 133.322F; // to Pa

    	rxChars = (char*)rxBuffer;

    	float
		ax = (bmi160Accel.x * 2.0F) / 32768.0F,
		ay = (bmi160Accel.y * 2.0F) / 32768.0F,
		az = (bmi160Accel.z * 2.0F) / 32768.0F,
		gx = (bmi160Gyro.x * 2000.0F) / 32768.0F,
		gy = (bmi160Gyro.y * 2000.0F) / 32768.0F,
		gz = (bmi160Gyro.z * 2000.0F) / 32768.0F;

    	h = 0; m = 0; s = 0; ms = 0;*/
    	//memset(gpsBuffer, 0, sizeof(gpsBuffer));
    	//sprintf(gpsBuffer, "%s", rxChars); // если вместо %2d написать %2hhd в часах, то показывают только их
    	//sscanf(strstr(rxChars, "$GNGGA,"), "$GNGGA,%2d%2d%2d.%2d\r\n", &h, &m, &s, &ms);

    	/*sprintf(
    	  rxCharBuf,
		  "GPSDATA: %s; RMC: %d:%d:%d.%d, TIME: %d, TEMP: %.2fC, PRES: %.2fPa, HUMI: %.2f, ALTI: %.2fM, A: %.2f : %.2f : %.2f, G: %.2f : %.2f : %.2f;\r\n",
		  rxChars, h, m, s, ms, HAL_GetTick(), temp, pres, humi, alti, ax, ay, az, gx, gy, gz
    	);

    	f_mount(&fs, "", 0);
        f_open(&file, "file.txt", FA_OPEN_ALWAYS | FA_WRITE);
    	f_lseek(&file, f_size(&file));
    	f_write(&file, rxCharBuf, sizeof(rxCharBuf), &byteswritten);
    	f_close(&file);
    	f_mount(NULL, "", 0);*/

    	//rxChars = 0;
    	//memset(rxBuffer, 0, sizeof(rxBuffer));
    	//rxIndex = 0;
    //}
    //HAL_UART_Receive_IT(&huart4, &rxTmp, 1);
    //HAL_Delay(100);

    // LORA
	/*char dataBuf[64];
	uint8_t dataSize = sprintf(dataBuf, "HELLO!\r\n");
	uint8_t transmit = 0xAA;
	transmit = lora_send_packet_blocking(&lora, (uint8_t*)dataBuf, dataSize, 1000);

	uint16_t size = sprintf(rxCharBuf, "REG OP MODE : 0x%x\r\n LORA INIT: 0x%x\r\n LORA_TRANSMIT: 0x%x\r\n", mode, init, transmit);
	f_mount(&fs, "", 0);
	f_open(&file, "file.txt", FA_OPEN_ALWAYS | FA_WRITE);
	f_lseek(&file, f_size(&file));
	f_write(&file, rxCharBuf, size, &byteswritten);
	f_close(&file);
	f_mount(NULL, "", 0);
	HAL_Delay(500);*/

	// ADC Photoresistor
	HAL_ADCEx_InjectedStart(&hadc2);
	HAL_ADCEx_InjectedPollForConversion(&hadc2, 1000);
	uint32_t photoresistor = HAL_ADCEx_InjectedGetValue(&hadc2, 1);
	HAL_ADCEx_InjectedStop(&hadc2);

	// use photoresist for burning wire
	if(photoresistor >= 2000 && usePhotoresist == 0) {
		usePhotoresist = 1;
		// enable PC6
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		//
	}
	//

	uint16_t size = sprintf(rxCharBuf, "PHOTORESIST: %d\r\n", photoresistor);
	f_mount(&fs, "", 0);
	f_open(&file, "file.txt", FA_OPEN_ALWAYS | FA_WRITE);
	f_lseek(&file, f_size(&file));
	f_write(&file, rxCharBuf, size, &byteswritten);
	f_close(&file);
	f_mount(NULL, "", 0);

	HAL_Delay(100);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

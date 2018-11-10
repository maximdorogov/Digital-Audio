
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include <math.h>
#include "CS43L22.h"
#include "data_table.h"
#include <string.h>

#define AUDIO_BUFFER_SIZE 32 //si uso 64 se escucha un plop cada tanto.

#define USE_ADC  0
#define USE_LOOKUP_TABLE_INDEX 1
#define USE_LOOKUP_TABLE 2
#define USE_LOOKUP_TABLE_COMPLEMENT_2 3
#define USE_DATA_SAMPLE 4



#define RUN_OPT USE_ADC//USE_LOOKUP_TABLE_COMPLEMENT_2  //aca se define de donde levanto el audio para transferir al Cirrus




typedef enum{left = 0,right}channel_t;

typedef enum{audio_read_state = 0,audio_write_state ,audio_process_state,audio_send_state}audio_state_t;

typedef enum{data_ready = 0, idle}flag_t;

typedef enum{buffer_A = 0,buffer_B}buffer_t;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint16_t audioBufferA[2*AUDIO_BUFFER_SIZE];
uint16_t audioBufferB[2*AUDIO_BUFFER_SIZE];

flag_t flag = idle;
buffer_t  buffer_to_send;
buffer_t buffer_to_fill = buffer_A;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void CS43L22_init(void);

int16_t audio_dsp(uint16_t *);


uint16_t audio_read(void);
void audio_send_buffer(uint16_t *,uint16_t);
void audio_buffer_init(void);                    //inicializa el audio buffer con ceros.

//dsp functions

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
DSAFDSA
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
  MX_I2S3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //audio_buffer_init(); //lleno ambos buffers con ceros
  CS43L22_init();  //configuro el DAC

  HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
/*buffer_usaddo = 0

		proceso
			buffer_usado < dato adc
			inc
			se lleno?
				swap buffer

		proceso
			envias buffer libre

	  */
	  switch(flag){

		  case idle:
			  break;

		  case data_ready:

			  flag = idle;

			  if(buffer_to_send == buffer_A){

				  HAL_I2S_Transmit(&hi2s3,audioBufferA,2*AUDIO_BUFFER_SIZE,1);
			  }

			  if(buffer_to_send == buffer_B){

				  HAL_I2S_Transmit(&hi2s3,audioBufferB,2*AUDIO_BUFFER_SIZE,1);

			  }
			  break;

		  default:
			  break;
	  }

	//  HAL_I2S_Transmit(&hi2s3,audioBufferA,2*AUDIO_BUFFER_SIZE,10);

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 258;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 175;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Sampling_Check_GPIO_Port, Sampling_Check_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Sampling_Check_Pin */
  GPIO_InitStruct.Pin = Sampling_Check_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Sampling_Check_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int16_t audio_dsp(uint16_t *sample){

	 return  (*sample) - 2047;

}

void audio_buffer_init(){

	size_t i;

	for(i = 0;i<2*AUDIO_BUFFER_SIZE;i++){

		audioBufferA[i] = 0;
		audioBufferB[i] = 0;

	}

}


void audio_send_buffer(uint16_t *buffer,uint16_t buffer_size){

	HAL_I2S_Transmit(&hi2s3,buffer,2*buffer_size,10);

}

uint16_t audio_read(void){

#if (RUN_OPT == USE_ADC)

	return HAL_ADC_GetValue(&hadc1);

#endif

#if (RUN_OPT == USE_LOOKUP_TABLE_INDEX)

	static uint8_t index = 0;

	if(index >= SINE_SAMPLES){

		index = 0;
	}

	return sine_lookup[++index];

#endif

#if (RUN_OPT == USE_LOOKUP_TABLE_COMPLEMENT_2)

	static uint8_t index = 0;

	if(index >= SINE_SAMPLES){

		index = 0;
	}

	return SINE_COMP[++index];

#endif

#if (RUN_OPT == USE_DATA_SAMPLE)

	return data_sample;

#endif
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static uint16_t i = 0;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

  	  if(buffer_to_fill == buffer_A){

		  audioBufferA[i] = audio_read();
		  audioBufferA[i+1] =  audioBufferA[i];
		  i = i+2;

		  if(i >= 2*AUDIO_BUFFER_SIZE){

			i=0;

			buffer_to_send = buffer_A;
			buffer_to_fill = buffer_B;
			flag = data_ready;
		  }
  	  }
		  //////////////////////////////////////////////

  	  if(buffer_to_fill == buffer_B){

		  audioBufferB[i] = audio_read();
		  audioBufferB[i+1] =  audioBufferB[i];
		  i = i+2;

		  if(i >= 2*AUDIO_BUFFER_SIZE){

			i=0;

			buffer_to_send = buffer_B;
			buffer_to_fill = buffer_A;
			flag = data_ready;
		  }
  	  }
}

void CS43L22_write(uint8_t reg, uint8_t Cmd){

	uint8_t iData[2]; // Buffer to read, change and write back register values of the DAC

	iData[0] =  reg;
	iData[1] = Cmd;

	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDRESS, iData, 2, 100);


}

void CS43L22_init(){

	uint8_t iData[2]; // Buffer to read, change and write back register values of the DAC
	// - Set RESET high
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

	// - Load desired register settings

	//   - Power Ctl 1
	//     Write 0x01 to register "Power Ctl 1" to ensure the CS43L22 is in power-down mode.
	//     0000 0001b = Power down (default)
	//     1001 1110b = Powered up
	CS43L22_write( CS43L22_REG_POWER_CTL1, 0x01);

	// - Power Ctl 2
	// PDN_HPB[0:1]  = 10 (HP-B always on)
	// PDN_HPA[0:1]  = 10 (HP-A always on)
	// PDN_SPKB[0:1] = 11 (Speaker B always off)
	// PDN_SPKA[0:1] = 11 (Speaker A always off)
	CS43L22_write( CS43L22_REG_POWER_CTL2, 0xAF);

	//   - Clocking Ctl
	CS43L22_write( CS43L22_REG_CLOCKING_CTL,(1 << 7)); //Tal vez tenga que poner 0x81 para dividir clock

	//   - Interface Ctl 1
	iData[0] = CS43L22_REG_INTERFACE_CTL1;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDRESS, iData, 1, 100); // Transmit register address to the device ...
	HAL_I2C_Master_Receive(&hi2c1, CS43L22_ADDRESS, &iData[1], 1, 100);  // ... and read 1 byte (the register content).
	iData[1] &= (1 << 5); // Clear all bits except bit 5 which is reserved
	iData[1] &= ~(1 << 7);  // Slave
	iData[1] &= ~(1 << 6);  // Clock polarity: Not inverted
	iData[1] &= ~(1 << 4);  // No DSP mode
	iData[1] |= (1 << 2);  // I2S up to 24 bit (default)
	iData[1] |=  (3 << 0);  // 16-bit audio word length for I2S interface
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDRESS, iData, 2, 100);

	// - Misc Ctl
	iData[0] = CS43L22_REG_MISC_CTL ;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDRESS, iData, 1, 100); // Transmit register address to the device ...
	HAL_I2C_Master_Receive(&hi2c1, CS43L22_ADDRESS, &iData[1], 1, 100);  // ... and read 1 byte (the register content).
	iData[1] &= ~(1 << 7);   // Disable passthrough for AIN-A
	iData[1] &= ~(1 << 6);   // Disable passthrough for AIN-B
	iData[1] |=  (1 << 5);   // Mute passthrough on AIN-A
	iData[1] |=  (1 << 4);   // Mute passthrough on AIN-B
	iData[1] &= ~(1 << 3);   // Changed settings take affect immediately
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDRESS, iData, 2, 100);

	//   - Unmute headphone and speaker
	CS43L22_write( CS43L22_REG_PLAYBACK_CTL2, 0x00);

	//   - Volume PCM-A and Volume PCM-B
	CS43L22_write( CS43L22_REG_PCMA_VOL, 0x00);
	CS43L22_write( CS43L22_REG_PCMB_VOL, 0x00);

	// - Perform initialization as described in the datasheet of the CS43L22, Chapter "4.11 Required Initialization Settings"

	// Write 0x99 to register 0x00.
	CS43L22_write( 0x00, 0x99);

	// Write 0x80 to register 0x47.
	CS43L22_write( 0x47, 0x80);

	// Write '1'b to bit 7 in register 0x32.
	iData[0] = 0x32;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDRESS, iData, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, CS43L22_ADDRESS, &iData[1], 1, 100);
	iData[1] |= 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDRESS, iData, 2, 100);

	// Write '0'b to bit 7 in register 0x32.
	iData[0] = 0x32;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDRESS, iData, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, CS43L22_ADDRESS, &iData[1], 1, 100);
	iData[1] &= ~(0x80);
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDRESS, iData, 2, 100);

	// Write 0x00 to register 0x00.
	CS43L22_write( 0x00, 0x00);

	// - Set the "Power Ctl 1" register (0x02) to 0x9E
	CS43L22_write( CS43L22_REG_POWER_CTL1 , 0x9E);
}
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

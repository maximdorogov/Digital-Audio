
#include "main.h"
#include "stm32f4xx_hal.h"


#include <math.h>
#include "CS43L22.h"
#include "data_table.h"
#include <string.h>
#include "stm32_DiscoveryBoard.h"

#define AUDIO_BUFFER_SIZE 180
//con 2*90 el plop se escucha cada tanto 90 samples por cada canal intercaladas

#define USE_ADC  0
#define USE_LOOKUP_TABLE_COMPLEMENT_2 3

#define ADC_OFFSET 0//2048  //Si uso LOOKUP TABLE offset = 0

#define DSP
#define RUN_OPT USE_ADC//USE_LOOKUP_TABLE_COMPLEMENT_2 //aca se define de donde levanto el audio para transferir al Cirrus

typedef enum{data_ready_to_send = 0,data_ready_to_dsp, idle}flag_t;

typedef enum{FALSE,TRUE}bool_t;

typedef enum{buffer_A = 0,buffer_B,no_fill}buffer_t;



uint16_t audioBufferA[AUDIO_BUFFER_SIZE];
uint16_t audioBufferB[AUDIO_BUFFER_SIZE];

uint16_t *audioToSend = NULL;
uint16_t *audioToUpdate = NULL;

flag_t flag = idle;
uint8_t adc_done = 0;
buffer_t  buffer_to_send = buffer_B;
buffer_t buffer_to_fill = buffer_A;//no_fill;
uint16_t sample;

////////////////////////////echo dsp effect////////////////////////////////////////

uint16_t dsp_echo_short(uint16_t data,size_t depth){

	static uint16_t sDelayBuffer[25000];
	static uint16_t counter = 0;

	sDelayBuffer[counter] = (data + sDelayBuffer[counter])>>1;

	counter ++;

	if(counter >= depth){ counter = 0; }

	return sDelayBuffer[counter] + data;

}


uint16_t dsp_echo(uint16_t data,size_t depth){

	static uint16_t sDelayBuffer[30000];
	static uint16_t counter = 0;

	sDelayBuffer[counter] = (data + sDelayBuffer[counter])>>1;

	counter ++;

	if(counter >= depth){ counter = 0; }

	return sDelayBuffer[counter] + data;
}

void dsp_overdrive(uint16_t *data,uint16_t threshold){

	if(*data >= threshold){

		*data =  threshold;

	}

}

////////////////////////////////////////////////////////////////////

volatile bool_t transferComplete = TRUE;

/* USER CODE END PV */



/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

uint16_t audio_read(void);

void audio_effect(uint16_t*);
uint16_t* select_buffer_to_transmit(buffer_t);

void audio_buffer_init(void);   //inicializa el audio buffer con ceros.
void fill_buffers();
void load_buffer(uint16_t *);

int main(void)
{

	BoardInit(); //configuro todos los perifericos del micro

	CS43L22_init();   //configuro el DAC CS43L22

	HAL_TIM_Base_Start(&htim2); //activo el timer
	HAL_ADC_Start_IT(&hadc1); // y el ADC

	audio_buffer_init();         //inicializo los buffers con ceros
	audioToSend = audioBufferB;   //asigno punteros a c/u de los buffers
	audioToUpdate = audioBufferA;

	while (1)
	{
		if(transferComplete && (flag == data_ready_to_send)){

			flag = idle;

			audioToSend = select_buffer_to_transmit(buffer_to_send); //me devuelve el puntero al buffer listo para enviar

			CS43L22_AudioSend(audioToSend,AUDIO_BUFFER_SIZE); //Envio el buffer por I2S al codec

			transferComplete = FALSE;

		}

		if(adc_done){

			adc_done = 0;

			fill_buffers();

		}

	}

}



void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hi2s);

	//HAL_GPIO_TogglePin(Sampling_Check_GPIO_Port, Sampling_Check_Pin);

	transferComplete = TRUE;

}


void audio_effect(uint16_t *smp){

	*smp = (uint16_t)(*smp - ADC_OFFSET)<<0;
};

void load_buffer(uint16_t *buff){

	static size_t i = 0;

	sample = audio_read();

#ifdef DSP

	sample = dsp_echo(sample,25000);

	sample = dsp_echo_short(sample,1500);

#endif
	buff[i] = sample;

	buff[i+1] =  buff[i];

	i = i+2;

	if( (i >= AUDIO_BUFFER_SIZE)){

		buffer_t aux = buffer_to_send;
		buffer_to_send = buffer_to_fill;
		buffer_to_fill = aux;

		flag = data_ready_to_send;
		i=0;

	}

}

void fill_buffers(){

	if((buffer_to_fill == buffer_A)){

		audioToUpdate = audioBufferA;
		load_buffer(audioToUpdate);

	}
	if((buffer_to_fill == buffer_B)){

		audioToUpdate = audioBufferB;
		load_buffer(audioToUpdate);
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)

{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hadc);
	adc_done = 1;

}

uint16_t* select_buffer_to_transmit(buffer_t bf){

	uint16_t *ptr = NULL;

	if(buffer_to_send == buffer_A){

		ptr = audioBufferA;

	}else if(buffer_to_send == buffer_B){

		ptr = audioBufferB;

	}

	return ptr;

}

/* USER CODE BEGIN 4 */

void audio_buffer_init(){

	size_t i;

	for(i = 0;i<AUDIO_BUFFER_SIZE;i++){

		audioBufferA[i] = 0;
		audioBufferB[i] = 0;

	}

}


uint16_t audio_read(void){

#if (RUN_OPT == USE_ADC)

	return HAL_ADC_GetValue(&hadc1);

#endif

#if (RUN_OPT == USE_LOOKUP_TABLE_COMPLEMENT_2)

	static size_t index = 0;

	if(index >= SINE_SAMPLES){

		index = 0;
	}

	return SINE_COMP[index++];

#endif

}

void CS43L22_EXTERNAL_DAC_I2S_transmit(uint16_t *buffer,uint16_t buffer_size){

	HAL_I2S_Transmit_IT(&hi2s3,buffer,buffer_size);

}


void CS43L22_EXTERNAL_DAC_I2C_write(uint8_t *iData, uint8_t len)
{ //maneja el periferico i2c para comunicaicon con el dac

	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_ADDRESS, iData, len, 100);

}

void CS43L22_EXTERNAL_DAC_I2C_recieve(uint8_t *iData){

	HAL_I2C_Master_Receive(&hi2c1, CS43L22_ADDRESS, iData, 1, 100);

}

void CS43L22_EXTERNAL_DAC_enable()
{
	//esta funcion se encarga de poner en alto o bajo el pin de reset del dac
	//recibe un 1 para poner el pin en alto y un 0 para ponerlo en bajo

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

}


/* USER CODE END 4 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

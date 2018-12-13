
#include "main.h"
#include "stm32f4xx_hal.h"


#include <math.h>
#include <sin256.h> //256 muestras complemento a 2 de una senoidal
#include "CS43L22.h"
#include <string.h>
#include "stm32_DiscoveryBoard.h"
#include "AudioEffects.h"

#define AUDIO_BUFFER_SIZE 180
//con 2*90 el plop se escucha cada tanto 90 samples por cada canal intercaladas

//opciones de compilacion

#define USE_ADC  0
#define USE_LOOKUP_TABLE_COMPLEMENT_2 3

#define ADC_OFFSET 0//2048  //Si uso LOOKUP TABLE offset = 0

// defino si quiero usar efectos en el camino de la senial

#define DSP

// defino la cantidad maxima de muestras que voy a reservar para el buffer del delay

#ifdef DSP

	#define MAX_DELAY 30000

#endif

//aca se define de donde levanto el audio para transferir al DAC externo

#define RUN_OPT USE_ADC

/* %%%%%%%%%%%%%%%%% Declaro buffers para lectura y transmision de audio %%%%%%%%%%%%%%%%%%%*/

uint16_t audioBufferA[AUDIO_BUFFER_SIZE];
uint16_t audioBufferB[AUDIO_BUFFER_SIZE];

uint16_t delayBuffer[MAX_DELAY];

/* %%%%%%%%%%%%%%%%% Declaro punteros para lectura y transmision de audio %%%%%%%%%%%%%%%%%%%*/

uint16_t *audioToSend = NULL;
uint16_t *audioToUpdate = NULL;

/* %%%%%%%%%%%%%%%%% Declaro variables globales %%%%%%%%%%%%%%%%%%%*/


flag_t flag = idle;

uint8_t adc_done = 0;

buffer_t  buffer_to_send = buffer_B;
buffer_t buffer_to_fill = buffer_A;

uint16_t sample;
volatile bool_t transferComplete = TRUE;

DSP_Delay_t long_delay;


/*%%%%%%%%%%%%%%%%%%%%%% Prototipos de funciones %%%%%%%%%%%%%%%%%%%%%%%%*/

uint16_t audio_read(void);
uint16_t* select_buffer_to_transmit(buffer_t);

void audio_buffer_init(void);   	//inicializo el audio buffer con ceros.
void fill_buffers();
void load_buffer(uint16_t *);



int main(void)
{
	uint16_t dTime  = 25000; 	    //defino el tiempo de delay
	uint8_t dgain = 1;       	    //las ganancias representan una potencia de 2. con lo cual 1 equivale a 2^1, 2 es 2^2, etc..

	audioToSend = audioBufferB;     //asigno punteros a c/u de los buffers
	audioToUpdate = audioBufferA;

	BoardInit(); 					//configuro todos los perifericos del micro

	CS43L22_init();   				//configuro el DAC CS43L22

	audio_buffer_init();         	//inicializo los buffers con ceros

	DSP_DelayInit(&long_delay, dTime, delayBuffer, MAX_DELAY, dgain);


	HAL_TIM_Base_Start(&htim2); 	//activo el timer
	HAL_ADC_Start_IT(&hadc1); 		// y el ADC



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


void load_buffer(uint16_t *buff){

	static size_t i = 0;

	sample = audio_read();

#ifdef DSP

	sample =  DSP_DelaySample(&long_delay,sample);

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

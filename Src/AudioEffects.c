
#include "AudioEffects.h"


void DSP_DelayInit(DSP_Delay_t *DelayDSP, size_t dTime, uint16_t *buffer, size_t buffer_len, uint8_t gain){

	DelayDSP->delay_time = dTime;
	DelayDSP->gain = gain;
	DelayDSP->ptr = buffer;
	DelayDSP->bufferLen = buffer_len;
	DelayDSP->s_index = 0;
}

uint16_t DSP_DelaySample(DSP_Delay_t *DelayDSP, uint16_t sample){

	DelayDSP->ptr[ DelayDSP->s_index ] = ( sample + DelayDSP->ptr[ DelayDSP->s_index ] ) >> DelayDSP->gain;

	DelayDSP->s_index++;

	if( DelayDSP->s_index >= DelayDSP->delay_time){

		DelayDSP->s_index = 0;

	}

	return sample + DelayDSP->ptr[ DelayDSP->s_index ];

}


/*float16_t DSP_DelayGetDelayTime(DSP_Delay_t *DelayDSP,uint32_t fs){

	return (DelayDSP->delay_time);

}*/

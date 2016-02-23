#include "audio_output.h"


#define OUT_BUFFER_SAMPLES 1024
#define MIC_BUFFER_SAMPLES (OUT_BUFFER_SAMPLES*3/2)
//* 3/2)
/// 2)
#define SAMPLE_RATE 32000

int volume = 75;

int numbuff = 0;
volatile uint8_t rdy_to_play;

static volatile uint16_t mic_head;
static volatile uint16_t  mic_tail;    // head and tail indices to mic buffer


//#define  __TONE_TEST
//#define  __TEST
#ifdef __TEST
#ifdef __TONE_TEST
#include "sin440hz.h"
int16_t * out_buff0 = sine440hz;
int16_t * out_buff1 = sine440hz;
void data_rdy_callback(int16_t *buffer, int num_samples)
{
}

void fill_buffer(int16_t* buff,int num_samples)
{
    
}


#else
float32_t x=0;
int16_t out_buff0[OUT_BUFFER_SAMPLES];
int16_t out_buff1[OUT_BUFFER_SAMPLES];
void data_rdy_callback(int16_t *buffer, int num_samples)
{
}
void fill_buffer(int16_t* buff, int num_samples)
{
   
    uint16_t i;
    for(i=0;i<OUT_BUFFER_SAMPLES/2;i+=2){	
	buff[i+1]=buff[i]=(int16_t)(32767*arm_sin_f32(440*x));
	x+=(2*PI/(SAMPLE_RATE));
    }
}
#endif
#else
int16_t out_buff0[OUT_BUFFER_SAMPLES];
int16_t out_buff1[OUT_BUFFER_SAMPLES];
int16_t micbuff [MIC_BUFFER_SAMPLES];

static void fill_buffer (int16_t *buffer, int num_samples)
{
	int count = num_samples / 2;
	uint8_t go=0;
	
//	dsp(micbuff + mic_tail, count);
	
	// make mono to stereo here
//	while(!go){
	    
	if(mic_tail > mic_head){
	    // blink led on buffer under runs
	    if( MIC_BUFFER_SAMPLES+mic_head-mic_tail < count) {
		GPIO_ToggleBits(GPIOD, GPIO_Pin_13);	
//	    } else {
//		go=1;
		
	    }
	    
	    
	}
	else {
	    if( mic_head-mic_tail < count) {
		GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
		/*	usart_send_uint(mic_head);
		usart_send_byte(' ');
		usart_send_uint(mic_tail);usart_CRLF();*/
		
		
//	    } else {
//		go=1;
		
	    }
	    
	    
	}
//	}
	

	
  while (count--) 
	{
	    *buffer++ = micbuff [mic_tail];
	    *buffer++ = micbuff [mic_tail];
	    mic_tail++;
	    mic_tail %=MIC_BUFFER_SAMPLES;
	    
		//=(mic_tail + 1 >= MIC_BUFFER_SAMPLES) ? 0 : mic_tail + 1;
	}
}



void data_rdy_callback(int16_t *buffer, int num_samples)
{
    // static int clip_timer;
    int clip = 0, i;

    for (i = 0; i < num_samples; ++i) {
        int16_t sample = *buffer++;
        if (sample >= 32700)
            sample=32700;
	if( sample <= -32700)
	    sample=-32700;
	
        micbuff [(mic_head + i)%MIC_BUFFER_SAMPLES] = sample;
    }
    
    mic_head = (mic_head + num_samples)%MIC_BUFFER_SAMPLES;
//(mic_head + num_samples >= MIC_BUFFER_SAMPLES) ?  (mic_head + num_samples - MIC_BUFFER_SAMPLES) : mic_head + num_samples;
}
#endif

void audio_output_init()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);   
    __disable_irq();
    EVAL_AUDIO_SetAudioInterface(AUDIO_INTERFACE_I2S);  
    /* Initialize the Audio codec and all related peripherals (I2S, I2C, IOExpander, IOs...) */  
    EVAL_AUDIO_Init(OUTPUT_DEVICE_AUTO, volume, I2S_AudioFreq_8k);//I2S_AudioFreq_4k );
    NVIC_SetPriority(AUDIO_I2S_DMA_IRQ,2); // write is more prio than the dma
    EVAL_AUDIO_PauseResume(AUDIO_PAUSE);
    __enable_irq();
    // fill_buffer(out_buff0,OUT_BUFFER_SAMPLES);
    rdy_to_play=0;
       
#ifndef __TEST
    int i=0;

    
    
    while(i<OUT_BUFFER_SAMPLES){
	
	out_buff1[i]=out_buff0[i]=0;
	i++;
	
    }
    
#endif
    numbuff=1;  
    //   while(mic_head<OUT_BUFFER_SAMPLES);
    rdy_to_play=1;
    
    Audio_MAL_Play((uint32_t)out_buff0, OUT_BUFFER_SAMPLES*2 );
    EVAL_AUDIO_PauseResume(AUDIO_RESUME);
    
     // EVAL_AUDIO_Play(out_buff0, OUT_BUFFER_SAMPLES*2 );
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size)
{
    if(!rdy_to_play){
	Audio_MAL_Play((uint32_t)out_buff0, OUT_BUFFER_SAMPLES*2 );
	return;
	
    }
    
    if(numbuff){
	Audio_MAL_Play((uint32_t)out_buff1, OUT_BUFFER_SAMPLES*2 );

//	EVAL_AUDIO_Play(out_buff1, OUT_BUFFER_SAMPLES *2);
	numbuff=0;
	fill_buffer(out_buff0,OUT_BUFFER_SAMPLES);
    } else {
	 Audio_MAL_Play((uint32_t)out_buff0, OUT_BUFFER_SAMPLES *2);
//	EVAL_AUDIO_Play(out_buff1, OUT_BUFFER_SAMPLES *2);
	numbuff=1;
	fill_buffer(out_buff1,OUT_BUFFER_SAMPLES);
    }
    
    
}



void EVAL_AUDIO_HalfTransfer_CallBack(uint32_t pBuffer, uint32_t Size)
{
    
}
uint32_t Codec_TIMEOUT_UserCallback(void)
{
    return 0;
    
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void)
{
  return 0;
}

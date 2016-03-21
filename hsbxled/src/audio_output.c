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
uint16_t fft_buff0[OUT_BUFFER_SAMPLES];
uint16_t fft_buff1[OUT_BUFFER_SAMPLES];
uint16_t max_buff[OUT_BUFFER_SAMPLES];

int16_t micbuff [MIC_BUFFER_SAMPLES];
int16_t max_buff_decay_M=5;
int16_t max_buff_decay_D=8;




float32_t sliding_w0[FREQUENCY_BUCKETS];
float32_t sliding_w1[FREQUENCY_BUCKETS];
float32_t sliding_w2[FREQUENCY_BUCKETS];
float32_t sliding_w3[FREQUENCY_BUCKETS];
float32_t sliding_w4[FREQUENCY_BUCKETS];
float32_t sliding_w5[FREQUENCY_BUCKETS];
float32_t sliding_w6[FREQUENCY_BUCKETS];
float32_t sliding_w7[FREQUENCY_BUCKETS];
float32_t sliding_w8[FREQUENCY_BUCKETS];
float32_t sliding_w9[FREQUENCY_BUCKETS];

float32_t * sliding_window[SLIDING_HISTORY]={
    sliding_w0,
    sliding_w1,
    sliding_w2,
    sliding_w3,
    sliding_w4,
    sliding_w5,
    sliding_w6,
    sliding_w7,
    sliding_w8,
    sliding_w9
};

uint8_t pulse_buff[FREQUENCY_BUCKETS];
extern volatile uint8_t pulse_detected;


static void fft_buffer(int16_t *in_buffer,uint16_t *out_buffer,int num_samples)
{
    float32_t mapped_in_buffer[OUT_BUFFER_SAMPLES];
    float32_t mapped_out_buffer[OUT_BUFFER_SAMPLES];
    float32_t sliding_mean[FREQUENCY_BUCKETS];
    float32_t sliding_variance[FREQUENCY_BUCKETS];
    
    arm_rfft_fast_instance_f32 fft;
    float32_t * slide_save;
    uint16_t frequency_bucket_chunck_size = OUT_BUFFER_SAMPLES/FREQUENCY_BUCKETS;

    int oldbucket,bucket;
    float32_t bucket_nrg;
    
    int i,j;
    slide_save = sliding_window[0];
    for(i=0;i<SLIDING_HISTORY-1;i++){
	sliding_window[i]=sliding_window[i+1];
    }
    sliding_window[i]=slide_save;
    
   
    
    for(i=0;i<num_samples;i++){
	mapped_in_buffer[i] = (float32_t)in_buffer[i]/(float32_t)0xffff;
    }
    arm_rfft_fast_init_f32(&fft,num_samples);
    arm_rfft_fast_f32(&fft,mapped_in_buffer,mapped_out_buffer,0);
    // flags 0 -> FFT, 1 IFFT
    
    memset(pulse_buff,0,OUT_BUFFER_SAMPLES);
    memset(sliding_mean,0,FREQUENCY_BUCKETS*sizeof(float32_t));
    memset(sliding_variance,0,FREQUENCY_BUCKETS*sizeof(float32_t));

    
    for(j=0;j<FREQUENCY_BUCKETS;j++){
	for(i=0;i<SLIDING_HISTORY;i++){
	    sliding_mean[j]+=sliding_window[i][j];
	}
	sliding_mean[j]/=SLIDING_HISTORY;
	for(i=0;i<SLIDING_HISTORY;i++){
	    bucket_nrg =  sliding_mean[j] - sliding_window[i][j];
	    bucket_nrg *= bucket_nrg;
	    sliding_variance[j]+=bucket_nrg;
	}
	sliding_variance[j]/=SLIDING_HISTORY;
    }
    

    oldbucket = 0;
    bucket_nrg = 0;
    
    for(i=0;i<num_samples;i++){
	if(mapped_out_buffer[i])
	    mapped_out_buffer[i] *= mapped_out_buffer[i]; // energy
	bucket = (int)(i/frequency_bucket_chunck_size);
	if(bucket != oldbucket){
	    if(!pulse_detected){
		if(bucket_nrg > sliding_mean[oldbucket]*(1.0+sliding_variance[oldbucket])){	    
		    pulse_buff[oldbucket]=1;
		    pulse_detected = 1;
		} else {	
		    pulse_buff[oldbucket]=0;		
		}
	    }
	    
	    slide_save[oldbucket]=bucket_nrg;
	    oldbucket=bucket;
	    bucket_nrg = mapped_out_buffer[i];
	} else {
	    bucket_nrg+=mapped_out_buffer[i];
	}
    }
}


static void fill_buffer (int16_t *buffer, int num_samples)
{
    int count = num_samples / 2;
    if(mic_tail > mic_head){
	// blink led on buffer under runs
	if( MIC_BUFFER_SAMPLES+mic_head-mic_tail < count) {
	    GPIO_ToggleBits(GPIOD, GPIO_Pin_13);			
	}
    }
    else {
	if( mic_head-mic_tail < count) {
	    GPIO_ToggleBits(GPIOD, GPIO_Pin_14);		
	}
    }
    while (count--) 
    {
	*buffer++ = micbuff [mic_tail];
	*buffer++ = micbuff [mic_tail];
	mic_tail++;
	mic_tail %=MIC_BUFFER_SAMPLES;
    }
}

void data_rdy_callback(int16_t *buffer, int num_samples)
{

    int i;
    for (i = 0; i < num_samples; ++i) {
        int16_t sample = *buffer++;
        if (sample >= 32700)
            sample=32700;
	if( sample <= -32700)
	    sample=-32700;
        micbuff [(mic_head + i)%MIC_BUFFER_SAMPLES] = sample;
    }
    mic_head = (mic_head + num_samples)%MIC_BUFFER_SAMPLES;
    
    
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
    rdy_to_play=1;
    Audio_MAL_Play((uint32_t)out_buff0, OUT_BUFFER_SAMPLES*2 );
    EVAL_AUDIO_PauseResume(AUDIO_RESUME);
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size)
{
    if(!rdy_to_play){
	Audio_MAL_Play((uint32_t)out_buff0, OUT_BUFFER_SAMPLES*2 );
	return;
	
    }
    if(numbuff){
	Audio_MAL_Play((uint32_t)out_buff1, OUT_BUFFER_SAMPLES*2 );
	numbuff=0;
	fill_buffer(out_buff0,OUT_BUFFER_SAMPLES);
	fft_buffer(out_buff0,fft_buff0,OUT_BUFFER_SAMPLES);
	
    } else {
	 Audio_MAL_Play((uint32_t)out_buff0, OUT_BUFFER_SAMPLES *2);
	numbuff=1;
	fill_buffer(out_buff1,OUT_BUFFER_SAMPLES);
	fft_buffer(out_buff1,fft_buff1,OUT_BUFFER_SAMPLES);
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

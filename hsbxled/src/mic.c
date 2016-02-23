#include "mic.h"

/* SPI Configuration defines */
#define SPI_SCK_PIN                       GPIO_Pin_10
#define SPI_SCK_GPIO_PORT                 GPIOB
#define SPI_SCK_GPIO_CLK                  RCC_AHB1Periph_GPIOB
#define SPI_SCK_SOURCE                    GPIO_PinSource10
#define SPI_SCK_AF                        GPIO_AF_SPI2
#define SPI_MOSI_PIN                      GPIO_Pin_3
#define SPI_MOSI_GPIO_PORT                GPIOC
#define SPI_MOSI_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define SPI_MOSI_SOURCE                   GPIO_PinSource3
#define SPI_MOSI_AF                       GPIO_AF_SPI2

#define MIC_SPI_IRQHANDLER          SPI2_IRQHandler
#define REC_FREQ                32000     // Audio recording frequency in Hz
#define PCM_OUT_SIZE            40        // PCM buffer output size

// a lot comes from Christoph Lauer filter example
// but i won't use st pdm filter proprietary blob
// 1) because it's proprietary
// 2) because it's compiled with soft float (wasting silicon = meh)
// -> open source pdm fir lib by  Volkov Oleg, many thanks Oleg !
#define INTERNAL_BUFF_SIZE     8
static struct pdm_fir_filter Filter;

uint8_t MicInited=0;
uint32_t AudioRecBitRes = 16;             // The audio sample amplitude resolution
int16_t recBuff[PCM_OUT_SIZE];            // A pointer to an buffer
uint32_t AudioRecChnlNbr = 1;             // Audio recording number of channels (1 for Mono or 2 for Stereo)
uint16_t* pAudioRecBuf;                   // Main buffer pointer for the recorded data storing
uint32_t AudioRecCurrSize = 0;            // Current size of the recorded buffer

//static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE]; // Temporary data sample
static uint32_t InternalBufferSize = 0;



void mic_gpio_init()
{
     GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(SPI_MOSI_GPIO_CLK, ENABLE);

  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK , ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
  GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
  
  /* Connect SPI pins to AF5 */  
  GPIO_PinAFConfig(SPI_SCK_GPIO_PORT, SPI_SCK_SOURCE, SPI_SCK_AF);
  
  /* SPI MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPI_MOSI_PIN;
  GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF);
}

void mic_spi_init()
{
     I2S_InitTypeDef I2S_InitStructure;

  /* Enable the SPI clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
  
  /* SPI configuration */
  SPI_I2S_DeInit(SPI2);
  I2S_InitStructure.I2S_AudioFreq = 32000;//1024000/2;//I2S_AudioFreq_Default;
  I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
  /* Initialize the I2S peripheral with the structure above */
  I2S_Init(SPI2, &I2S_InitStructure);

  /* Enable the Rx buffer not empty interrupt */
  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
}

void mic_nvic_init()
{
    NVIC_InitTypeDef NVIC_InitStructure;

//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); 
  /* Configure the SPI interrupt priority */
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void mic_init()
{
    int i;
    
    if (MicInited)
    {
	/* No need for initialization */
	return ;
    }
     /* Enable CRC module */
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    mic_gpio_init();
    mic_spi_init();
    mic_nvic_init();
    MicInited=1;
    pdm_fir_flt_init(&Filter);

    //prep fire a bit

    for(i=0;i<8;i++)
		pdm_fir_flt_put(&Filter, 0);
     pdm_fir_flt_get(&Filter, 16);
    
    RCC_PLLI2SCmd(ENABLE); //lacking from examples
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
    I2S_Cmd(SPI2, ENABLE);
  
    
}



static __IO uint32_t recBuffFillness = 0;
__IO uint32_t Data_Status =0;

void MIC_SPI_IRQHANDLER(void)
{
 
    uint16_t app;
  
    
    /* Check if data are available in SPI Data register */
    if (SPI_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
    {
	SPI_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);
	
	app = SPI_I2S_ReceiveData(SPI2);
//	InternalBuffer[InternalBufferSize++] = SWAPBYTES_16(app);
    
	/* buffer full , Check to prevent overflow condition */
	InternalBufferSize++;
	   GPIO_ToggleBits(GPIOD, GPIO_Pin_12);	
	pdm_fir_flt_put(&Filter, app);
	    /* Check to prevent overflow condition */
	    if (InternalBufferSize >= INTERNAL_BUFF_SIZE)
	    {

		InternalBufferSize = 0;
		recBuff[recBuffFillness] = pdm_fir_flt_get(&Filter, 16);
		recBuffFillness++;
		if(recBuffFillness >= PCM_OUT_SIZE) {

		    data_rdy_callback(	recBuff,PCM_OUT_SIZE);
		    
		    Data_Status = 1;
		    recBuffFillness=0;
		}
	    }
	    
	   
		
		
		    
	    
    }
}


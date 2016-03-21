/**
  ******************************************************************************
  * @file    IO_Toggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?
#include "main.h"
/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


uint32_t * heap = (uint32_t *)0x10000000;

// see the  manual p 68 , this is 64k of CCM
// we have 65536 bytes free
// 1st 1024 bytes for the usart input
// 2nd 1024 bytes for FFT
// -> 1 "stick" (2 chips) -> 24 bytes
//room for 2645 "sticks" shouldbegoodenough(TM)

uint8_t * usart_temp ;
uint8_t * stick  ;

extern uint8_t * pulse_buff;
volatile uint8_t pulse_detected;

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */
    RCC_ClocksTypeDef RCC_Clocks;
    GPIO_InitTypeDef  GPIO_InitStructure;

    int i;
    usart_temp = (uint8_t *) heap;
    stick  =  ((uint8_t *)heap)+2048;



    
    /* SysTick end of count event each 1ms */
    RCC_GetClocksFreq(&RCC_Clocks);
    // SetSysClock();
    SysTick_Config(SystemCoreClock / 1000);
    
    /* GPIOD Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
    GPIOD->ODR = 0;
    GPIOA->ODR = 0;
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIOD->ODR = 0;
  
  mic_init();
  audio_output_init();

  usart_init();
  GPIOA->ODR = 0;
  usart_clrscrn();
  usart_send_string("HSBXLed uart interface, Welcome");
  usart_CRLF();

  ucsx912_init();
 
  GPIOD->ODR ^= (1<<14);

 

  
  
  // uint8_t stick[48];
  
  
  uint8_t col_r=0;
  uint8_t col_g=0;
  uint8_t col_b=0;
  
  
  GPIOD->ODR ^= (1<<14);
  
  while (1)
  {
      if(pulse_detected){
	  memset(usart_temp,0,7);
	  pulse_detected=0;
	  /* for(i=0;i<FREQUENCY_BUCKETS;i++){
	      *(usart_temp+i)='A'+((pulse_buff[i]>0)?1:0);
	  }
	  */
	   GPIOD->ODR ^= (1<<14);
//	  usart_send_string(usart_temp);
//	  usart_CRLF();	  
      }
      
      
      /*   Delay(100);

      for(i=0;i<16;i++){
	  stick[i*3]=col_r;
	  stick[i*3+1]=0;//col_r;
	  stick[i*3+2]=0;//col_r;  	  
      }
      ucsx912_senddata(stick,48);
      col_r+=10;
      */
      /*     if(col_r == 255){
	  col_g++;
	  if(col_g == 255){
	      col_b++;
	  }
	  }*/
  }
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00)
    { 
	TimingDelay--;
    }
    
}

void Delay(__IO uint32_t nTime)
{
   TimingDelay = nTime;
   while(TimingDelay != 0);
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

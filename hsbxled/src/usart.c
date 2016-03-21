#include "usart.h"

#define PIN_USART2_TX GPIO_Pin_2
#define PIN_USART2_RX GPIO_Pin_3
#define PIN_USART2_TX_Source GPIO_PinSource2
#define PIN_USART2_RX_Source GPIO_PinSource3
#define PORT_USART2 GPIOA

volatile unsigned char uart_ready_to_send=1;

char * uart_mess_to_send;
volatile char * uart_curr_send;
volatile unsigned int uart_sent_nbr;
char * uart_mess_to_receive;
volatile char * uart_curr_receive;
volatile unsigned int uart_received_nbr;

void usart_send_uint(uint32_t  n)
{
    // len(4294967295)=10
    int i;
    uint32_t tmp=n;
    char str[11];
    for(i=0;i<10;i++)
	str[i]=0;
    i=0;
    while(tmp){ // get nbr of digits
	tmp/=10;
	i++;
    }
    tmp=n;
    str[i]=0;
    str[0]='0';
    if(i)
	i--; // 0 based array
    
    while(i){
	str[i]='0'+tmp%10;
	tmp/=10;
	i--;
    }
    str[i]='0'+tmp;
    str[10]=0;
    usart_send_string(str);
    while(!uart_ready_to_send); // block until done , str is ON THE STACK 
}

void usart_send_string(char * str)
{
    // wait until uart stopped sending the current thing
    while(uart_ready_to_send==0);
    uart_ready_to_send=0;
    uart_mess_to_send=str;
    uart_curr_send=str;
    USART_Cmd(USART2,ENABLE);
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  // we sent a byte
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
//    USART2->DR=*str;
    
//    USART_SendData(USART2,*str);
    
}
void usart_send_byte(char c)
{
    char str[2];
    str[0]=c;
    str[1]=0;
    usart_send_string(str);
     while(!uart_ready_to_send); // block until done , str is ON THE STACK
   
    
}

void usart_CRLF(void)
{
    usart_send_string("\r\n");
    
}

void usart_clrscrn(void)
{
    usart_send_string("\033[2J\033[H");
    
}


void usart_init(void)
{
    USART_ClockInitTypeDef USART_ClockInitStruct;
    USART_InitTypeDef USART_InitStruct;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    //   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    USART_InitStruct.USART_BaudRate=9600;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits= USART_StopBits_1;  
    USART_InitStruct.USART_Parity=USART_Parity_No;
    USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
    USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    // UART clock
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_1Edge;
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable;
    // Configure GPIO mode 
    GPIO_InitStructure.GPIO_Pin =  PIN_USART2_TX| PIN_USART2_RX;
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    //apply gpio conf and set AF

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);       
    GPIO_PinAFConfig(PORT_USART2,PIN_USART2_RX_Source,GPIO_AF_USART2 );
    GPIO_PinAFConfig(PORT_USART2,PIN_USART2_TX_Source,GPIO_AF_USART2 );

    GPIO_Init(PORT_USART2, &GPIO_InitStructure);
    GPIOA->ODR = 0;   
    USART_Init(USART2, &USART_InitStruct);
    USART_ClockInit(USART2, &USART_ClockInitStruct);
    
    
    NVIC_Init(&NVIC_InitStructure);
    
    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  // we received a byte
   
    USART_Cmd(USART2,ENABLE);
  
    
   
}
/*
extern volatile unsigned char uart_ready_to_send;
extern volatile char * uart_curr_send;
extern volatile unsigned int uart_sent_nbr;
extern char * uart_mess_to_receive;
extern volatile char * uart_curr_receive;
extern volatile unsigned int uart_received_nbr;
*/
unsigned char pending_LF = 0;


void   USART2_IRQHandler(void)
{
    unsigned char c;
    
    if(USART_GetITStatus(USART2,USART_IT_TXE) != RESET){
	USART_ClearITPendingBit(USART2,USART_IT_TXE);
	if(pending_LF){
	    USART2->DR = '\n';
	    pending_LF=0;	    
	    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	    USART_ClearITPendingBit(USART2,USART_IT_TXE);
	    uart_ready_to_send=1;
	    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	    return;
	    
	    
	}
	
	if(uart_curr_send == NULL){    
	    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	}

	if(*uart_curr_send==0){
	    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	    
	    uart_ready_to_send=1;
	} else {
	    USART2->DR = *uart_curr_send;
	    uart_curr_send++; 
	}
	USART_ClearITPendingBit(USART2,USART_IT_TXE);	
    }
    if(USART_GetITStatus(USART2,USART_IT_RXNE)){
	// echo
	c = USART2->DR;
	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
	((volatile USART_TypeDef*)USART2)->DR=c;
	if(uart_curr_receive !=NULL){
	    *uart_curr_receive++ = c;   
	}
	if(c=='\r'){
	    uart_ready_to_send=0;
	    pending_LF=1;
	    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	}
    }
}

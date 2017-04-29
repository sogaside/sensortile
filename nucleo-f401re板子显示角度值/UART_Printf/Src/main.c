/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.4
  * @date    29-January-2016
  * @brief   This example shows how to retarget the C library printf function 
  *          to the UART.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "stdlib.h"
#include "stdio.h"
#include "lcd.h"
#include "stm32f4xx_hal_uart.h" 
uint8_t state=0;
uint8_t  *pTxBuffPtr1; 
uint16_t USART_RX_STA,count;         		//����״̬���
#define RXBUFFERSIZE   200 //�����С
uint8_t  aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���
#define USART_REC_LEN  			200  	//�����������ֽ��� 200  	
uint8_t  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
uint16_t watch_cr1,watch_cr2,watch_cr3,watch_sr,temp = 0;
uint8_t *x_char,*y_char,*z_char,x_receve[20],y_receve[20],z_receve[20];

void uart_init(uint32_t bound);
void beepms(uint16_t va);
void xianshi(void);//��ʾ��Ϣ  
void refshow(void);//ˢ����ʾ
void Load_Drow_Dialog(void)
{
	LCD_Clear(WHITE);//����   
 	POINT_COLOR=BLUE;//��������Ϊ��ɫ 
	LCD_ShowString(lcddev.width-24,0,200,16,16,(uint8_t*)"RST");//��ʾ��������
  	POINT_COLOR=RED;//���û�����ɫ 
}
void xianshi()//��ʾ��Ϣ
{ 
	LCD_Clear(WHITE);//���� 
	BACK_COLOR=WHITE ;
	POINT_COLOR=RED;   
	//��ʾ32*32����
	showhanzi32(0,0,0);	 //�����:
	showhanzi32(40,0,1);
	showhanzi32(80,0,2); 
	showhanzi32(120,0,3);  	
	showhanzi32(0,50,4);	 //������:
	showhanzi32(40,50,5);
	showhanzi32(80,50,6); 
	showhanzi32(120,50,7);  	
	showhanzi32(0,100,8);	 //��ת��:
	showhanzi32(40,100,9);	
	showhanzi32(80,100,10); 
	showhanzi32(120,100,11); 	
}
void showqq()
{ 
	uint16_t x,y; 
	x=0;
	y=75;
	while(y<lcddev.height-39)
	{
		x=0;
		while(x<lcddev.width-39)
		{
			showimage(x,y);	
			x+=40;
		}
		y+=40;
	 }	  
}
void refshow(void)	 //ˢ����ʾ
{
	switch(state)
	{
		case 0:
		LCD_Clear(WHITE);
	    xianshi();
    	showqq();
		break;
		case 1:
		LCD_Clear(BLACK);	
		break;
		case 2:
		LCD_Clear(RED);
		break;
		case 3:
		LCD_Clear(GREEN);
		break;
		case 4:
		LCD_Clear(BLUE);
		break;
	}	
}

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
static void SystemClock_Config(void);
static void Error_Handler(void);


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	uint16_t i,j;
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  x_char = x_receve; 
  y_char = y_receve;
  z_char = z_receve;
  /* Configure the system clock to 84 MHz */
  SystemClock_Config();
   
	uart_init(2400);
/* Output a message on Hyperterminal using printf function */
//  printf("\n\r UART Printf Example: retarget the C library printf function to the UART\n\r");
	SPIx_Init();	//SPI1��ʼ��
	LCD_Init();
   	POINT_COLOR=RED;//��������Ϊ��ɫ
  	xianshi();	   //��ʾ��Ϣ
//	LCD_Clear(WHITE);//����
	
//x,y:�������
//width,height:�����С  
//size:�����С
//*p:�ַ�����ʼ��ַ		  
//void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p)

//��ָ�����������ָ����ɫ��			 
//(sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex-sx+1)*(ey-sy+1)   
//color:Ҫ������ɫ
//void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color)
  while (1)
  {
	  x_char = x_receve;
	 y_char = y_receve;
	  z_char = z_receve;
//	LCD_Color_Fill(160,0,220,220,(uint16_t*)WHITE ) ; 
	LCD_ShowString(160,16,64,32,16,(uint8_t*)x_char);
	LCD_ShowString(160,66,64,32,16,(uint8_t*)y_char);
	LCD_ShowString(160,116,64,32,16,(uint8_t*)z_char);
  }
}


//��ʼ��IO ����2 
//bound:������
void uart_init(uint32_t bound)
{	
	UartHandle.Instance          = USART2;

	UartHandle.Init.BaudRate     = bound;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	if(HAL_UART_Init(&UartHandle) != HAL_OK)
	{
	/* Initialization Error */
	Error_Handler(); 
	}
	HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
}
//UART�ײ��ʼ����ʱ��ʹ�ܣ��������ã��ж�����
//�˺����ᱻHAL_UART_Init()����
//huart:���ھ��

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
	__HAL_RCC_USART2_CLK_ENABLE();			//ʹ��USART2ʱ��
	
	GPIO_Initure.Pin=GPIO_PIN_2;			//PA2 TX
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
	GPIO_Initure.Pull=GPIO_PULLUP;			//����
	GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
	GPIO_Initure.Alternate=GPIO_AF7_USART2;	//����ΪUSART2
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA2

	GPIO_Initure.Pin=GPIO_PIN_3;			//PA3 RX
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//
	GPIO_Initure.Pull=GPIO_PULLUP;			//����
	GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
	GPIO_Initure.Alternate=GPIO_AF7_USART2;	//����ΪUSART2			
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA3
		
	HAL_NVIC_EnableIRQ(USART2_IRQn);		//ʹ��USART2�ж�ͨ��
	HAL_NVIC_SetPriority(USART2_IRQn,3,3);	//��ռ���ȼ�3�������ȼ�3
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint32_t timeout=0,i,j;
	uint32_t maxDelay=0x1FFFF;
//	x_receve[0] = *x_char;
//	x_receve[1] = *(x_char+1);
//	x_receve[2] = *(x_char+2);
//	x_receve[3] = *(x_char+3);
//	x_receve[4] = *(x_char+4);
//	x_receve[5] = *(x_char+5);
//	x_char = x_receve;
//	
//	y_receve[0] = *y_char;
//	y_receve[1] = *(y_char+1);
//	y_receve[2] = *(y_char+2);
//	y_receve[3] = *(y_char+3);
//	y_receve[4] = *(y_char+4);
//	y_receve[5] = *(y_char+5);
//	y_char = y_receve;	
//	
//	z_receve[0] = *z_char;
//	z_receve[1] = *(z_char+1);
//	z_receve[2] = *(z_char+2);
//	z_receve[3] = *(z_char+3);
//	z_receve[4] = *(z_char+4);
//	z_receve[5] = *(z_char+5);
//	z_char = z_receve;	
	if(aRxBuffer[0] == 120) // x��ascii��
	{
//		x_receve[0]= (uint8_t)(((aRxBuffer[2]-0x30)*1000+(aRxBuffer[3]-0x30)*100+(aRxBuffer[4]-0x30)*10+aRxBuffer[5]-0x30)/1024*90)/10;
//		x_receve[1]= (uint8_t)(((aRxBuffer[2]-0x30)*1000+(aRxBuffer[3]-0x30)*100+(aRxBuffer[4]-0x30)*10+aRxBuffer[5]-0x30)/1024*90)%10;
		for(j = 0;j < 6;j++)
		{
			x_receve[j] = aRxBuffer[j];
		}		
	}
	else if(aRxBuffer[0] == 121)
	{
//		y_receve[0]= (uint8_t)(((aRxBuffer[2]-0x30)*1000+(aRxBuffer[3]-0x30)*100+(aRxBuffer[4]-0x30)*10+aRxBuffer[5]-0x30)/1024*90)/10;
//		y_receve[1]= (uint8_t)(((aRxBuffer[2]-0x30)*1000+(aRxBuffer[3]-0x30)*100+(aRxBuffer[4]-0x30)*10+aRxBuffer[5]-0x30)/1024*90)%10;	
		for(j = 0;j < 6;j++)
		{
			y_receve[j] = aRxBuffer[j];
		}		
	}
	else if(aRxBuffer[0] == 122)
	{
//		z_receve[0]= (uint8_t)(((aRxBuffer[2]-0x30)*1000+(aRxBuffer[3]-0x30)*100+(aRxBuffer[4]-0x30)*10+aRxBuffer[5]-0x30)/1024*90)/10;
//		z_receve[1]= (uint8_t)(((aRxBuffer[2]-0x30)*1000+(aRxBuffer[3]-0x30)*100+(aRxBuffer[4]-0x30)*10+aRxBuffer[5]-0x30)/1024*90)%10;
		for(j = 0;j < 6;j++)
		{
			z_receve[j] = aRxBuffer[j];
		}		
	}		
	while(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)//һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
		{
		 timeout++; //��ʱ����
		 if(timeout>maxDelay) break;	
		}	
	for(i = 0;i < RXBUFFERSIZE;i++ )
	{
		aRxBuffer[i] = 0;	
	}
}
/* �����жϺ���*/
void USART2_IRQHandler()
{
	uint32_t tmp1 = 0U, tmp2 = 0U;
  tmp1 = __HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE);
  tmp2 = __HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_RXNE);
  /* UART in mode Receiver ---------------------------------------------------*/
	if((tmp1 != RESET) && (tmp2 != RESET))
	{ 
		*UartHandle.pRxBuffPtr++ = (uint8_t)(UartHandle.Instance->DR & (uint8_t)0x00FFU);
		if(*UartHandle.pRxBuffPtr == 120)
		{
			x_char =UartHandle.pRxBuffPtr;
		}
		if(*UartHandle.pRxBuffPtr == 121)
		{
			y_char =UartHandle.pRxBuffPtr;
		}
		if(*UartHandle.pRxBuffPtr == 122)
		{
			z_char =UartHandle.pRxBuffPtr;
		}		
	}
	tmp1 = __HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_ORE);
	tmp2 = __HAL_UART_GET_IT_SOURCE(&UartHandle, UART_IT_ERR);
	/* UART Over-Run interrupt occurred ----------------------------------------*/
	if((tmp1 != RESET) && (tmp2 != RESET))
	{ 
		__HAL_UART_CLEAR_OREFLAG(&UartHandle);
		*UartHandle.pRxBuffPtr++ = (uint8_t)(UartHandle.Instance->DR & (uint8_t)0x00FFU);
		if(*UartHandle.pRxBuffPtr == 120)
		{
			x_char =UartHandle.pRxBuffPtr;
		}
		if(*UartHandle.pRxBuffPtr == 121)
		{
			y_char =UartHandle.pRxBuffPtr;
		}
		if(*UartHandle.pRxBuffPtr == 122)
		{
			z_char =UartHandle.pRxBuffPtr;
		}
		UartHandle.ErrorCode |= HAL_UART_ERROR_ORE;
	}
	if((--UartHandle.RxXferCount <= 1U)|(aRxBuffer[6]!=0))
	{
		count = 0;
		__HAL_UART_DISABLE_IT(&UartHandle, UART_IT_RXNE);

		/* Disable the UART Parity Error Interrupt */
		__HAL_UART_DISABLE_IT(&UartHandle, UART_IT_PE);

		/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
		__HAL_UART_DISABLE_IT(&UartHandle, UART_IT_ERR);

		/* Rx process is completed, restore huart->RxState to Ready */
		UartHandle.RxState = HAL_UART_STATE_READY;

		HAL_UART_RxCpltCallback(&UartHandle);
	}	  
	count++; 
}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF); 

  return ch;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while(1)
  {
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

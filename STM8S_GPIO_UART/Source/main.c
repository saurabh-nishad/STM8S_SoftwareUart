/**
  ******************************************************************************
  * @file    GPIO_Toggle\main.c
  * @author  MCD Application Team
  * @version V2.0.4
  * @date    26-April-2018
  * @brief   This file contains the main function for GPIO Toggle example.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "swuart.h"
/**
  * @addtogroup GPIO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Evalboard I/Os configuration */
#define LED_PORT 		(GPIOD)
#define LED_PIN 		(GPIO_PIN_0) // LEDs mask (EVAL board)

/* Private macro -------------------------------------------------------------*/

#define switch_LED_off		{ GPIO_WriteHigh(LED_PORT, LED_PIN); }	//LEDs control
#define switch_LED_on		{ GPIO_WriteLow(LED_PORT, LED_PIN); }

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t *data;
u8 tx_byte;								// transmitted byte
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief Programable loop delay
  * @par Parameters:
  * wt: number of loops
  * @retval None
  */
void delay_loop(u16 wt) {
	while(wt--);
}
/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  u8 buff, sts;
  
  /* Initialize I/Os in Output Mode */
  GPIO_Init(LED_PORT, (GPIO_Pin_TypeDef)LED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);

  uart_init();							// init pins and variables of SW UART
  
  uart_receive_enable;						// enable receive process
    
  enableInterrupts();						// enable interrupts
  data = "Hello how are you\r\n";
  
  while (1)
  {
    uart_send_string(data);

    switch_LED_on;						// LEDs on
    delay_loop(100);						// dummy loop
    sts = uart_read(&buff);					// read receive buffer
    if(sts != receive_buffer_full || buff != tx_byte) // check content of the status & data registers
		switch_LED_off;						// LEDs off if the data byte isn't correctly received
    delay_loop((u16)(50000));					// let the LEDs indication be observe  
  }

}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

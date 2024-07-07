/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "at32f403a_407_wk_config.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */

#include "at32f403a_407_int.h"
#include "usart_1.h" 
#include "stdio.h" 
#include "delay.h"
#include "led.h"
#include "AS5600.h"
#include "foc.h"

/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

uint8_t rxdata[4]={0};

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */

  /* add user code end 1 */

  /* system clock config. */
  wk_system_clock_config();

  /* config periph clock. */
  wk_periph_clock_config();

  /* init debug function. */
  wk_debug_config();

  /* nvic config. */
  wk_nvic_config();

  /* init usart1 function. */
  wk_usart1_init();

  /* init tmr1 function. */
  wk_tmr1_init();

  /* init tmr2 function. */
  wk_tmr2_init();

  /* init gpio function. */
  wk_gpio_config();

  /* add user code begin 2 */
  delay_init();
  angle_init();
  
  tmr_interrupt_enable(TMR1,TMR_OVF_INT,TRUE);
  
  usart_interrupt_enable(USART1,USART_RDBF_INT,TRUE);
  
  
//	int t=0,i=0;
//	float num=3.1415f;
  /* add user code end 2 */

  while(1)
  {
    /* add user code begin 3 */
		//x=angle_get();
		//printf("%lf\r\n",x);
	  //printf("%lf,%lf,%lf\r\n",Ua,Ub,Uc);
	  printf("%lf,%lf\r\n",tar,speed);

    /* add user code end 3 */
  }
}

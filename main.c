
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include "udp.h"
#include "ptp.c"


#define LWIP_PTP
#define z2_synq
#define z3_synq



#define ptp_port 	1234
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
ip_addr_t local_ip;
ip_addr_t remote_ip;
extern struct netif gnetif;
//struct udp_pcb *udpc;
//struct udp_pcb *udpc1;
//struct pbuf *pb1;
uint8_t tx_buf[2];
//uint16_t udp_rcv_cnt;
int32_t  bf_sytl, bf_syth, af_sytl, af_syth, afs6_l, afs6_h, bfs6_l, bfs6_h;
uint8_t i=0;
uint8_t synq_state;
int32_t rcv_buf[3];
//extern struct ETH_TimeStamp tx_ts;
extern ETH_TimeStamp tx_ts;
extern ETH_TimeStamp rx_ts;
ETH_TimeStamp 	t1,t2,t3,t4, offset, pr_delay, bw1,bw2, bw3, bw4, bw5, current_time,
								stf1, stf2, mtf1, mtf2, ts_tmp;
#ifdef z2_synq
	ETH_TimeStamp  mtf3, stf3; 
	uint16_t	fer7;
#endif

#ifdef z2_synq
	ETH_TimeStamp  mtf4, stf4; 
	uint16_t	fer8;
#endif
								
int32_t  mclk_count, diff_count;
double frq_scale_factor[4], sclk_count, mean_frq_scale_factor, tv_fsf1, tv_fsf2, tv_fsf3, tv_fsf4 ;
uint32_t addend, addend0;

uint8_t coarse_flag, offset_mod_flag;

uint16_t udp_320_cnt, port_copy, rad_ofset, s8_cnt;
//int32_t ptphdr;
uint8_t ptphdr,fer, cer, endpr, er_cnt;
//uint16_t max_allowed_offset;
//uint8_t ptp_timeout, test_tout_cnt;
err_t err2;
double  offset_ns_float, adm_rate;

int32_t  t_ofsset, endp_ofset;
uint16_t cn1, cn2, cn3, cn4, cn5, cn6, cn7, cn8, cn9, cn10, cn11, cn12;
uint32_t synq_interval, max_ofset;
uint16_t fer4, fer5, fer6;
uint8_t  cercnt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
//static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void ptp_start(void);
void udp_receive_callback(void *arg, struct udp_pcb *udpc, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void ip_asign(void);
//void udp_client_init(void);
void udp_slave_init(void);
void udp_send1(void);
void ETH_SetPTPTimeStampAddend(uint32_t Value);
void ETH_EnablePTPTimeStampAddend(void);
void ETH_PTP_GET_TIME( ETH_TimeStamp *time );
void ETH_PTP_SET_TIME( ETH_TimeStamp *time);
void ETH_InitializePTPTimeStamp(void);
void adj_freq( uint32_t addend );
int32_t my_abs(int32_t a);
//ETH_TimeStamp  minus_calc(ETH_TimeStamp* t1, ETH_TimeStamp* t2 );
//ETH_TimeStamp  minus_plus_calc(ETH_TimeStamp* t1, ETH_TimeStamp* t2, uint8_t opr );
//ETH_TimeStamp nsec_minus(ETH_TimeStamp* t);
void  minus_plus_calc(ETH_TimeStamp *time ,const ETH_TimeStamp *t1, const ETH_TimeStamp *t2, uint8_t opr );
void nsec_minus(ETH_TimeStamp *t);
//uint16_t change_allowed_offset(int32_t ofset_ns);
//void udp_send1(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim->Instance == TIM4)
	{
		if(synq_state > 3)		ptp_timeout++;
		if(ptp_timeout>3)
			{
				udp_send1();
				test_tout_cnt++;
				
				tx_buf[0] = 0x55;
				pb1 = pbuf_alloc(PBUF_TRANSPORT, 1, PBUF_RAM);
				err2 = pbuf_take(pb1, tx_buf, 1);
				if(err2== ERR_OK) 
					{
					//Connect to the remote client 
					udp_connect(udpc1, &remote_ip, port_copy);
					udp_send(udpc1, pb1);
					pbuf_free(pb1);
					ptp_timeout = 0;
					test_tout_cnt++;
					}
			
			}
		
	}

}*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_LWIP_Init();
  //MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	ptp_start();
	ip_asign();
  udp_slave_init();
	
	//max_allowed_offset = first_max_allowed_offset;
  //HAL_TIM_Base_Start_IT(&htim4);
	//coarse_flag = 1;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
   MX_LWIP_Process();
  /* USER CODE END WHILE */
  
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
/*static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4000; // 1000 is 9.25ms
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}*/

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB14 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ptp_start(void)
{
	
	//assert_param(IS_ETH_PTP_UPDATE(UpdateMethod));
	ETH->MACIMR |=  0x200;  // disable timestamp trigger interrupt generation
  ETH-> PTPTSCR |= 0x00000001;   // enable ptp time stamping
	//ETH-> PTPSSIR = 8;     //  in coarse method: sub_second increment register  0XA:FOR 96MHZ
													// : 5 FOR 216MHZ
	ETH-> PTPSSIR = 6;
	
	ETH_SetPTPTimeStampAddend( Base_addend ); // hclk = 216M 
	ETH_EnablePTPTimeStampAddend();
	while(  ETH->PTPTSCR & 0x00000020 );
	
	ETH->PTPTSCR |= 0x00000002;  // fine correction method
	
	
	
	ETH->PTPTSCR |= 0x00000200; // sub_second reg overflow when recieve 999 999 999
	
	//ETH->PTPTSCR |= 0x00000c00;
  ETH->PTPTSCR |= 0x00000100; //set TSSARFE bit -> timestamp enable for all rcv frames
	
	ETH->PTPTSHUR = 0;
	ETH->PTPTSLUR = 0;
	ETH-> PTPTSCR |= 0x00000004;  // set bit2 = tssti  time stamp system time initialize
	while(ETH->PTPTSCR & 0x4){};
	
	//Enable enhanced descriptor structure 
    ETH->DMABMR |= ETH_DMABMR_EDE;
		
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void ip_asign(void)
{
local_ip = gnetif.ip_addr;
 ip4addr_aton("192.168.1.110", &remote_ip); // .100 for stm    .50 for pc 
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++

/*void udp_client_init(void)
{
	
	err_t err;
	
	 // Create a new UDP control block  
  udpc = udp_new();
  
  if (udpc!=NULL)
  {
    //assign destination IP address 
    //IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );
    
    // configure destination IP address and port
    err= udp_connect(udpc, &remote_ip, 320);
    
    if (err == ERR_OK)
    {
      // Set a receive callback for the upcb 
      udp_recv(udpc, udp_receive_callback, NULL);  
    }
		else udp_remove(udpc);
  }
}*/


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void udp_slave_init(void)
{
   struct udp_pcb *udpc;
   err_t err;
   
   /* Create a new UDP control block  */
   udpc = udp_new();
   
   if (udpc)
   {
     /* Bind the upcb to the UDP_PORT port */
     /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
      err = udp_bind(udpc, &local_ip, ptp_port);
      
      if(err == ERR_OK)
      {
        /* Set a receive callback for the upcb */
        udp_recv(udpc, udp_receive_callback, NULL);
      }
      else
      {
        udp_remove(udpc);
      }
   }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*void udp_send1(void)
{
  struct pbuf *pb;
	struct udp_pcb *udpc1;
	uint16_t len;
	err_t err;
	uint8_t buf[2];
	
	//len = sprintf(buf, "a");
	//tx_buf[0] = 0x55;
	//buf[1] = 0x57;
	buf[0] = 0x55;
	len = 1;
	
	// Create a new UDP control block  
   udpc1 = udp_new();
	
	pb = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
	err = pbuf_take(pb, buf, len);
	if(err == ERR_OK )
	 {
		udp_connect(udpc1, &remote_ip, 1230);
		 
		udp_send(udpc1, pb);
		//tx_tsl = heth.TxDesc->TimeStampLow;
		//pbuf_free(pb);
		 
		 udp_disconnect(udpc1);
		 
		 pbuf_free(pb);
		 //udp_remove(udpc);
	 }
}*/
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void udp_receive_callback(void *arg, struct udp_pcb *udpc, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{ 
	err_t  err1;
	int32_t temp_int;
	
	
	port_copy = port;
	//ptp_timeout = 0;
	
	//udpn320_cnt++;
	if(port == ptp_port)
		{
			udp_320_cnt++;
			synq_state++;
	pbuf_copy_partial( p, rcv_buf, p->len, 0); 
	ptphdr = (rcv_buf[2] & 0xff);
	if(synq_state!= ptphdr)
		{
			if(synq_state > 3)	// untill corase rcv packets ok
				{
					//tx_buf[0] = 0x55; 
					tx_buf[0] = (0x80 | synq_state);
					if(synq_state==4)				{fer4++;}
					else if(synq_state==5)	{fer5++;}
				  else if(synq_state==6)  {fer6++;}
					else if(synq_state==7)  {fer7++;}
					else if(synq_state==8)  {fer8++;}
					
					synq_state--;
					//fer = tx_buf[0];
					//synq_state = 3;
				}  
			else		
				{
					tx_buf[0] = 0x66;
					synq_state = 0;
					//cer = tx_buf[0];
					cercnt++;
				}
			err1 = pbuf_take(p, tx_buf, 1);
			if(err1== ERR_OK) 
					{
					//Connect to the remote client 
					udp_connect(udpc, addr, port);
					udp_send(udpc, p);
					}
			 // free the UDP connection, so we can accept new clients 
			//udp_disconnect(udpc);
		}
	else if(synq_state==1)	
		{
			t2 = rx_ts;
			synq_interval = (rcv_buf[2] >>8) * 1000000;
		}
	else if(synq_state==2)	
			{
			 //pbuf_copy_partial( p, rcv_buf, p->len, 0); 
			 t1.TimeStampHigh = rcv_buf[1];
			 t1.TimeStampLow  = rcv_buf[0];
			 tx_buf[0] = 0x12;
			 err1 = pbuf_take(p, tx_buf, 1);
			 if(err1== ERR_OK) 
					{
					//Connect to the remote client 
					udp_connect(udpc, addr, port);
					udp_send(udpc, p);
					}
			 // free the UDP connection, so we can accept new clients 
			 //udp_disconnect(udpc);
			 //udp_send1();
			 t3 = tx_ts;
			}
	else if(synq_state==3)
				{
				 bf_syth = ETH->PTPTSHR;
				 bf_sytl = ETH->PTPTSLR;
				 //pbuf_copy_partial( p, rcv_buf, p->len, 0); 
				 t4.TimeStampHigh = rcv_buf[1];
				 t4.TimeStampLow  = rcv_buf[0];
					
				 //synq_state = 0;
					//coarse_flag = 0;
					
					minus_plus_calc( &bw1, &t2, &t1, minus );
					minus_plus_calc( &bw2, &t4, &t3, minus );
					nsec_minus( &bw1 ); // if sec is neg, also apply neg in nsec 
					nsec_minus( &bw2 );
					
					minus_plus_calc(  &bw4, &bw2, &bw1, minus); 
					
					
					offset.TimeStampHigh = bw4.TimeStampHigh /2;
					offset.TimeStampLow = bw4.TimeStampLow /2;
					
					//max_allowed_offset = change_allowed_offset(my_abs(offset.TimeStampLow));
					
					temp_int = my_abs(offset.TimeStampLow);
					//ETH_PTPTime_UpdateOffset( offset.TimeStampHigh  , offset.TimeStampLow);
					if( (offset.TimeStampHigh != 0) || (temp_int > max_allowed_offset))
						{
							ETH_PTP_GET_TIME( &current_time );
							minus_plus_calc( &bw5, &current_time, &offset, plus);
							ETH_PTP_SET_TIME(&bw5);
							HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
							rad_ofset++;
							offset_mod_flag = 1;
						}
					else
						{
							offset_ns_float = offset.TimeStampLow;
						  //offset_ns_float = -(stf4.TimeStampLow - mtf4.TimeStampLow - pr_delay.TimeStampLow)/2;
						  adm_rate = (offset_ns_float/synq_interval) + 1;
							addend0 = ETH->PTPTSAR;
							addend = adm_rate * addend0 ;
							adj_freq( addend );
							t_ofsset = my_abs(offset.TimeStampLow);
							if(t_ofsset < 50 ){ cn1++;}
							else if(t_ofsset < 100 ){ cn2++;}
							else if(t_ofsset < 150 ){ cn3++;}
							else if(t_ofsset < 200 ){ cn4++;}
							else if(t_ofsset < 250 ){ cn5++;}
							else if(t_ofsset < 300 ){ cn6++;}
							else if(t_ofsset < 400 ){ cn7++;}
							else if(t_ofsset < 500 ){ cn8++;}
							else if(t_ofsset < 600 ){ cn9++;}
							else if(t_ofsset < 800 ){ cn10++;}
							else if(t_ofsset < 1000 ){ cn11++;}
							else  	cn12++;
							
							if(s8_cnt > 10) 
								{
									if(t_ofsset > max_ofset) 	max_ofset = t_ofsset;
								}
						}
					//max_allowed_offset = change_allowed_offset(my_abs(offset.TimeStampLow));
					minus_plus_calc(  &bw3, &bw1, &bw2, plus);
					pr_delay.TimeStampHigh = bw3.TimeStampHigh / 2 ;
					pr_delay.TimeStampLow = bw3.TimeStampLow / 2;
					af_syth = ETH->PTPTSHR;
					af_sytl = ETH->PTPTSLR;
						
				
					//delay = ((t2-t1) + (t4-t3))/2;
				}
	else if(synq_state==4)  
		{
		stf1 = rx_ts;
		//synq_interval = (rcv_buf[2] >>8) * 1000000;
		}
	else if(synq_state==5) 
				{
				 stf2 = rx_ts; 
				 mtf1.TimeStampHigh = rcv_buf[1];
				 mtf1.TimeStampLow =  rcv_buf[0];
				 /*tx_buf[0] = 0x23;
				 err1 = pbuf_take(p, tx_buf, 1);
				 if(err1== ERR_OK) 
						{
						//Connect to the remote client 
						udp_connect(udpc, addr, port);
						udp_send(udpc, p);
						}
				 // free the UDP connection, so we can accept new clients 
				 udp_disconnect(udpc);*/
				}
	else if(synq_state==6)
			{
			 bfs6_h = ETH->PTPTSHR;
			 bfs6_l = ETH->PTPTSLR;
			
			 #ifdef z2_synq
					stf3 = rx_ts;
			 #endif
			 //pbuf_copy_partial( p, rcv_buf, p->len, 0);
			 mtf2.TimeStampHigh = rcv_buf[1];
			 mtf2.TimeStampLow = rcv_buf[0];
			 minus_plus_calc( &ts_tmp ,&stf2, &stf1, minus);
			 sclk_count = ts_tmp.TimeStampLow;
			 minus_plus_calc( &ts_tmp ,&mtf2, &mtf1, minus);
			 mclk_count = ts_tmp.TimeStampLow;
			 diff_count = mclk_count - sclk_count;
			 frq_scale_factor[0] = (((mclk_count + diff_count) / sclk_count)+1)/2 ;
			 //frq_scale_factor[0] = (mclk_count + diff_count) / sclk_count;	
			 #ifndef z2_synq
				 addend0 = ETH->PTPTSAR;
				 addend = frq_scale_factor[0] * addend0 ;
					
				 adj_freq( addend );
					
				 synq_state = 0;
				 s8_cnt++;
				 tx_buf[0] = 0x77;
				err1 = pbuf_take(p, tx_buf, 1);
				if(err1== ERR_OK) 
					{
					udp_connect(udpc, addr, port);
					udp_send(udpc, p);
					}
				 //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
			 #endif
			 afs6_h = ETH->PTPTSHR;
			 afs6_l = ETH->PTPTSLR;	
			}
			
		#ifdef z2_synq
		else if(synq_state==7)
			{
			 #ifdef z3_synq
					stf4 = rx_ts;
			 #endif
			 mtf3.TimeStampHigh = rcv_buf[1];
			 mtf3.TimeStampLow = rcv_buf[0];
			 minus_plus_calc( &ts_tmp ,&stf3, &stf2, minus);
			 sclk_count = ts_tmp.TimeStampLow;
			 minus_plus_calc( &ts_tmp ,&mtf3, &mtf2, minus);
			 mclk_count = ts_tmp.TimeStampLow;
			 diff_count = mclk_count - sclk_count;
			 frq_scale_factor[1] = (((mclk_count + diff_count) / sclk_count)+1)/2 ;
			 tv_fsf1 = frq_scale_factor[1];
			 //frq_scale_factor[1] = (mclk_count + diff_count) / sclk_count;
			 //addend0 = ETH->PTPTSAR;
			 #ifndef z3_synq
				 addend0 = ETH->PTPTSAR;
				 mean_frq_scale_factor = (frq_scale_factor[0] + frq_scale_factor[1])/2;				
				 addend = mean_frq_scale_factor * addend0 ;
					
				 adj_freq( addend );
					
				 synq_state = 0;	
				 s8_cnt++;
				 tx_buf[0] = 0x77;
				err1 = pbuf_take(p, tx_buf, 1);
				if(err1== ERR_OK) 
					{
					udp_connect(udpc, addr, port);
					udp_send(udpc, p);
					}
			 #endif
			
			
			}
		#endif
			
		#ifdef z3_synq	
		else if(synq_state==8)
			{
			 mtf4.TimeStampHigh = rcv_buf[1];
			 mtf4.TimeStampLow = rcv_buf[0];
			 minus_plus_calc( &ts_tmp ,&stf4, &stf3, minus);
			 sclk_count = ts_tmp.TimeStampLow;
			 minus_plus_calc( &ts_tmp ,&mtf4, &mtf3, minus);
			 mclk_count = ts_tmp.TimeStampLow;
			 diff_count = mclk_count - sclk_count;
			 frq_scale_factor[2] = (((mclk_count + diff_count) / sclk_count)+1)/2 ;
			 tv_fsf2 = frq_scale_factor[2];
			 //frq_scale_factor[2] = (mclk_count + diff_count) / sclk_count;

				
				
			 minus_plus_calc( &ts_tmp ,&stf4, &stf1, minus);
			 sclk_count = ts_tmp.TimeStampLow;
			 minus_plus_calc( &ts_tmp ,&mtf4, &mtf1, minus);
			 mclk_count = ts_tmp.TimeStampLow;
			 diff_count = mclk_count - sclk_count;
			 frq_scale_factor[3] = (((mclk_count + diff_count) / sclk_count)+1)/2 ;
			 tv_fsf3 = frq_scale_factor[3];
				
			 /*minus_plus_calc( &ts_tmp ,&stf3, &stf1, minus);
			 sclk_count = ts_tmp.TimeStampLow;
			 minus_plus_calc( &ts_tmp ,&mtf3, &mtf1, minus);
			 mclk_count = ts_tmp.TimeStampLow;
			 diff_count = mclk_count - sclk_count;
			 frq_scale_factor[4] = (((mclk_count + diff_count) / sclk_count)+1)/2 ;
			 tv_fsf4 = frq_scale_factor[4];*/
			 
			 
		  
			
			 mean_frq_scale_factor = (frq_scale_factor[0] + frq_scale_factor[1] + frq_scale_factor[2] + frq_scale_factor[3])/4;
					
			 //if( /*(my_abs(offset.TimeStampLow) > 1000) &&*/ offset_mod_flag==0)
			 #ifdef end_adm_rate
			 if( offset_mod_flag==0 )
					{
						//offset_ns_float = offset.TimeStampLow;
						offset_ns_float = (offset.TimeStampLow - (stf4.TimeStampLow - mtf4.TimeStampLow - pr_delay.TimeStampLow))/2;
						//offset_ns_float = -(stf4.TimeStampLow - mtf4.TimeStampLow - pr_delay.TimeStampLow)/2;
						adm_rate = (offset_ns_float/synq_interval) + 1;
						mean_frq_scale_factor = adm_rate * mean_frq_scale_factor;
						
						t_ofsset = my_abs(offset.TimeStampLow);
						if(t_ofsset < 50 ){ cn1++;}
						else if(t_ofsset < 100 ){ cn2++;}
						else if(t_ofsset < 150 ){ cn3++;}
						else if(t_ofsset < 200 ){ cn4++;}
						else if(t_ofsset < 250 ){ cn5++;}
						else if(t_ofsset < 300 ){ cn6++;}
						else if(t_ofsset < 400 ){ cn7++;}
						else if(t_ofsset < 500 ){ cn8++;}
						else if(t_ofsset < 600 ){ cn9++;}
						else if(t_ofsset < 800 ){ cn10++;}
						else if(t_ofsset < 1000 ){ cn11++;}
				    else  	cn12++;
						
						if(s8_cnt > 10) 
							{
								if(t_ofsset > max_ofset) 	max_ofset = t_ofsset;
							}
					}
				#endif
				endp_ofset = -(stf4.TimeStampLow - mtf4.TimeStampLow - pr_delay.TimeStampLow);

			 addend0 = ETH->PTPTSAR;
			 addend = mean_frq_scale_factor * addend0 ;
			
			 adj_freq( addend );
				
			 synq_state = 0;	
			 s8_cnt++;
			 offset_mod_flag  = 0;
					
					
			 tx_buf[0] = 0x77;
			 err1 = pbuf_take(p, tx_buf, 1);
			 if(err1== ERR_OK) 
					{
					//Connect to the remote client 
					udp_connect(udpc, addr, port);
					udp_send(udpc, p);
					}
			 // free the UDP connection, so we can accept new clients 
			 //udp_disconnect(udpc);
		
			}
		#endif
		
		}			
	

	
  /* Free receive pbuf */
  pbuf_free(p);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*ETH_TimeStamp nsec_minus(ETH_TimeStamp* t)
{
  ETH_TimeStamp time;
	
	if(t->TimeStampHigh <0 )	{ t->TimeStampLow = -(t->TimeStampLow); }
	time.TimeStampHigh = t->TimeStampHigh;
	time.TimeStampLow  = t->TimeStampLow;
	return time;
	//return *t;
}*/


void nsec_minus(ETH_TimeStamp *t)
{
  //ETH_TimeStamp time;
	
	if(t->TimeStampHigh <0 )	{ t->TimeStampLow = -(t->TimeStampLow); }
	//time.TimeStampHigh = t->TimeStampHigh;
	//time.TimeStampLow  = t->TimeStampLow;
	//return time;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* ETH_TimeStamp  minus_plus_calc(ETH_TimeStamp* t1, ETH_TimeStamp* t2, uint8_t opr )
{
//int32_t  secdiff, nsecdiff;
 ETH_TimeStamp time;

	if(opr==1) // minus
		{
		 time.TimeStampHigh =  t1->TimeStampHigh - t2->TimeStampHigh;
		 time.TimeStampLow =   t1->TimeStampLow - t2->TimeStampLow;
		}
	else   // plus
		{
     time.TimeStampHigh =  t1->TimeStampHigh + t2->TimeStampHigh;
		 time.TimeStampLow =   t1->TimeStampLow + t2->TimeStampLow;
		}
	
	
	
	if( time.TimeStampHigh<0 && time.TimeStampLow>0 )
	{
	 time.TimeStampHigh +=1;
   time.TimeStampLow  = biliard - time.TimeStampLow;	
   if(time.TimeStampHigh==0) { time.TimeStampLow = -(time.TimeStampLow); }		
	}
	else if( time.TimeStampHigh>0 && time.TimeStampLow<0 ) 
	{
	 time.TimeStampHigh = time.TimeStampHigh - 1; 
	 time.TimeStampLow = time.TimeStampLow + biliard;
	}
	
	return time;
	
}*/



void  minus_plus_calc(ETH_TimeStamp *time ,const ETH_TimeStamp *t1, const ETH_TimeStamp *t2, uint8_t opr )
{
//int32_t  secdiff, nsecdiff;
 //ETH_TimeStamp time;

	if(opr==1) // minus
		{
		 time->TimeStampHigh =  t1->TimeStampHigh - t2->TimeStampHigh;
		 time->TimeStampLow =   t1->TimeStampLow - t2->TimeStampLow;
		}
	else   // plus
		{
     time->TimeStampHigh =  t1->TimeStampHigh + t2->TimeStampHigh;
		 time->TimeStampLow =   t1->TimeStampLow + t2->TimeStampLow;
		}
	
	
	
	if( time->TimeStampHigh<0 && time->TimeStampLow>0 )
	{
	 time->TimeStampHigh +=1;
   time->TimeStampLow  = biliard - time->TimeStampLow;	
   if(time->TimeStampHigh==0) { time->TimeStampLow = -(time->TimeStampLow); }		
	}
	else if( time->TimeStampHigh>0 && time->TimeStampLow<0 ) 
	{
	 time->TimeStampHigh = time->TimeStampHigh - 1; 
	 time->TimeStampLow = time->TimeStampLow + biliard;
	}
	
	
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ETH_PTP_GET_TIME( ETH_TimeStamp *time )
{
	time->TimeStampHigh = ETH->PTPTSHR;
	time->TimeStampLow = ETH->PTPTSLR;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ETH_PTP_SET_TIME( ETH_TimeStamp *time)
{
	

	uint32_t Sign;
	uint32_t SecondValue;
	uint32_t NanoSecondValue;
	//uint32_t SubSecondValue;

	/* determine sign and correct Second and Nanosecond values */
	if( time->TimeStampHigh < 0 || (time->TimeStampHigh == 0 && time->TimeStampLow < 0))
	{
		Sign = ETH_PTP_NegativeTime;
		SecondValue = -time->TimeStampHigh;
		NanoSecondValue = -time->TimeStampLow;
	}
	else
	{
		Sign = ETH_PTP_PositiveTime;
		SecondValue = time->TimeStampHigh;
		NanoSecondValue = time->TimeStampLow;
	}

	

	/* Write the offset (positive or negative) in the Time stamp update high and low registers. */
	ETH_SetPTPTimeStampUpdate(Sign, SecondValue, NanoSecondValue);
	/* Set Time stamp control register bit 2 (Time stamp init). */
	ETH_InitializePTPTimeStamp();
	/* The Time stamp counter starts operation as soon as it is initialized
	 * with the value written in the Time stamp update register. */
	while( ETH->PTPTSCR & 0x00000004 );

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*uint16_t change_allowed_offset(int32_t ofset_ns)
{
	uint16_t  allow_offset = 0;
	//uint8_t   shmit_flag = 0;
	
	if(ofset_ns < 400) 	allow_offset  = 500; // 500-800 is 500--- shmit triger
	else 								allow_offset  = 1900;
	//else if(ofset_ns > 1300)  allow_offset  = 1900;
	//else if(ofset_ns < 700)   allow_offset  = 1000;
	//else 											allow_offset  = 1900;
	
	return allow_offset;
}*/
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*uint8_t  shmit_trg (uint16_t thr)
{
	uint8_t  thro = 0;
	
	if()


}*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

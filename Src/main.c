/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#define TEST_CASE       "F4:03/01/2017\r\n"

// Trigger:
// OUT: LD6-BLUE    PD15
// IN:  BUTTON      PA0

/* Notes_09/12/2016:
- START_TRIGGER: GPIO_MODE_IT_RISING_FALLING
- DMA: NORMAL
- Rising/Falling edge of output trigger  _____--__________--_____

* Notes_17/12/2016:
- calc phase_shift use atan2 --> failed: phase offset @each cycle
- CAN transmit error (^.^)

* Notes_28/12/2016:
- trigger: 3000000

*Note_02/01/2017:
- Smart Threshold V2
- Trigger in CAN BUS
- Filter for Main Receiver
*/

// arm cmsis library includes
#define ARM_MATH_CM4
#include "arm_math.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const uint16_t PROCESS_WINDOW = 28000;
const uint16_t PROCESS_CYCLE  = 1;    //5m  max, RF_Delay >0.4ms => -200
const uint16_t BUFFER_SIZE    = 28000;

uint16_t ADC_buf[BUFFER_SIZE];
uint16_t i,j,k,pre_j;
uint16_t trig_cycle, init_cycle;
uint16_t max_cycle, min_cycle;

uint64_t THRESHOLD[4];

char print_en;

float32_t temp_sin[PROCESS_WINDOW];
float32_t temp_cos[PROCESS_WINDOW];

float32_t res_sin[PROCESS_CYCLE];
float32_t res_cos[PROCESS_CYCLE];
float32_t calc_res[PROCESS_CYCLE];
float32_t phase_res;

float64_t max_val, min_val;

const float32_t sin_ref[35] = {0, 0.179, 0.351, 0.513, 0.658, 0.782, 0.881, 0.951, 0.991, 0.999, 0.975, 0.920, 0.835, 0.723, 0.588, 0.434, 0.266, 0.0900, -0.0900, -0.266, -0.434, -0.588, -0.723, -0.835, -0.920, -0.975, -0.999, -0.991, -0.951, -0.881, -0.782, -0.658, -0.513, -0.351, -0.179};
const float32_t cos_ref[35] = {1, 0.984, 0.936, 0.858, 0.753, 0.623, 0.474, 0.309, 0.134, -0.045, -0.223, -0.393, -0.551, -0.691, -0.809, -0.901, -0.964, -0.996, -0.996, -0.964, -0.901, -0.809, -0.691, -0.551, -0.393, -0.223, -0.0450, 0.134, 0.309, 0.474, 0.623, 0.753, 0.858, 0.936, 0.984};

CanTxMsgTypeDef TxM;
CanRxMsgTypeDef RxM;
CAN_FilterConfTypeDef sFilterConfig;
uint8_t a;
uint8_t ui8_my_addr;
  
enum system_mode
{
  TRIGGER_MODE,
  DEBUG_MODE,
} system_mode;  

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
 
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
  return ch;
}

void CAN_Set_Node_Addr(uint8_t addr);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  system_mode = DEBUG_MODE;
  print_en = 0; 
  THRESHOLD[0] = 0.5*1000000;     // Energy @ starting point  
  THRESHOLD[1] = 0*1000000;         // Minimum max value --> discard calc value
  THRESHOLD[2] = 100*1000000;     // Maximum max value --> stop update max --> fix bug when too close
  
  // 1x: NODE
  // 2x: MAIN RECEIVER
  CAN_Set_Node_Addr(15);

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  printf(TEST_CASE);
  printf("\r\nTRIGGER_MODE\r\n");
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    //================== Wave Detecting Algorithm ==================//
    // Detect change of proces cycle --> Process previous cycle
//    if(pre_j != j){     
//      // Calculate Wave Energy
//      res_sin[pre_j] = 0;
//      res_cos[pre_j] = 0;
//      for(i=0; i<PROCESS_WINDOW; i++){
//        res_sin[pre_j] += sin_ref[i]*ADC_buf[PROCESS_WINDOW*pre_j+i];
//        res_cos[pre_j] += cos_ref[i]*ADC_buf[PROCESS_WINDOW*pre_j+i];
//      }
//      calc_res[pre_j] = res_sin[pre_j]*res_sin[pre_j] + res_cos[pre_j]*res_cos[pre_j];
//            
//      // Update pre_j value
//      pre_j = j;    
//      
//      //=============== Smart Threshold Algorithm V2 ===============//
//      
//      // Get max value & max_cycle
//      if((calc_res[pre_j-1] > max_val)&&(max_val<THRESHOLD[2])){      // Get max value
//          max_cycle = pre_j-1;
//          max_val = calc_res[max_cycle];          
//      }
//            
//      // When buffer is full/max detected --> Trace back initial wave cycle
//      if((j == PROCESS_CYCLE-200)||(max_val>THRESHOLD[2])){
//        HAL_ADC_Stop_DMA(&hadc1);

//        // Trace back initial wave cycle
//        for(k=1;k<30;k++){
//          if(calc_res[max_cycle-k] < THRESHOLD[0]){
//            init_cycle = max_cycle-k+1; 
//            break;    // 1st time valid
//          }
//        }
//        
//        // Filter: Eleminate result when wave is too weak 
////        if(max_val < THRESHOLD[1]){
////          init_cycle = 0;       // Out of range
////        }
//        
//        // Turn of flag to print result 
//        print_en = 1; 
//        
//        // SEND DISTANCE TO CAN BUS
//        hcan2.pTxMsg->Data[0] = ui8_my_addr;            // TX_ID
//        hcan2.pTxMsg->Data[1] = init_cycle>>8;          // DATA 1
//        hcan2.pTxMsg->Data[2] = init_cycle&~(0xFF00);   // DATA 2
//        hcan2.pTxMsg->Data[3] = 'D';                    // COMMAND
//        hcan2.pTxMsg->Data[4] = 20;                     // RX_ID
//        if(HAL_CAN_Transmit(&hcan2,5) != HAL_OK){
//          printf("Send Fail\r\n");
//        }       
//      }
//    }
    
    //================== System Mode ==================//
    switch (system_mode){
      case TRIGGER_MODE:
        if(print_en == 1){
//          printf("%d %d\r\n",init_cycle,trig_cycle-init_cycle);
          printf("%d %d\r\n",ui8_my_addr, init_cycle);      
          print_en = 0;
        }
        break;
      
      case  DEBUG_MODE:
        if(print_en == 1){
          print_en = 2;
//          printf("I^2+Q^2\r\n");
//          for(k=0; k<PROCESS_CYCLE; k++){
//            printf("%f\r\n",calc_res[k]);
//            //HAL_Delay(1);
//          }
          //printf("trig_c %i\r\n",trig_cycle);
          for(k=0; k<BUFFER_SIZE; k++){
            printf("%i ",ADC_buf[k]);
            //HAL_Delay(1);
          }
          printf("\r\n\n");
        } 
        break;
    }
  }

  /* USER CODE END 3 */

}

/** System Clock Configuration
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CAN2 init function */
static void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_3TQ;
  hcan2.Init.BS1 = CAN_BS1_9TQ;
  hcan2.Init.BS2 = CAN_BS2_4TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = DISABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  // Config CAN Filter for CAN2
  hcan2.pTxMsg = &TxM;
  hcan2.pRxMsg = &RxM;
  
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = 20;          // Filter ID (11bit MSBs)
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0x0700;  //0000 0111 0000 0000
  sFilterConfig.FilterMaskIdLow = 0x0700;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 0;
  
  HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
  
  hcan2.pTxMsg->StdId = ui8_my_addr;      // TX ID
  hcan2.pTxMsg->ExtId = 0x0;
  hcan2.pTxMsg->RTR = CAN_RTR_DATA;
  hcan2.pTxMsg->IDE = CAN_ID_STD;
  hcan2.pTxMsg->DLC = 5;      // 1 TX_ID, 2_DATA, 1_COMMAND/CONFIG, 1_RX_ID
  
  
  // Start the Reception process and enable reception interrupt 
  if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) !=HAL_OK){
    printf("rev Init fail\r\n");
  }
}

void CAN_Set_Node_Addr(uint8_t addr)
{
  ui8_my_addr = addr;
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_SW_Pin */
  GPIO_InitStruct.Pin = USER_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_SW_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

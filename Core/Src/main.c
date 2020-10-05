/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define BUFFERSIZE 256

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

__IO ITStatus Uart1Ready = RESET;

uint8_t aTxBufferPos = 0;
uint8_t aTxBuffer[256];

//uint8_t aRxBufferPos = 0;
//uint8_t aRxBuffer[BUFFERSIZE];

struct MBCmds {
   uint8_t  cmdlen;
   uint8_t  cmd[256];
};

struct MBCmds mbcmds[10] = {
    {1, {0x53}},                      //TEST TAMAGAWA
};

struct tmgw_struc {
   uint8_t cf;
   uint8_t sf;
   uint8_t d0;
   uint8_t d1;
   uint8_t d2;
   uint8_t d3;
   uint8_t d4;
   uint8_t d5;
   uint8_t d6;
   uint8_t d7;
   uint8_t crc;
   uint8_t len;
   uint8_t received;       //13
};

struct tmgw_struc2 {
    struct tmgw_struc rx;
    struct tmgw_struc tx;
    struct tmgw_struc test;
};

struct tmgw_struc2 tmgw;

int txcnt2 = 0;

__IO uint32_t uwCRCValue = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void processTMGWMessage(UART_HandleTypeDef* huart, uint8_t* ptr, int len)
{
    UNUSED(len);
    UNUSED(huart);
    UNUSED(ptr);
    printf("asdf\r\n");
}

void HAL_UART_RxIdleCallback(UART_HandleTypeDef* huart)
{
    BKPT;
    uint16_t rxXferCount = 0;

    //
    if( (huart->hdmarx != NULL) && (huart->Instance == USART1) )
    {

        HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);          //debug only

        __HAL_UART_CLEAR_IDLEFLAG(huart);
        __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);

//        HAL_UART_DMAStop(huart);
//        HAL_UART_Abort(huart);

        DMA_HandleTypeDef *hdma = huart->hdmarx;

        /* Determine how many items of data have been received */

        rxXferCount = huart->RxXferSize - __HAL_DMA_GET_COUNTER(hdma);
//        aRxBufferPos = rxXferCount;
        tmgw.rx.len = rxXferCount;

        huart->RxXferCount = 0;

        /* Check if a transmit process is ongoing or not */

        if(huart->gState == HAL_UART_STATE_BUSY_TX_RX)
        {
            huart->gState = HAL_UART_STATE_BUSY_TX;
        }
        else
        {
            huart->gState = HAL_UART_STATE_READY;
        }

//        printf("CF2:0x%02x %02d\r\n", tmgw.rx.cf, tmgw.rx.len);

        tmgw.rx.received = 1;

        HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);          //debug only

        return;
    }

    return;
}

void startReceive()
{
    memset(&tmgw.rx, 0, sizeof(tmgw.rx));
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)&tmgw.rx, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);          //debug only
//        printf("R2%02x\n", tmgw.rx.cf);
        HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);          //debug only
        startReceive();
        tmgw.rx.received = 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* Set transmission flag: transfer complete */
        Uart1Ready = SET;
        HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
    }
    txcnt2++;
}


void configure_tracing()
{
    /* STM32 specific configuration to enable the TRACESWO IO pin */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->MAPR |= (2 << 24); // Disable JTAG to release TRACESWO
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins

    if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
    {
        // Some (all?) STM32s don't allow writes to DBGMCU register until
        // C_DEBUGEN in CoreDebug->DHCSR is set. This cannot be set by the
        // CPU itself, so in practice you need to connect to the CPU with
        // a debugger once before resetting it.
        return;
    }

    /* Configure Trace Port Interface Unit */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable access to registers
    TPI->ACPR = 0; // Trace clock = HCLK/(x+1) = 8MHz = UART 's baudrate
                   // The HCLK of F105 is 8MHz so x is 0, and the F103 is 72MHz so x is 8
    TPI->SPPR = 2; // Pin protocol = NRZ/USART
    TPI->FFCR = 0x102; // TPIU packet framing enabled when bit 2 is set.
                       // You can use 0x100 if you only need DWT/ITM and not ETM.

    /* Configure PC sampling and exception trace  */
    DWT->CTRL = (1 << DWT_CTRL_CYCTAP_Pos) // Prescaler for PC sampling
                                           // 0 = x32, 1 = x512
              | (0 << DWT_CTRL_POSTPRESET_Pos) // Postscaler for PC sampling
                                               // Divider = value + 1
              | (1 << DWT_CTRL_PCSAMPLENA_Pos) // Enable PC sampling
              | (2 << DWT_CTRL_SYNCTAP_Pos)    // Sync packet interval
                                               // 0 = Off, 1 = Every 2^23 cycles,
                                               // 2 = Every 2^25, 3 = Every 2^27
              | (1 << DWT_CTRL_EXCTRCENA_Pos)  // Enable exception trace
              | (1 << DWT_CTRL_CYCCNTENA_Pos); // Enable cycle counter

    /* Configure instrumentation trace macroblock */
    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = (1 << ITM_TCR_TraceBusID_Pos) // Trace bus ID for TPIU
             | (1 << ITM_TCR_DWTENA_Pos)     // Enable events from DWT
             | (1 << ITM_TCR_SYNCENA_Pos)    // Enable sync packets
             | (1 << ITM_TCR_ITMENA_Pos);    // Main enable for ITM
    ITM->TER = 0xFFFFFFFF; // Enable all stimulus ports
}

void ITM_Init(void)
{
    /* STM32 specific configuration to enable the TRACESWO IO pin */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->MAPR |= (2 << 24); // Disable JTAG to release TRACESWO
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN; // Enable IO trace pins

    TPI->ACPR = 31;  // Output bits at 72000000/(31+1)=2.25MHz.
    TPI->SPPR = 2;   // Use Async mode (1 for RZ/Manchester)
    TPI->FFCR  = 0;   // Disable formatter

    /* Configure instrumentation trace macroblock */
    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = 1 << ITM_TCR_TraceBusID_Pos | ITM_TCR_SYNCENA_Msk |
               ITM_TCR_ITMENA_Msk;
    ITM->TER = 0xFFFFFFFF; // Enable all stimulus ports
}

//
static uint8_t crcCalc(uint8_t* Data, uint8_t Size)
{
  uint8_t j;
  uint8_t Carry;
  uint8_t Crc;

  Crc = 0;

  while (Size-- > 0)
  {
    Crc ^= *Data++;
    for (j = 8; j != 0; j--)
    {
      Carry = Crc & 0x80;
      Crc <<= 1;
      if (Carry != 0)
      {
        Crc ^= 0x01;  /* Polynome X^8 + 1  */
      }
    }
  }
  return (Crc & 0x00FF);
}

//printf on ITM
int __write(int file, char *ptr, int len)
{
    UNUSED(file);
  /* Implement your write code here, this is used by puts and printf for example */
  for(int i=0 ; i<len ; i++) {
//      BKPT;
    ITM_SendChar((*ptr++));
  }
  return len;
}

/* Function to reverse bits of num */
uint8_t reverseBits(uint8_t num)
{
    unsigned int  NO_OF_BITS = sizeof(num) * 8;
    unsigned int reverse_num = 0, i, temp;

    for (i = 0; i < NO_OF_BITS; i++)
    {
        temp = (num & (1 << i));
        if(temp)
            reverse_num |= (1 << ((NO_OF_BITS - 1) - i));
    }

    return reverse_num;
}

void sendmbcmd(uint8_t cmdno)
{
    //check for readiness for transmission
    if ( (huart1.gState == HAL_UART_STATE_READY) )
//    if ((huart1.gState == HAL_UART_STATE_READY) || (huart1.gState == HAL_UART_STATE_BUSY_RX)) //glupavo. na rs485 sme i se machkame sami po vreme na priemane
    {
        tmgw.rx.received = 0;

        tmgw.tx.cf = 0b01010010;
        tmgw.tx.sf = 0x0c ;
        tmgw.tx.d0 = 0x17;

    //    tmgw.tx.d1 = 0x40;
    //    tmgw.tx.d2 = 0x88;

        tmgw.tx.len = 4;            //+1 for crc

        tmgw.test.len = tmgw.tx.len;
    //    tmgw.test.cf = reverseBits(tmgw.tx.cf);
    //    tmgw.test.sf = reverseBits(tmgw.tx.sf);
    //    tmgw.test.d0 = reverseBits(tmgw.tx.d0);
    //    tmgw.test.d1 = reverseBits(tmgw.tx.d1);
    //    tmgw.test.d2 = reverseBits(tmgw.tx.d2);

        tmgw.test.cf = (tmgw.tx.cf);
        tmgw.test.sf = (tmgw.tx.sf);
        tmgw.test.d0 = (tmgw.tx.d0);
        tmgw.test.d1 = (tmgw.tx.d1);
        tmgw.test.d2 = (tmgw.tx.d2);


    //    //FIXED CRC CALCS.... FIXME
    //    uwCRCValue = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&tmgw.tx, tmgw.tx.len);
    //    tmgw.tx.d3 = crc8x_fast(0xff, &tmgw.test, tmgw.test.len-1);
    //    tmgw.tx.d3  = reverseBits(0x1d);
        tmgw.tx.d1 = crcCalc(&tmgw.test.cf, tmgw.test.len-1);

        HAL_StatusTypeDef stat = HAL_ERROR;
        HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);

        stat = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&tmgw.tx, tmgw.tx.len);         // W/O

        if(stat != HAL_OK)
        {
          Error_Handler();
        }

        /*## Wait for the end of the transfer ################################### zashtooo ?*/
        printf("CRC1:0x%04x", tmgw.tx.d1);
        while (Uart1Ready != SET)
        {
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

//    HAL_DBGMCU_EnableDBGStopMode();
//    HAL_DBGMCU_DisableDBGStopMode();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  ITM_Init();
//  configure_tracing();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

//  HAL_Delay(200);

  printf("BOOT\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int cnt = 0;
  int txcnt = 0;

  while (1)
  {
      //tuka ako niamame zabaviane se uebava... da vidim kyde tochno
      if ( (cnt % 130067) == 1) {
          printf("%04d\n", cnt);
      }
      cnt++;
//      printf("DE:%04x\n", tmgw.rx.received);
//      printf("DF:%04x\n", tmgw.rx.cf);
//      BKPT;
//      if ( (tmgw.rx.received == 1) && (tmgw.rx.cf == 0x52) ) {
//          sendmbcmd(0);
////          printf("TX:%04d %04d\r\n", txcnt++, txcnt2);
//      }
      //poluchavame received na nulichki
//      if ( (tmgw.rx.received == 1) && (tmgw.rx.cf != 0x52) ) {
//          printf("RX:%02x\n", tmgw.rx.cf);
//      }
      startReceive();
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 95;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2500000;
//  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

//  //Activate RX Idle callback.
//  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
//  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  //Activate UART DMA for receive
  startReceive();
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DE_Pin */
  GPIO_InitStruct.Pin = DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

    BKPT;

//  printf("HAL ERROR!!!");

  while (1) {
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    HAL_Delay(100);
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

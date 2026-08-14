/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm45686_spi.h"      /* ICM_BusesInit, ICM_InitAllSensors, ICM_StartBurstRead, ICM_ParseAllFIFO */
#include "icm45686_data.h"     /* g_fifo_batch_ready, структуры данных датчиков */
#include "uart_telemetry.h"    /* UART_Telemetry_Init, UART_SendBatch */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_BDMA_Init(void);
static void MX_USART1_UART_Init(void);

static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);

static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_SPI6_Init(void);



/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //LL_GPIO_ResetOutputPin(GPIOG, LL_GPIO_PIN_5); // Выключение внешнего генератора!

  MX_DMA_Init();
  MX_BDMA_Init();   /* [NEW] обязательно — раньше была не нужна */

  MX_SPI1_Init();
  MX_SPI5_Init();
  MX_SPI4_Init();

  MX_SPI2_Init();   /* [NEW] раскомментировать */
  MX_SPI3_Init();   /* [NEW] раскомментировать */
  MX_SPI6_Init();   /* [NEW] раскомментировать */

  MX_TIM6_Init();
  MX_TIM7_Init();

  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  ICM_BusesInit();

  uint32_t imu_faults = ICM_InitAllSensors();
  (void)imu_faults;  /* debug: breakpoint здесь для проверки fault-маски */


  /* Инициализация UART-телеметрии (DMA TX) */
  UART_Telemetry_Init();

  /* Запуск TIM6 — генерирует прерывание каждые ~3.125 мс (10 пакетов @ 3200 Гц) */
  LL_TIM_EnableCounter(TIM6);
  LL_TIM_EnableIT_UPDATE(TIM6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* [REWRITE v2] Атомарное потребление event-битов вместо чтения
     * g_fifo_batch_ready напрямую. ICM_ConsumeEvents() забирает и
     * очищает весь bitmap за одну exclusive-access операцию (LDREX/STREX),
     * поэтому main loop никогда не видит частично обновлённое состояние,
     * даже если ISR трёх разных шин выставляют события почти одновременно. */
    uint32_t events = ICM_ConsumeEvents();

    if ((events & ICM_EVT_BATCH_READY) != 0U)
    {
      /*
       * Парсер должен брать полезные FIFO байты с [1]:
       * g_fifo_data[bus][imu][1].
       * g_fifo_data[bus][imu][0] — ответ на командный байт FIFO_DATA.
       */
      ICM_ParseAllFIFO();

      UART_BuildAndSendSyncFrame();
      //UART_SendBatch();
    }

    if ((events & (ICM_EVT_BUS0_FAULT | ICM_EVT_BUS1_FAULT | ICM_EVT_BUS2_FAULT)) != 0U)
    {
      /* Опционально: инкремент диагностических счётчиков / телеметрия faults.
       * Само восстановление шины уже выполнено в ICM_RecoverBus() внутри ISR. */
    }

    if ((events & ICM_EVT_DMA_TIMEOUT) != 0U)
    {
      /* Опционально: телеметрия DMA timeout. Recovery уже произошло в ISR. */
    }
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
  /* [FIX] STM32H723 @ 550 MHz, VOS0: требуется FLASH_LATENCY_4 (4 wait states),
   * а не LATENCY_3. LATENCY_3 достаточно только до ~450 МГц при VOS0 —
   * см. RM0468 Table "FLASH recommended number of wait states". Работа
   * с недостаточным latency формально вне timing margin и может приводить
   * к redxbit corruption под температурной/voltage нагрузкой. */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1R_Enable();
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL1_SetM(1);
  LL_RCC_PLL1_SetN(34);
  LL_RCC_PLL1_SetP(1);
  LL_RCC_PLL1_SetQ(2);
  LL_RCC_PLL1_SetR(2);
  LL_RCC_PLL1_SetFRACN(3072);
  LL_RCC_PLL1FRACN_Enable();
  LL_RCC_PLL1_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL1_IsReady() != 1)
  {
  }

   /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
   LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
  {

  }
  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

  LL_Init1msTick(550000000);

  LL_SetSystemCoreClock(550000000);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  LL_RCC_PLL2P_Enable();
  LL_RCC_PLL2Q_Enable();
  LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL2_SetM(1);
  LL_RCC_PLL2_SetN(12);
  LL_RCC_PLL2_SetP(5);
  LL_RCC_PLL2_SetQ(5);
  LL_RCC_PLL2_SetR(2);
  LL_RCC_PLL2_SetFRACN(4096);
  LL_RCC_PLL2FRACN_Enable();
  LL_RCC_PLL2_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL2_IsReady() != 1)
  {
  }

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL2P);

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI1 DMA Init */

  /* SPI1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_2, LL_DMAMUX1_REQ_SPI1_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_2, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_2);

  /* SPI1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_3, LL_DMAMUX1_REQ_SPI1_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_3, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_3);

  /* SPI1 interrupt Init */
  NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(SPI1_IRQn);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity   = LL_SPI_POLARITY_LOW;    // CPOL=0
  SPI_InitStruct.ClockPhase      = LL_SPI_PHASE_1EDGE;     // CPHA=0
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Disable(SPI1);
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI1, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL2P);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PC1   ------> SPI2_MOSI
  PC2_C   ------> SPI2_MISO
  PB10   ------> SPI2_SCK
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI2 DMA Init */

  /* SPI2_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_4, LL_DMAMUX1_REQ_SPI2_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_4, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_4, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_4);

  /* SPI2_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_5, LL_DMAMUX1_REQ_SPI2_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

  /* SPI2 interrupt Init */
  NVIC_SetPriority(SPI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(SPI2_IRQn);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity   = LL_SPI_POLARITY_LOW;    // CPOL=0
  SPI_InitStruct.ClockPhase      = LL_SPI_PHASE_1EDGE;     // CPHA=0
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI2, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI2);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL2P);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  /**SPI3 GPIO Configuration
  PB2   ------> SPI3_MOSI
  PC10   ------> SPI3_SCK
  PC11   ------> SPI3_MISO
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SPI3 DMA Init */

  /* SPI3_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_6, LL_DMAMUX1_REQ_SPI3_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);

  /* SPI3_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_7, LL_DMAMUX1_REQ_SPI3_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_7, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_7);

  /* SPI3 interrupt Init */
  NVIC_SetPriority(SPI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(SPI3_IRQn);

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity   = LL_SPI_POLARITY_LOW;    // CPOL=0
  SPI_InitStruct.ClockPhase      = LL_SPI_PHASE_1EDGE;     // CPHA=0
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Init(SPI3, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI3, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI3);
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI45_CLKSOURCE_PLL2Q);

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
  /**SPI4 GPIO Configuration
  PE2   ------> SPI4_SCK
  PE5   ------> SPI4_MISO
  PE6   ------> SPI4_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_5|LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* SPI4 DMA Init */

  /* SPI4_RX Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_SPI4_RX);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

  /* SPI4_TX Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_SPI4_TX);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_1, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_1);

  /* SPI4 interrupt Init */
  NVIC_SetPriority(SPI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(SPI4_IRQn);

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  LL_SPI_Disable(SPI4);
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity   = LL_SPI_POLARITY_LOW;    // CPOL=0
  SPI_InitStruct.ClockPhase      = LL_SPI_PHASE_1EDGE;     // CPHA=0
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Init(SPI4, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI4, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI4, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI4);
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI45_CLKSOURCE_PLL2Q);

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI5);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF);
  /**SPI5 GPIO Configuration
  PF7   ------> SPI5_SCK
  PF8   ------> SPI5_MISO
  PF9   ------> SPI5_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* SPI5 DMA Init */

  /* SPI5_RX Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_2, LL_DMAMUX1_REQ_SPI5_RX);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);

  /* SPI5_TX Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_3, LL_DMAMUX1_REQ_SPI5_TX);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_3, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_3);

  /* SPI5 interrupt Init */
  NVIC_SetPriority(SPI5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(SPI5_IRQn);

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity   = LL_SPI_POLARITY_LOW;    // CPOL=0
  SPI_InitStruct.ClockPhase      = LL_SPI_PHASE_1EDGE;     // CPHA=0
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Disable(SPI5);
  LL_SPI_Init(SPI5, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI5, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI5, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI5);
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief SPI6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI6_CLKSOURCE_PLL2Q);

  /* Peripheral clock enable */
  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SPI6);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);
  /**SPI6 GPIO Configuration
  PC12   ------> SPI6_SCK
  PG12   ------> SPI6_MISO
  PG14   ------> SPI6_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* SPI6 DMA Init */

  /* SPI6_RX Init */
  LL_BDMA_SetPeriphRequest(BDMA, LL_BDMA_CHANNEL_0, LL_DMAMUX2_REQ_SPI6_RX);

  LL_BDMA_SetDataTransferDirection(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_BDMA_SetChannelPriorityLevel(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PRIORITY_VERYHIGH);

  LL_BDMA_SetMode(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MODE_NORMAL);

  LL_BDMA_SetPeriphIncMode(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PERIPH_NOINCREMENT);

  LL_BDMA_SetMemoryIncMode(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MEMORY_INCREMENT);

  LL_BDMA_SetPeriphSize(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PDATAALIGN_BYTE);

  LL_BDMA_SetMemorySize(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MDATAALIGN_BYTE);

  /* SPI6_TX Init */
  LL_BDMA_SetPeriphRequest(BDMA, LL_BDMA_CHANNEL_1, LL_DMAMUX2_REQ_SPI6_TX);

  LL_BDMA_SetDataTransferDirection(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_BDMA_SetChannelPriorityLevel(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PRIORITY_HIGH);

  LL_BDMA_SetMode(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MODE_NORMAL);

  LL_BDMA_SetPeriphIncMode(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PERIPH_NOINCREMENT);

  LL_BDMA_SetMemoryIncMode(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MEMORY_INCREMENT);

  LL_BDMA_SetPeriphSize(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PDATAALIGN_BYTE);

  LL_BDMA_SetMemorySize(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MDATAALIGN_BYTE);

  /* SPI6 interrupt Init */
  NVIC_SetPriority(SPI6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(SPI6_IRQn);

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  /* SPI6 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity   = LL_SPI_POLARITY_LOW;    // CPOL=0
  SPI_InitStruct.ClockPhase      = LL_SPI_PHASE_1EDGE;     // CPHA=0
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;
  LL_SPI_Init(SPI6, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI6, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI6, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI6);
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  /* TIM6 interrupt Init */
  NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  TIM_InitStruct.Prescaler = 274;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 9999;
  LL_TIM_Init(TIM6, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM6);
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  TIM_InitStruct.Prescaler = 274;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0xFFFF;
  LL_TIM_Init(TIM7, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM7);
  LL_TIM_SetTriggerOutput(TIM7, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM7);
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART16_CLKSOURCE_PCLK2);

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB14   ------> USART1_TX
  PB15   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 DMA Init */

  /* USART1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_USART1_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);

  /* USART1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_USART1_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 12000000;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_8;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_BDMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_BDMA);

  /* DMA interrupt init */
  /* BDMA_Channel0_IRQn interrupt configuration */
  NVIC_SetPriority(BDMA_Channel0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(BDMA_Channel0_IRQn);
  /* BDMA_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(BDMA_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(BDMA_Channel1_IRQn);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* =========================================================
     * Тактирование всех используемых портов GPIO
     * ========================================================= */
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOH);

    /* =========================================================
     * Все CS-пины в HIGH ДО конфигурации режима —
     * исключаем случайный LOW во время Init.
     *
     * НИЖНЯЯ ПЛАТА (SPI1, SPI5, SPI4 — датчики 1–18):
     *   SPI1: PB12 PB13 PE8  PE9  PF13 PF14
     *   SPI5: PE14 PE15 PE7  PG1  PB0  PB1
     *   SPI4: PE10 PE11 PF15 PG0  PC4  PC5
     *
     * ВЕРХНЯЯ ПЛАТА (SPI3, SPI2, SPI6 — датчики 19–36):
     *   SPI3: PD0  PD1  PD6  PD7  PB8  PE0
     *   SPI2: PA9  PA10 PD4  PD5  PB3  PB4
     *   SPI6: PA8  PC9  PD2  PD3  PG9  PG15
     * ========================================================= */

    /* GPIOA: PA9(CS7/SPI2) PA10(CS8/SPI2) PA8(CS13/SPI6) */
    LL_GPIO_SetOutputPin(GPIOA,
        LL_GPIO_PIN_8  |   /* CS13 — датчик 31, SPI6 */
        LL_GPIO_PIN_9  |   /* CS7  — датчик 25, SPI2 */
        LL_GPIO_PIN_10);   /* CS8  — датчик 26, SPI2 */

    /* GPIOB: PB12(CS1/SPI1) PB13(CS2/SPI1) PB0(CS11/SPI5) PB1(CS12/SPI5)
     *        PB8(CS5/SPI3)  PB3(CS11/SPI2) PB4(CS12/SPI2) */
    LL_GPIO_SetOutputPin(GPIOB,
        LL_GPIO_PIN_0  |   /* CS11 — датчик 11, SPI5 */
        LL_GPIO_PIN_1  |   /* CS12 — датчик 12, SPI5 */
        LL_GPIO_PIN_3  |   /* CS11 — датчик 29, SPI2 */
        LL_GPIO_PIN_4  |   /* CS12 — датчик 30, SPI2 */
        LL_GPIO_PIN_8  |   /* CS5  — датчик 23, SPI3 */
        LL_GPIO_PIN_12 |   /* CS1  — датчик 1,  SPI1 */
        LL_GPIO_PIN_13);   /* CS2  — датчик 2,  SPI1 */

    /* GPIOC: PC4(CS17/SPI4) PC5(CS18/SPI4) PC9(CS14/SPI6) */
    LL_GPIO_SetOutputPin(GPIOC,
        LL_GPIO_PIN_4  |   /* CS17 — датчик 17, SPI4 */
        LL_GPIO_PIN_5  |   /* CS18 — датчик 18, SPI4 */
        LL_GPIO_PIN_9);    /* CS14 — датчик 32, SPI6 */

    /* GPIOD: PD0(CS1/SPI3) PD1(CS2/SPI3) PD6(CS3/SPI3) PD7(CS4/SPI3)
     *        PD4(CS9/SPI2) PD5(CS10/SPI2) PD2(CS15/SPI6) PD3(CS16/SPI6) */
    LL_GPIO_SetOutputPin(GPIOD,
        LL_GPIO_PIN_0  |   /* CS1  — датчик 19, SPI3 */
        LL_GPIO_PIN_1  |   /* CS2  — датчик 20, SPI3 */
        LL_GPIO_PIN_2  |   /* CS15 — датчик 33, SPI6 */
        LL_GPIO_PIN_3  |   /* CS16 — датчик 34, SPI6 */
        LL_GPIO_PIN_4  |   /* CS9  — датчик 27, SPI2 */
        LL_GPIO_PIN_5  |   /* CS10 — датчик 28, SPI2 */
        LL_GPIO_PIN_6  |   /* CS3  — датчик 21, SPI3 */
        LL_GPIO_PIN_7);    /* CS4  — датчик 22, SPI3 */

    /* GPIOE: PE8(CS3/SPI1) PE9(CS4/SPI1) PE14(CS7/SPI5) PE15(CS8/SPI5)
     *        PE7(CS9/SPI5) PE10(CS13/SPI4) PE11(CS14/SPI4) PE0(CS6/SPI3) */
    LL_GPIO_SetOutputPin(GPIOE,
        LL_GPIO_PIN_0  |   /* CS6  — датчик 24, SPI3 */
        LL_GPIO_PIN_7  |   /* CS9  — датчик 9,  SPI5 */
        LL_GPIO_PIN_8  |   /* CS3  — датчик 3,  SPI1 */
        LL_GPIO_PIN_9  |   /* CS4  — датчик 4,  SPI1 */
        LL_GPIO_PIN_10 |   /* CS13 — датчик 13, SPI4 */
        LL_GPIO_PIN_11 |   /* CS14 — датчик 14, SPI4 */
        LL_GPIO_PIN_14 |   /* CS7  — датчик 7,  SPI5 */
        LL_GPIO_PIN_15);   /* CS8  — датчик 8,  SPI5 */

    /* GPIOF: PF13(CS5/SPI1) PF14(CS6/SPI1) PF15(CS15/SPI4) */
    LL_GPIO_SetOutputPin(GPIOF,
        LL_GPIO_PIN_13 |   /* CS5  — датчик 5,  SPI1 */
        LL_GPIO_PIN_14 |   /* CS6  — датчик 6,  SPI1 */
        LL_GPIO_PIN_15);   /* CS15 — датчик 15, SPI4 */

    /* GPIOG: PG0(CS16/SPI4) PG1(CS10/SPI5) PG9(CS17/SPI6) PG15(CS18/SPI6) */
    LL_GPIO_SetOutputPin(GPIOG,
        LL_GPIO_PIN_0  |   /* CS16 — датчик 16, SPI4 */
        LL_GPIO_PIN_1  |   /* CS10 — датчик 10, SPI5 */
        LL_GPIO_PIN_5  |   /* генератор ON */
        LL_GPIO_PIN_9  |   /* CS17 — датчик 35, SPI6 */
        LL_GPIO_PIN_15);   /* CS18 — датчик 36, SPI6 */

    /* =========================================================
     * PE3 — аналоговый вход. НЕ ТРОГАТЬ — КЗ НА ПЛАТЕ!
     * ========================================================= */
    GPIO_InitStruct.Pin        = LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* =========================================================
     * GPIOA: PA8(CS13/SPI6) PA9(CS7/SPI2) PA10(CS8/SPI2)
     * ========================================================= */
    GPIO_InitStruct.Pin        = LL_GPIO_PIN_8  |   /* CS13 — датчик 31, SPI6 */
                                  LL_GPIO_PIN_9  |   /* CS7  — датчик 25, SPI2 */
                                  LL_GPIO_PIN_10;    /* CS8  — датчик 26, SPI2 */
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* =========================================================
     * GPIOB: PB0(CS11/SPI5) PB1(CS12/SPI5)
     *        PB3(CS11/SPI2)  PB4(CS12/SPI2)
     *        PB8(CS5/SPI3)
     *        PB12(CS1/SPI1)  PB13(CS2/SPI1)
     * ========================================================= */
    GPIO_InitStruct.Pin        = LL_GPIO_PIN_0  |   /* CS11 — датчик 11, SPI5 */
                                  LL_GPIO_PIN_1  |   /* CS12 — датчик 12, SPI5 */
                                  LL_GPIO_PIN_3  |   /* CS11 — датчик 29, SPI2 */
                                  LL_GPIO_PIN_4  |   /* CS12 — датчик 30, SPI2 */
                                  LL_GPIO_PIN_8  |   /* CS5  — датчик 23, SPI3 */
                                  LL_GPIO_PIN_12 |   /* CS1  — датчик 1,  SPI1 */
                                  LL_GPIO_PIN_13;    /* CS2  — датчик 2,  SPI1 */
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* =========================================================
     * GPIOC: PC4(CS17/SPI4) PC5(CS18/SPI4) PC9(CS14/SPI6)
     * ========================================================= */
    GPIO_InitStruct.Pin        = LL_GPIO_PIN_4  |   /* CS17 — датчик 17, SPI4 */
                                  LL_GPIO_PIN_5  |   /* CS18 — датчик 18, SPI4 */
                                  LL_GPIO_PIN_9;     /* CS14 — датчик 32, SPI6 */
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* =========================================================
     * GPIOD: PD0(CS1/SPI3)  PD1(CS2/SPI3)
     *        PD2(CS15/SPI6) PD3(CS16/SPI6)
     *        PD4(CS9/SPI2)  PD5(CS10/SPI2)
     *        PD6(CS3/SPI3)  PD7(CS4/SPI3)
     * ========================================================= */
    GPIO_InitStruct.Pin        = LL_GPIO_PIN_0  |   /* CS1  — датчик 19, SPI3 */
                                  LL_GPIO_PIN_1  |   /* CS2  — датчик 20, SPI3 */
                                  LL_GPIO_PIN_2  |   /* CS15 — датчик 33, SPI6 */
                                  LL_GPIO_PIN_3  |   /* CS16 — датчик 34, SPI6 */
                                  LL_GPIO_PIN_4  |   /* CS9  — датчик 27, SPI2 */
                                  LL_GPIO_PIN_5  |   /* CS10 — датчик 28, SPI2 */
                                  LL_GPIO_PIN_6  |   /* CS3  — датчик 21, SPI3 */
                                  LL_GPIO_PIN_7;     /* CS4  — датчик 22, SPI3 */
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* =========================================================
     * GPIOE: PE0(CS6/SPI3)
     *        PE7(CS9/SPI5)   PE8(CS3/SPI1)   PE9(CS4/SPI1)
     *        PE10(CS13/SPI4) PE11(CS14/SPI4)
     *        PE14(CS7/SPI5)  PE15(CS8/SPI5)
     * Примечание: PE3 — аналоговый, уже настроен выше, НЕ включать сюда!
     * ========================================================= */
    GPIO_InitStruct.Pin        = LL_GPIO_PIN_0  |   /* CS6  — датчик 24, SPI3 */
                                  LL_GPIO_PIN_7  |   /* CS9  — датчик 9,  SPI5 */
                                  LL_GPIO_PIN_8  |   /* CS3  — датчик 3,  SPI1 */
                                  LL_GPIO_PIN_9  |   /* CS4  — датчик 4,  SPI1 */
                                  LL_GPIO_PIN_10 |   /* CS13 — датчик 13, SPI4 */
                                  LL_GPIO_PIN_11 |   /* CS14 — датчик 14, SPI4 */
                                  LL_GPIO_PIN_14 |   /* CS7  — датчик 7,  SPI5 */
                                  LL_GPIO_PIN_15;    /* CS8  — датчик 8,  SPI5 */
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* =========================================================
     * GPIOF: PF13(CS5/SPI1) PF14(CS6/SPI1) PF15(CS15/SPI4)
     * ========================================================= */
    GPIO_InitStruct.Pin        = LL_GPIO_PIN_13 |   /* CS5  — датчик 5,  SPI1 */
                                  LL_GPIO_PIN_14 |   /* CS6  — датчик 6,  SPI1 */
                                  LL_GPIO_PIN_15;    /* CS15 — датчик 15, SPI4 */
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* =========================================================
     * GPIOG: PG0(CS16/SPI4) PG1(CS10/SPI5)
     *        PG5 (генератор) PG9(CS17/SPI6) PG15(CS18/SPI6)
     * ========================================================= */
    LL_GPIO_SetOutputPin(GPIOG, LL_GPIO_PIN_5); /* PG5 HIGH = генератор ON */

    GPIO_InitStruct.Pin        = LL_GPIO_PIN_0  |   /* CS16 — датчик 16, SPI4 */
                                  LL_GPIO_PIN_1  |   /* CS10 — датчик 10, SPI5 */
                                  LL_GPIO_PIN_5  |   /* генератор CLK EN      */
                                  LL_GPIO_PIN_9  |   /* CS17 — датчик 35, SPI6 */
                                  LL_GPIO_PIN_15;    /* CS18 — датчик 36, SPI6 */
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
    LL_MPU_Disable();

    /* =========================================================
     * Region 0: D2 SRAM (0x30000000, 256KB)
     *   Используется DMA1, DMA2 (шины SPI1..SPI5, USART1).
     *   Non-Cacheable + Shareable — обязательно для DMA-буферов
     *   на Cortex-M7 с включённым D-cache.
     *   ACCESS_SHAREABLE обеспечивает cache coherency между CPU и DMA.
     * ========================================================= */
    LL_MPU_ConfigRegion(LL_MPU_REGION_NUMBER0,
                        0x00U,
                        0x30000000U,
                        LL_MPU_REGION_SIZE_256KB     |
                        LL_MPU_REGION_PRIV_RW        |
                        LL_MPU_ACCESS_NOT_BUFFERABLE |
                        LL_MPU_ACCESS_NOT_CACHEABLE  |
                        LL_MPU_ACCESS_SHAREABLE      |  /* <-- ИСПРАВЛЕНО */
                        LL_MPU_INSTRUCTION_ACCESS_DISABLE);
    LL_MPU_EnableRegion(LL_MPU_REGION_NUMBER0);

    /* =========================================================
     * Region 1: D3 SRAM (0x38000000, 32KB)
     *   Используется BDMA (только D3-домен!) для SPI6 RX/TX.
     *   g_fifo_data_spi6[] ДОЛЖЕН быть размещён здесь через
     *   __attribute__((section(".sram4"))) или аналог в ld-скрипте.
     *   Non-Cacheable + Shareable — те же требования что и Region 0.
     * ========================================================= */
    LL_MPU_ConfigRegion(LL_MPU_REGION_NUMBER1,
                        0x00U,
                        0x38000000U,
                        LL_MPU_REGION_SIZE_32KB      |
                        LL_MPU_REGION_PRIV_RW        |
                        LL_MPU_ACCESS_NOT_BUFFERABLE |
                        LL_MPU_ACCESS_NOT_CACHEABLE  |
                        LL_MPU_ACCESS_SHAREABLE      |
                        LL_MPU_INSTRUCTION_ACCESS_DISABLE);
    LL_MPU_EnableRegion(LL_MPU_REGION_NUMBER1);

    LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  (void)file; (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

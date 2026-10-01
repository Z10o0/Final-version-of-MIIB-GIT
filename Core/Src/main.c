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
#include "icm45686_spi.h"
#include "icm45686_data.h"
#include "uart_telemetry.h"
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
  uint64_t imu_faults;
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and SysTick. */
  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);

  /* System interrupt init */
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

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

  /*
   * LL_GPIO_ResetOutputPin(GPIOG, LL_GPIO_PIN_5);
   * Выключение внешнего генератора.
   */

  MX_DMA_Init();
  MX_BDMA_Init();

  MX_SPI1_Init();
  MX_SPI5_Init();
  MX_SPI4_Init();

  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI6_Init();

  MX_TIM6_Init();
  MX_TIM7_Init();

  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  ICM_BusesInit();

  imu_faults = ICM_InitAllSensors();
  (void)imu_faults;

  /* Инициализация UART-телеметрии и DMA TX ring buffer. */
  UART_Telemetry_Init();

  /*
   * TIM7 — независимый watchdog FSM, 1 кГц.
   * Он обнаруживает зависшие DMA/EOT-состояния и не задаёт output rate.
   */
  LL_TIM_SetCounter(TIM7, 0U);
  LL_TIM_ClearFlag_UPDATE(TIM7);
  LL_TIM_EnableIT_UPDATE(TIM7);
  LL_TIM_EnableCounter(TIM7);

  /*
   * TIM6 — acquisition trigger:
   * 2,5 мс, 8 FIFO-пакетов при ODR 3200 Гц,
   * один output frame 400 Гц.
   */
  LL_TIM_SetCounter(TIM6, 0U);
  LL_TIM_ClearFlag_UPDATE(TIM6);
  LL_TIM_EnableIT_UPDATE(TIM6);
  LL_TIM_EnableCounter(TIM6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t events = ICM_ConsumeEvents();

    if ((events & ICM_EVT_BATCH_READY) != 0U)
    {
      /*
       * DMA payload начинается с [1]:
       * [0] — ответ на командный байт FIFO_DATA;
       * [1..160] — восемь 20-байтовых HIRES-пакетов.
       */
      ICM_ParseAllFIFO();
      ICM_AverageAllBatches();
      UART_BuildAndSendSyncFrame();
    }

    if ((events &
         (ICM_EVT_BUS0_FAULT |
          ICM_EVT_BUS1_FAULT |
          ICM_EVT_BUS2_FAULT |
          ICM_EVT_BUS3_FAULT |
          ICM_EVT_BUS4_FAULT |
          ICM_EVT_BUS5_FAULT)) != 0U)
    {
      /*
       * Recovery шины уже выполнен FSM.
       * Здесь разрешена только неблокирующая диагностика.
       */
    }

    if ((events & ICM_EVT_DMA_TIMEOUT) != 0U)
    {
      /*
       * DMA timeout зарегистрирован watchdog.
       * Блокирующая FIFO-resync из ISR не выполняется.
       */
    }

    if ((events & ICM_EVT_FRAME_SKIP) != 0U)
    {
      /*
       * g_fifo_resync_required остаётся sticky diagnostic flag.
       * Полная ресинхронизация FIFO относится к отдельному патчу.
       */
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
  /*
   * STM32H723:
   * SYSCLK/Core clock = 550 МГц.
   * HCLK              = 275 МГц.
   * PCLK1             = 137,5 МГц.
   * TIM6/TIM7 clock   = 275 МГц, поскольку APB1 divider != 1.
   */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);

  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }

  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);

  while (LL_PWR_IsActiveFlag_VOS() == 0U)
  {
  }

  LL_RCC_HSE_Enable();

  while (LL_RCC_HSE_IsReady() != 1U)
  {
  }

  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1R_Enable();

  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);

  LL_RCC_PLL1_SetM(1U);
  LL_RCC_PLL1_SetN(34U);
  LL_RCC_PLL1_SetP(1U);
  LL_RCC_PLL1_SetQ(2U);
  LL_RCC_PLL1_SetR(2U);
  LL_RCC_PLL1_SetFRACN(3072U);
  LL_RCC_PLL1FRACN_Enable();
  LL_RCC_PLL1_Enable();

  while (LL_RCC_PLL1_IsReady() != 1U)
  {
  }

  /*
   * Промежуточный AHB divider нужен при переключении SYSCLK
   * на частоту выше 80 МГц.
   */
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

  while (LL_RCC_GetSysClkSource() !=
         LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
  {
  }

  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

  LL_Init1msTick(550000000U);
  LL_SetSystemCoreClock(550000000U);
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

  LL_RCC_PLL2_SetM(1U);
  LL_RCC_PLL2_SetN(12U);
  LL_RCC_PLL2_SetP(5U);
  LL_RCC_PLL2_SetQ(5U);
  LL_RCC_PLL2_SetR(2U);
  LL_RCC_PLL2_SetFRACN(4096U);
  LL_RCC_PLL2FRACN_Enable();
  LL_RCC_PLL2_Enable();

  while (LL_RCC_PLL2_IsReady() != 1U)
  {
  }
}

/**
  * @brief SPI1 Initialization Function
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL2P);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);

  /*
   * PA5  -> SPI1_SCK
   * PA6  -> SPI1_MISO
   * PA7  -> SPI1_MOSI
   */
  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_5 |
      LL_GPIO_PIN_6 |
      LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI1 RX: DMA1 Stream2 */
  LL_DMA_SetPeriphRequest(DMA1,
                          LL_DMA_STREAM_2,
                          LL_DMAMUX1_REQ_SPI1_RX);
  LL_DMA_SetDataTransferDirection(
      DMA1,
      LL_DMA_STREAM_2,
      LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(
      DMA1,
      LL_DMA_STREAM_2,
      LL_DMA_PRIORITY_VERYHIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(
      DMA1,
      LL_DMA_STREAM_2,
      LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(
      DMA1,
      LL_DMA_STREAM_2,
      LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(
      DMA1,
      LL_DMA_STREAM_2,
      LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(
      DMA1,
      LL_DMA_STREAM_2,
      LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_2);

  /* SPI1 TX: DMA1 Stream3 */
  LL_DMA_SetPeriphRequest(DMA1,
                          LL_DMA_STREAM_3,
                          LL_DMAMUX1_REQ_SPI1_TX);
  LL_DMA_SetDataTransferDirection(
      DMA1,
      LL_DMA_STREAM_3,
      LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetStreamPriorityLevel(
      DMA1,
      LL_DMA_STREAM_3,
      LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(
      DMA1,
      LL_DMA_STREAM_3,
      LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(
      DMA1,
      LL_DMA_STREAM_3,
      LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(
      DMA1,
      LL_DMA_STREAM_3,
      LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(
      DMA1,
      LL_DMA_STREAM_3,
      LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_3);

  NVIC_SetPriority(
      SPI1_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(SPI1_IRQn);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0U;

  LL_SPI_Disable(SPI1);
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI1, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
}

/**
  * @brief SPI2 Initialization Function
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL2P);

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);

  LL_SYSCFG_CloseAnalogSwitch(LL_SYSCFG_ANALOG_SWITCH_PC2);

  /*
   * PC1  -> SPI2_MOSI
   * PC2_C -> SPI2_MISO
   * PB10 -> SPI2_SCK
   */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_2;
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

  /* SPI2 RX: DMA1 Stream4 */
  LL_DMA_SetPeriphRequest(DMA1,
                          LL_DMA_STREAM_4,
                          LL_DMAMUX1_REQ_SPI2_RX);
  LL_DMA_SetDataTransferDirection(
      DMA1,
      LL_DMA_STREAM_4,
      LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(
      DMA1,
      LL_DMA_STREAM_4,
      LL_DMA_PRIORITY_VERYHIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(
      DMA1,
      LL_DMA_STREAM_4,
      LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(
      DMA1,
      LL_DMA_STREAM_4,
      LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(
      DMA1,
      LL_DMA_STREAM_4,
      LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(
      DMA1,
      LL_DMA_STREAM_4,
      LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_4);

  /* SPI2 TX: DMA1 Stream5 */
  LL_DMA_SetPeriphRequest(DMA1,
                          LL_DMA_STREAM_5,
                          LL_DMAMUX1_REQ_SPI2_TX);
  LL_DMA_SetDataTransferDirection(
      DMA1,
      LL_DMA_STREAM_5,
      LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetStreamPriorityLevel(
      DMA1,
      LL_DMA_STREAM_5,
      LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(
      DMA1,
      LL_DMA_STREAM_5,
      LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(
      DMA1,
      LL_DMA_STREAM_5,
      LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(
      DMA1,
      LL_DMA_STREAM_5,
      LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(
      DMA1,
      LL_DMA_STREAM_5,
      LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

  NVIC_SetPriority(
      SPI2_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(SPI2_IRQn);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0U;

  LL_SPI_Disable(SPI2);
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI2, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI2);
}

/**
  * @brief SPI3 Initialization Function
  * @retval None
  */
static void MX_SPI3_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL2P);

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);

  /*
   * PB2  -> SPI3_MOSI
   * PC10 -> SPI3_SCK
   * PC11 -> SPI3_MISO
   */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SPI3 RX: DMA1 Stream6 */
  LL_DMA_SetPeriphRequest(DMA1,
                          LL_DMA_STREAM_6,
                          LL_DMAMUX1_REQ_SPI3_RX);
  LL_DMA_SetDataTransferDirection(
      DMA1,
      LL_DMA_STREAM_6,
      LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(
      DMA1,
      LL_DMA_STREAM_6,
      LL_DMA_PRIORITY_VERYHIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(
      DMA1,
      LL_DMA_STREAM_6,
      LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(
      DMA1,
      LL_DMA_STREAM_6,
      LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(
      DMA1,
      LL_DMA_STREAM_6,
      LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(
      DMA1,
      LL_DMA_STREAM_6,
      LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);

  /* SPI3 TX: DMA1 Stream7 */
  LL_DMA_SetPeriphRequest(DMA1,
                          LL_DMA_STREAM_7,
                          LL_DMAMUX1_REQ_SPI3_TX);
  LL_DMA_SetDataTransferDirection(
      DMA1,
      LL_DMA_STREAM_7,
      LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetStreamPriorityLevel(
      DMA1,
      LL_DMA_STREAM_7,
      LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(
      DMA1,
      LL_DMA_STREAM_7,
      LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(
      DMA1,
      LL_DMA_STREAM_7,
      LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(
      DMA1,
      LL_DMA_STREAM_7,
      LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(
      DMA1,
      LL_DMA_STREAM_7,
      LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_7);

  NVIC_SetPriority(
      SPI3_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(SPI3_IRQn);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0U;

  LL_SPI_Disable(SPI3);
  LL_SPI_Init(SPI3, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI3, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI3);
}

/**
  * @brief SPI4 Initialization Function
  * @retval None
  */
static void MX_SPI4_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI45_CLKSOURCE_PLL2Q);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);

  /*
   * PE2 -> SPI4_SCK
   * PE5 -> SPI4_MISO
   * PE6 -> SPI4_MOSI
   */
  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_2 |
      LL_GPIO_PIN_5 |
      LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* SPI4 RX: DMA2 Stream0 */
  LL_DMA_SetPeriphRequest(DMA2,
                          LL_DMA_STREAM_0,
                          LL_DMAMUX1_REQ_SPI4_RX);
  LL_DMA_SetDataTransferDirection(
      DMA2,
      LL_DMA_STREAM_0,
      LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(
      DMA2,
      LL_DMA_STREAM_0,
      LL_DMA_PRIORITY_VERYHIGH);
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(
      DMA2,
      LL_DMA_STREAM_0,
      LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(
      DMA2,
      LL_DMA_STREAM_0,
      LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(
      DMA2,
      LL_DMA_STREAM_0,
      LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(
      DMA2,
      LL_DMA_STREAM_0,
      LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

  /* SPI4 TX: DMA2 Stream1 */
  LL_DMA_SetPeriphRequest(DMA2,
                          LL_DMA_STREAM_1,
                          LL_DMAMUX1_REQ_SPI4_TX);
  LL_DMA_SetDataTransferDirection(
      DMA2,
      LL_DMA_STREAM_1,
      LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetStreamPriorityLevel(
      DMA2,
      LL_DMA_STREAM_1,
      LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(
      DMA2,
      LL_DMA_STREAM_1,
      LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(
      DMA2,
      LL_DMA_STREAM_1,
      LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(
      DMA2,
      LL_DMA_STREAM_1,
      LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(
      DMA2,
      LL_DMA_STREAM_1,
      LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_1);

  NVIC_SetPriority(
      SPI4_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(SPI4_IRQn);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0U;

  LL_SPI_Disable(SPI4);
  LL_SPI_Init(SPI4, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI4, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI4, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI4);
}

/**
  * @brief SPI5 Initialization Function
  * @retval None
  */
static void MX_SPI5_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI45_CLKSOURCE_PLL2Q);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI5);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF);

  /*
   * PF7 -> SPI5_SCK
   * PF8 -> SPI5_MISO
   * PF9 -> SPI5_MOSI
   */
  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_7 |
      LL_GPIO_PIN_8 |
      LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* SPI5 RX: DMA2 Stream2 */
  LL_DMA_SetPeriphRequest(DMA2,
                          LL_DMA_STREAM_2,
                          LL_DMAMUX1_REQ_SPI5_RX);
  LL_DMA_SetDataTransferDirection(
      DMA2,
      LL_DMA_STREAM_2,
      LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(
      DMA2,
      LL_DMA_STREAM_2,
      LL_DMA_PRIORITY_VERYHIGH);
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(
      DMA2,
      LL_DMA_STREAM_2,
      LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(
      DMA2,
      LL_DMA_STREAM_2,
      LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(
      DMA2,
      LL_DMA_STREAM_2,
      LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(
      DMA2,
      LL_DMA_STREAM_2,
      LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);

  /* SPI5 TX: DMA2 Stream3 */
  LL_DMA_SetPeriphRequest(DMA2,
                          LL_DMA_STREAM_3,
                          LL_DMAMUX1_REQ_SPI5_TX);
  LL_DMA_SetDataTransferDirection(
      DMA2,
      LL_DMA_STREAM_3,
      LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetStreamPriorityLevel(
      DMA2,
      LL_DMA_STREAM_3,
      LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(
      DMA2,
      LL_DMA_STREAM_3,
      LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(
      DMA2,
      LL_DMA_STREAM_3,
      LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(
      DMA2,
      LL_DMA_STREAM_3,
      LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(
      DMA2,
      LL_DMA_STREAM_3,
      LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_3);

  NVIC_SetPriority(
      SPI5_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(SPI5_IRQn);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0U;

  LL_SPI_Disable(SPI5);
  LL_SPI_Init(SPI5, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI5, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI5, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI5);
}

/**
  * @brief SPI6 Initialization Function
  * @retval None
  */
static void MX_SPI6_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI6_CLKSOURCE_PLL2Q);

  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SPI6);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);

  /*
   * PC12 -> SPI6_SCK
   * PG12 -> SPI6_MISO
   * PG14 -> SPI6_MOSI
   */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_12 | LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* SPI6 RX: BDMA Channel0 */
  LL_BDMA_SetPeriphRequest(
      BDMA,
      LL_BDMA_CHANNEL_0,
      LL_DMAMUX2_REQ_SPI6_RX);
  LL_BDMA_SetDataTransferDirection(
      BDMA,
      LL_BDMA_CHANNEL_0,
      LL_BDMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_BDMA_SetChannelPriorityLevel(
      BDMA,
      LL_BDMA_CHANNEL_0,
      LL_BDMA_PRIORITY_VERYHIGH);
  LL_BDMA_SetMode(
      BDMA,
      LL_BDMA_CHANNEL_0,
      LL_BDMA_MODE_NORMAL);
  LL_BDMA_SetPeriphIncMode(
      BDMA,
      LL_BDMA_CHANNEL_0,
      LL_BDMA_PERIPH_NOINCREMENT);
  LL_BDMA_SetMemoryIncMode(
      BDMA,
      LL_BDMA_CHANNEL_0,
      LL_BDMA_MEMORY_INCREMENT);
  LL_BDMA_SetPeriphSize(
      BDMA,
      LL_BDMA_CHANNEL_0,
      LL_BDMA_PDATAALIGN_BYTE);
  LL_BDMA_SetMemorySize(
      BDMA,
      LL_BDMA_CHANNEL_0,
      LL_BDMA_MDATAALIGN_BYTE);

  /* SPI6 TX: BDMA Channel1 */
  LL_BDMA_SetPeriphRequest(
      BDMA,
      LL_BDMA_CHANNEL_1,
      LL_DMAMUX2_REQ_SPI6_TX);
  LL_BDMA_SetDataTransferDirection(
      BDMA,
      LL_BDMA_CHANNEL_1,
      LL_BDMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_BDMA_SetChannelPriorityLevel(
      BDMA,
      LL_BDMA_CHANNEL_1,
      LL_BDMA_PRIORITY_HIGH);
  LL_BDMA_SetMode(
      BDMA,
      LL_BDMA_CHANNEL_1,
      LL_BDMA_MODE_NORMAL);
  LL_BDMA_SetPeriphIncMode(
      BDMA,
      LL_BDMA_CHANNEL_1,
      LL_BDMA_PERIPH_NOINCREMENT);
  LL_BDMA_SetMemoryIncMode(
      BDMA,
      LL_BDMA_CHANNEL_1,
      LL_BDMA_MEMORY_INCREMENT);
  LL_BDMA_SetPeriphSize(
      BDMA,
      LL_BDMA_CHANNEL_1,
      LL_BDMA_PDATAALIGN_BYTE);
  LL_BDMA_SetMemorySize(
      BDMA,
      LL_BDMA_CHANNEL_1,
      LL_BDMA_MDATAALIGN_BYTE);

  NVIC_SetPriority(
      SPI6_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(SPI6_IRQn);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0U;

  LL_SPI_Disable(SPI6);
  LL_SPI_Init(SPI6, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI6, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI6, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI6);
}

/**
  * @brief TIM6 Initialization Function
  * @retval None
  */
static void MX_TIM6_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  NVIC_SetPriority(
      TIM6_DAC_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  /*
   * TIM6CLK = 275 МГц:
   *
   * 275000000 / (274 + 1) = 1000000 Гц
   * 1000000 / (2499 + 1)  = 400 Гц
   */
  TIM_InitStruct.Prescaler = 274U;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 2499U;

  LL_TIM_Init(TIM6, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM6);
}

/**
  * @brief TIM7 Initialization Function
  * @retval None
  */
static void MX_TIM7_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

  /*
   * TIM7 — только watchdog stuck DMA/EOT.
   * Его приоритет ниже SPI/DMA/TIM6.
   */
  NVIC_SetPriority(
      TIM7_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
  NVIC_EnableIRQ(TIM7_IRQn);

  /*
   * TIM7CLK = 275 МГц:
   *
   * 275000000 / (274 + 1) = 1000000 Гц
   * 1000000 / (999 + 1)   = 1000 Гц
   */
  TIM_InitStruct.Prescaler = 274U;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 999U;

  LL_TIM_Init(TIM7, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM7);
  LL_TIM_SetTriggerOutput(TIM7, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM7);
}

/**
  * @brief USART1 Initialization Function
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART16_CLKSOURCE_PCLK2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);

  /*
   * PB14 -> USART1_TX
   * PB15 -> USART1_RX
   */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 RX: DMA1 Stream0 */
  LL_DMA_SetPeriphRequest(
      DMA1,
      LL_DMA_STREAM_0,
      LL_DMAMUX1_REQ_USART1_RX);
  LL_DMA_SetDataTransferDirection(
      DMA1,
      LL_DMA_STREAM_0,
      LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(
      DMA1,
      LL_DMA_STREAM_0,
      LL_DMA_PRIORITY_HIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(
      DMA1,
      LL_DMA_STREAM_0,
      LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(
      DMA1,
      LL_DMA_STREAM_0,
      LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(
      DMA1,
      LL_DMA_STREAM_0,
      LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(
      DMA1,
      LL_DMA_STREAM_0,
      LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);

  /* USART1 TX: DMA1 Stream1 */
  LL_DMA_SetPeriphRequest(
      DMA1,
      LL_DMA_STREAM_1,
      LL_DMAMUX1_REQ_USART1_TX);
  LL_DMA_SetDataTransferDirection(
      DMA1,
      LL_DMA_STREAM_1,
      LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetStreamPriorityLevel(
      DMA1,
      LL_DMA_STREAM_1,
      LL_DMA_PRIORITY_MEDIUM);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(
      DMA1,
      LL_DMA_STREAM_1,
      LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(
      DMA1,
      LL_DMA_STREAM_1,
      LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(
      DMA1,
      LL_DMA_STREAM_1,
      LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(
      DMA1,
      LL_DMA_STREAM_1,
      LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);

  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 12000000U;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_8;

  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(
      USART1,
      LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(
      USART1,
      LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);

  while ((LL_USART_IsActiveFlag_TEACK(USART1) == 0U) ||
         (LL_USART_IsActiveFlag_REACK(USART1) == 0U))
  {
  }
}

/**
  * @brief Enable BDMA controller clock
  */
static void MX_BDMA_Init(void)
{
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_BDMA);

  NVIC_SetPriority(
      BDMA_Channel0_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(BDMA_Channel0_IRQn);

  NVIC_SetPriority(
      BDMA_Channel1_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(BDMA_Channel1_IRQn);
}

/**
  * @brief Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  NVIC_SetPriority(
      DMA1_Stream0_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);

  NVIC_SetPriority(
      DMA1_Stream1_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA1_Stream1_IRQn);

  NVIC_SetPriority(
      DMA1_Stream2_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA1_Stream2_IRQn);

  NVIC_SetPriority(
      DMA1_Stream3_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA1_Stream3_IRQn);

  NVIC_SetPriority(
      DMA1_Stream4_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA1_Stream4_IRQn);

  NVIC_SetPriority(
      DMA1_Stream5_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  NVIC_SetPriority(
      DMA1_Stream6_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  NVIC_SetPriority(
      DMA1_Stream7_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);

  NVIC_SetPriority(
      DMA2_Stream0_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  NVIC_SetPriority(
      DMA2_Stream1_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  NVIC_SetPriority(
      DMA2_Stream2_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);

  NVIC_SetPriority(
      DMA2_Stream3_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOH);

  /*
   * Все CS устанавливаются в HIGH до переключения GPIO в output mode.
   */
  LL_GPIO_SetOutputPin(
      GPIOA,
      LL_GPIO_PIN_8 |
      LL_GPIO_PIN_9 |
      LL_GPIO_PIN_10);

  LL_GPIO_SetOutputPin(
      GPIOB,
      LL_GPIO_PIN_0 |
      LL_GPIO_PIN_1 |
      LL_GPIO_PIN_3 |
      LL_GPIO_PIN_4 |
      LL_GPIO_PIN_8 |
      LL_GPIO_PIN_12 |
      LL_GPIO_PIN_13);

  LL_GPIO_SetOutputPin(
      GPIOC,
      LL_GPIO_PIN_4 |
      LL_GPIO_PIN_5 |
      LL_GPIO_PIN_9);

  LL_GPIO_SetOutputPin(
      GPIOD,
      LL_GPIO_PIN_0 |
      LL_GPIO_PIN_1 |
      LL_GPIO_PIN_2 |
      LL_GPIO_PIN_3 |
      LL_GPIO_PIN_4 |
      LL_GPIO_PIN_5 |
      LL_GPIO_PIN_6 |
      LL_GPIO_PIN_7);

  LL_GPIO_SetOutputPin(
      GPIOE,
      LL_GPIO_PIN_0 |
      LL_GPIO_PIN_7 |
      LL_GPIO_PIN_8 |
      LL_GPIO_PIN_9 |
      LL_GPIO_PIN_10 |
      LL_GPIO_PIN_11 |
      LL_GPIO_PIN_14 |
      LL_GPIO_PIN_15);

  LL_GPIO_SetOutputPin(
      GPIOF,
      LL_GPIO_PIN_13 |
      LL_GPIO_PIN_14 |
      LL_GPIO_PIN_15);

  LL_GPIO_SetOutputPin(
      GPIOG,
      LL_GPIO_PIN_0 |
      LL_GPIO_PIN_1 |
      LL_GPIO_PIN_5 |
      LL_GPIO_PIN_9 |
      LL_GPIO_PIN_15);

  /* PE3 — аналоговый вход; не переводить в output. */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* GPIOA CS */
  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_8 |
      LL_GPIO_PIN_9 |
      LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* GPIOB CS */
  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_0 |
      LL_GPIO_PIN_1 |
      LL_GPIO_PIN_3 |
      LL_GPIO_PIN_4 |
      LL_GPIO_PIN_8 |
      LL_GPIO_PIN_12 |
      LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* GPIOC CS */
  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_4 |
      LL_GPIO_PIN_5 |
      LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* GPIOD CS */
  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_0 |
      LL_GPIO_PIN_1 |
      LL_GPIO_PIN_2 |
      LL_GPIO_PIN_3 |
      LL_GPIO_PIN_4 |
      LL_GPIO_PIN_5 |
      LL_GPIO_PIN_6 |
      LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* GPIOE CS */
  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_0 |
      LL_GPIO_PIN_7 |
      LL_GPIO_PIN_8 |
      LL_GPIO_PIN_9 |
      LL_GPIO_PIN_10 |
      LL_GPIO_PIN_11 |
      LL_GPIO_PIN_14 |
      LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* GPIOF CS */
  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_13 |
      LL_GPIO_PIN_14 |
      LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* GPIOG CS + external clock enable */
  LL_GPIO_SetOutputPin(GPIOG, LL_GPIO_PIN_5);

  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_0 |
      LL_GPIO_PIN_1 |
      LL_GPIO_PIN_5 |
      LL_GPIO_PIN_9 |
      LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief MPU Configuration
  * @retval None
  */
static void MPU_Config(void)
{
  LL_MPU_Disable();

  /*
   * Region 0: D2 SRAM, SPI1..SPI5 DMA buffers.
   */
  LL_MPU_ConfigRegion(
      LL_MPU_REGION_NUMBER0,
      0x00U,
      0x30000000U,
      LL_MPU_REGION_SIZE_256KB |
      LL_MPU_REGION_PRIV_RW |
      LL_MPU_ACCESS_NOT_BUFFERABLE |
      LL_MPU_ACCESS_NOT_CACHEABLE |
      LL_MPU_ACCESS_SHAREABLE |
      LL_MPU_INSTRUCTION_ACCESS_DISABLE);

  LL_MPU_EnableRegion(LL_MPU_REGION_NUMBER0);

  /*
   * Region 1: D3/SRAM4, SPI6 BDMA RX/TX buffers.
   */
  LL_MPU_ConfigRegion(
      LL_MPU_REGION_NUMBER1,
      0x00U,
      0x38000000U,
      LL_MPU_REGION_SIZE_32KB |
      LL_MPU_REGION_PRIV_RW |
      LL_MPU_ACCESS_NOT_BUFFERABLE |
      LL_MPU_ACCESS_NOT_CACHEABLE |
      LL_MPU_ACCESS_SHAREABLE |
      LL_MPU_INSTRUCTION_ACCESS_DISABLE);

  LL_MPU_EnableRegion(LL_MPU_REGION_NUMBER1);
  LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);
}

/**
  * @brief Error handler
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT

/**
  * @brief Reports assertion source
  * @param file Source file
  * @param line Source line
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}

#endif /* USE_FULL_ASSERT */

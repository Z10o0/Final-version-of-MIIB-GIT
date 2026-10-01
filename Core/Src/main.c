/* USER CODE BEGIN Header */
/*
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "icm45686_spi.h"
#include "icm45686_data.h"
#include "uart_telemetry.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* USER CODE BEGIN PV */
/* USER CODE END PV */

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

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  MPU_Config();

  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  NVIC_SetPriority(
      SysTick_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();

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

  uint64_t imu_faults = ICM_InitAllSensors();
  (void)imu_faults;

  UART_Telemetry_Init();

  /*
   * TIM7 watchdog: 1 кГц.
   * NVIC preemption priority 3, ниже SPI/DMA/TIM6.
   */
  LL_TIM_SetCounter(TIM7, 0U);
  LL_TIM_ClearFlag_UPDATE(TIM7);
  LL_TIM_EnableIT_UPDATE(TIM7);
  LL_TIM_EnableCounter(TIM7);

  /*
   * TIM6: 2.5 мс, восемь FIFO-пакетов @ 3200 Гц,
   * один output frame @ 400 Гц.
   */
  LL_TIM_SetCounter(TIM6, 0U);
  LL_TIM_ClearFlag_UPDATE(TIM6);
  LL_TIM_EnableIT_UPDATE(TIM6);
  LL_TIM_EnableCounter(TIM6);

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t events = ICM_ConsumeEvents();

    /*
     * Pipeline выполняется только в main loop:
     * raw FIFO parsing -> averaging -> один RS-кадр.
     */
    if ((events & ICM_EVT_BATCH_READY) != 0U)
    {
      ICM_ParseAllFIFO();
      ICM_AverageAllBatches();
      UART_BuildAndSendSyncFrame();
    }

    /*
     * Recovery выполняется существующей FSM.
     * Здесь не запускается блокирующий register/FIFO I/O.
     */
    if ((events & (ICM_EVT_BUS0_FAULT |
                   ICM_EVT_BUS1_FAULT |
                   ICM_EVT_BUS2_FAULT)) != 0U)
    {
    }

    if ((events & ICM_EVT_DMA_TIMEOUT) != 0U)
    {
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);

  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }

  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);

  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }

  LL_RCC_HSE_Enable();

  while (LL_RCC_HSE_IsReady() != 1)
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

  while (LL_RCC_PLL1_IsReady() != 1)
  {
  }

  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
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

  while (LL_RCC_PLL2_IsReady() != 1)
  {
  }
}

static void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL2P);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);

  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;

  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;

  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  LL_DMA_SetPeriphRequest(
      DMA1, LL_DMA_STREAM_2, LL_DMAMUX1_REQ_SPI1_RX);

  LL_DMA_SetDataTransferDirection(
      DMA1, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(
      DMA1, LL_DMA_STREAM_2, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(
      DMA1, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(
      DMA1, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(
      DMA1, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(
      DMA1, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_2);

  LL_DMA_SetPeriphRequest(
      DMA1, LL_DMA_STREAM_3, LL_DMAMUX1_REQ_SPI1_TX);

  LL_DMA_SetDataTransferDirection(
      DMA1, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(
      DMA1, LL_DMA_STREAM_3, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(
      DMA1, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(
      DMA1, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(
      DMA1, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(
      DMA1, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_3);

  NVIC_SetPriority(
      SPI1_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  NVIC_EnableIRQ(SPI1_IRQn);

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
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

static void MX_SPI2_Init(void)
{
  /* USER CODE BEGIN SPI2_Init 0 */
  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL2P);

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);

  LL_SYSCFG_CloseAnalogSwitch(LL_SYSCFG_ANALOG_SWITCH_PC2);

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

  LL_DMA_SetPeriphRequest(
      DMA1, LL_DMA_STREAM_4, LL_DMAMUX1_REQ_SPI2_RX);

  LL_DMA_SetDataTransferDirection(
      DMA1, LL_DMA_STREAM_4, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(
      DMA1, LL_DMA_STREAM_4, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(
      DMA1, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(
      DMA1, LL_DMA_STREAM_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(
      DMA1, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(
      DMA1, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_4);

  LL_DMA_SetPeriphRequest(
      DMA1, LL_DMA_STREAM_5, LL_DMAMUX1_REQ_SPI2_TX);

  LL_DMA_SetDataTransferDirection(
      DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(
      DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(
      DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(
      DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(
      DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(
      DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

  NVIC_SetPriority(
      SPI2_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  NVIC_EnableIRQ(SPI2_IRQn);

  /* USER CODE BEGIN SPI2_Init 1 */
  /* USER CODE END SPI2_Init 1 */

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;

  LL_SPI_Disable(SPI2);
  LL_SPI_Init(SPI2, &SPI_InitStruct);

  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI2, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI2);

  /* USER CODE BEGIN SPI2_Init 2 */
  /* USER CODE END SPI2_Init 2 */
}

static void MX_SPI3_Init(void)
{
  /* USER CODE BEGIN SPI3_Init 0 */
  /* USER CODE END SPI3_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL2P);

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);

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

  LL_DMA_SetPeriphRequest(
      DMA1, LL_DMA_STREAM_6, LL_DMAMUX1_REQ_SPI3_RX);

  LL_DMA_SetDataTransferDirection(
      DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(
      DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(
      DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(
      DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(
      DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(
      DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);

  LL_DMA_SetPeriphRequest(
      DMA1, LL_DMA_STREAM_7, LL_DMAMUX1_REQ_SPI3_TX);

  LL_DMA_SetDataTransferDirection(
      DMA1, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(
      DMA1, LL_DMA_STREAM_7, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(
      DMA1, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(
      DMA1, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(
      DMA1, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(
      DMA1, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_7);

  NVIC_SetPriority(
      SPI3_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  NVIC_EnableIRQ(SPI3_IRQn);

  /* USER CODE BEGIN SPI3_Init 1 */
  /* USER CODE END SPI3_Init 1 */

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;

  LL_SPI_Disable(SPI3);
  LL_SPI_Init(SPI3, &SPI_InitStruct);

  LL_SPI_SetStandard(SPI3, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI3, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI3);

  /* USER CODE BEGIN SPI3_Init 2 */
  /* USER CODE END SPI3_Init 2 */
}

static void MX_SPI4_Init(void)
{
  /* USER CODE BEGIN SPI4_Init 0 */
  /* USER CODE END SPI4_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI45_CLKSOURCE_PLL2Q);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);

  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_2 | LL_GPIO_PIN_5 | LL_GPIO_PIN_6;

  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;

  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  LL_DMA_SetPeriphRequest(
      DMA2, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_SPI4_RX);

  LL_DMA_SetDataTransferDirection(
      DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(
      DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(
      DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(
      DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(
      DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(
      DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

  LL_DMA_SetPeriphRequest(
      DMA2, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_SPI4_TX);

  LL_DMA_SetDataTransferDirection(
      DMA2, LL_DMA_STREAM_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(
      DMA2, LL_DMA_STREAM_1, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(
      DMA2, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(
      DMA2, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(
      DMA2, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(
      DMA2, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_1);

  NVIC_SetPriority(
      SPI4_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  NVIC_EnableIRQ(SPI4_IRQn);

  /* USER CODE BEGIN SPI4_Init 1 */
  /* USER CODE END SPI4_Init 1 */

  LL_SPI_Disable(SPI4);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
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

static void MX_SPI5_Init(void)
{
  /* USER CODE BEGIN SPI5_Init 0 */
  /* USER CODE END SPI5_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI45_CLKSOURCE_PLL2Q);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI5);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOF);

  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9;

  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;

  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  LL_DMA_SetPeriphRequest(
      DMA2, LL_DMA_STREAM_2, LL_DMAMUX1_REQ_SPI5_RX);

  LL_DMA_SetDataTransferDirection(
      DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(
      DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(
      DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(
      DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(
      DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(
      DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);

  LL_DMA_SetPeriphRequest(
      DMA2, LL_DMA_STREAM_3, LL_DMAMUX1_REQ_SPI5_TX);

  LL_DMA_SetDataTransferDirection(
      DMA2, LL_DMA_STREAM_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(
      DMA2, LL_DMA_STREAM_3, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(
      DMA2, LL_DMA_STREAM_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(
      DMA2, LL_DMA_STREAM_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(
      DMA2, LL_DMA_STREAM_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(
      DMA2, LL_DMA_STREAM_3, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_3);

  NVIC_SetPriority(
      SPI5_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  NVIC_EnableIRQ(SPI5_IRQn);

  /* USER CODE BEGIN SPI5_Init 1 */
  /* USER CODE END SPI5_Init 1 */

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
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

static void MX_SPI6_Init(void)
{
  /* USER CODE BEGIN SPI6_Init 0 */
  /* USER CODE END SPI6_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetSPIClockSource(LL_RCC_SPI6_CLKSOURCE_PLL2Q);

  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SPI6);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);

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

  LL_BDMA_SetPeriphRequest(
      BDMA, LL_BDMA_CHANNEL_0, LL_DMAMUX2_REQ_SPI6_RX);

  LL_BDMA_SetDataTransferDirection(
      BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_BDMA_SetChannelPriorityLevel(
      BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PRIORITY_VERYHIGH);

  LL_BDMA_SetMode(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MODE_NORMAL);

  LL_BDMA_SetPeriphIncMode(
      BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PERIPH_NOINCREMENT);

  LL_BDMA_SetMemoryIncMode(
      BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MEMORY_INCREMENT);

  LL_BDMA_SetPeriphSize(
      BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PDATAALIGN_BYTE);

  LL_BDMA_SetMemorySize(
      BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MDATAALIGN_BYTE);

  LL_BDMA_SetPeriphRequest(
      BDMA, LL_BDMA_CHANNEL_1, LL_DMAMUX2_REQ_SPI6_TX);

  LL_BDMA_SetDataTransferDirection(
      BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_BDMA_SetChannelPriorityLevel(
      BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PRIORITY_HIGH);

  LL_BDMA_SetMode(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MODE_NORMAL);

  LL_BDMA_SetPeriphIncMode(
      BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PERIPH_NOINCREMENT);

  LL_BDMA_SetMemoryIncMode(
      BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MEMORY_INCREMENT);

  LL_BDMA_SetPeriphSize(
      BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PDATAALIGN_BYTE);

  LL_BDMA_SetMemorySize(
      BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MDATAALIGN_BYTE);

  NVIC_SetPriority(
      SPI6_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  NVIC_EnableIRQ(SPI6_IRQn);

  /* USER CODE BEGIN SPI6_Init 1 */
  /* USER CODE END SPI6_Init 1 */

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 0x0;

  LL_SPI_Disable(SPI6);
  LL_SPI_Init(SPI6, &SPI_InitStruct);

  LL_SPI_SetStandard(SPI6, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_SetFIFOThreshold(SPI6, LL_SPI_FIFO_TH_01DATA);
  LL_SPI_DisableNSSPulseMgt(SPI6);

  /* USER CODE BEGIN SPI6_Init 2 */
  /* USER CODE END SPI6_Init 2 */
}

static void MX_TIM6_Init(void)
{
  /* USER CODE BEGIN TIM6_Init 0 */
  /* USER CODE END TIM6_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  NVIC_SetPriority(
      TIM6_DAC_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  /* USER CODE BEGIN TIM6_Init 1 */
  /* USER CODE END TIM6_Init 1 */

  /*
   * TIM6CLK = 275 MHz.
   * 275 MHz / (274 + 1) = 1 MHz.
   * 1 MHz / (2499 + 1) = 400 Hz.
   */
  TIM_InitStruct.Prescaler = 274U;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 2499U;

  LL_TIM_Init(TIM6, &TIM_InitStruct);

  LL_TIM_DisableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM6);

  /* USER CODE BEGIN TIM6_Init 2 */
  /* USER CODE END TIM6_Init 2 */
}

static void MX_TIM7_Init(void)
{
  /* USER CODE BEGIN TIM7_Init 0 */
  /* USER CODE END TIM7_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

  /* USER CODE BEGIN TIM7_Init 1 */
  /* USER CODE END TIM7_Init 1 */

  /*
   * TIM7CLK = 275 MHz.
   * 275 MHz / (274 + 1) / (999 + 1) = 1 kHz.
   */
  TIM_InitStruct.Prescaler = 274U;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 999U;

  LL_TIM_Init(TIM7, &TIM_InitStruct);

  LL_TIM_DisableARRPreload(TIM7);
  LL_TIM_SetTriggerOutput(TIM7, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM7);

  /* USER CODE BEGIN TIM7_Init 2 */

  NVIC_SetPriority(
      TIM7_IRQn,
      NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3U, 0U));

  NVIC_EnableIRQ(TIM7_IRQn);

  /* USER CODE END TIM7_Init 2 */
}

static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART16_CLKSOURCE_PCLK2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;

  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_DMA_SetPeriphRequest(
      DMA1, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_USART1_RX);

  LL_DMA_SetDataTransferDirection(
      DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(
      DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(
      DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(
      DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(
      DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(
      DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);

  LL_DMA_SetPeriphRequest(
      DMA1, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_USART1_TX);

  LL_DMA_SetDataTransferDirection(
      DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(
      DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(
      DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(
      DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(
      DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(
      DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */

  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 6000000;
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

  while ((!LL_USART_IsActiveFlag_TEACK(USART1)) ||
         (!LL_USART_IsActiveFlag_REACK(USART1)))
  {
  }

  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */
}

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

  LL_GPIO_SetOutputPin(
      GPIOA,
      LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10);

  LL_GPIO_SetOutputPin(
      GPIOB,
      LL_GPIO_PIN_0 | LL_GPIO_PIN_1 |
      LL_GPIO_PIN_3 | LL_GPIO_PIN_4 |
      LL_GPIO_PIN_8 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13);

  LL_GPIO_SetOutputPin(
      GPIOC,
      LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_9);

  LL_GPIO_SetOutputPin(
      GPIOD,
      LL_GPIO_PIN_0 | LL_GPIO_PIN_1 |
      LL_GPIO_PIN_2 | LL_GPIO_PIN_3 |
      LL_GPIO_PIN_4 | LL_GPIO_PIN_5 |
      LL_GPIO_PIN_6 | LL_GPIO_PIN_7);

  LL_GPIO_SetOutputPin(
      GPIOE,
      LL_GPIO_PIN_0 | LL_GPIO_PIN_7 |
      LL_GPIO_PIN_8 | LL_GPIO_PIN_9 |
      LL_GPIO_PIN_10 | LL_GPIO_PIN_11 |
      LL_GPIO_PIN_14 | LL_GPIO_PIN_15);

  LL_GPIO_SetOutputPin(
      GPIOF,
      LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);

  LL_GPIO_SetOutputPin(
      GPIOG,
      LL_GPIO_PIN_0 | LL_GPIO_PIN_1 |
      LL_GPIO_PIN_5 | LL_GPIO_PIN_9 | LL_GPIO_PIN_15);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;

  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_0 | LL_GPIO_PIN_1 |
      LL_GPIO_PIN_3 | LL_GPIO_PIN_4 |
      LL_GPIO_PIN_8 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13;

  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_9;

  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_0 | LL_GPIO_PIN_1 |
      LL_GPIO_PIN_2 | LL_GPIO_PIN_3 |
      LL_GPIO_PIN_4 | LL_GPIO_PIN_5 |
      LL_GPIO_PIN_6 | LL_GPIO_PIN_7;

  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_0 | LL_GPIO_PIN_7 |
      LL_GPIO_PIN_8 | LL_GPIO_PIN_9 |
      LL_GPIO_PIN_10 | LL_GPIO_PIN_11 |
      LL_GPIO_PIN_14 | LL_GPIO_PIN_15;

  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;

  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  LL_GPIO_SetOutputPin(GPIOG, LL_GPIO_PIN_5);

  GPIO_InitStruct.Pin =
      LL_GPIO_PIN_0 | LL_GPIO_PIN_1 |
      LL_GPIO_PIN_5 | LL_GPIO_PIN_9 | LL_GPIO_PIN_15;

  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void MPU_Config(void)
{
  LL_MPU_Disable();

  /* D2: SPI1...SPI5 DMA buffers. */
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

  /* D3/SRAM4: SPI6 BDMA buffers. */
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

  /*
   * AXI SRAM UART queue не покрывается новым MPU region.
   * Cache coherency обеспечивается UART_CleanTxDCache().
   */
  LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);
}

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

void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  (void)file;
  (void)line;

  /* USER CODE END 6 */
}

#endif

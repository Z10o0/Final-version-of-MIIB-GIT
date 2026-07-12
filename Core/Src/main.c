/* USER CODE BEGIN Header */
/**
 * @file    main.c
 * @brief   Точка входа. Инициализация и главный цикл.
 *
 *          Алгоритм работы:
 *          1. Системная инициализация (тактирование, GPIO, SPI, DMA, TIM)
 *          2. ICM_BusesInit()       — сброс CS и fault-флагов
 *          3. ICM_InitAllSensors()  — инициализация 18 IMU (fault-tolerant)
 *          4. UART_Telemetry_Init() — подготовка DMA UART TX
 *          5. TIM6 запущен         — генерирует прерывание каждые 3.125 мс
 *
 *          В каждом прерывании TIM6:
 *          → ICM_StartBurstRead() → DMA-каскад (SPI1→SPI4→SPI5)
 *          → g_fifo_batch_ready = 1
 *
 *          В main-loop:
 *          → ICM_ParseAllFIFO()  — разбор FIFO, нули для fault-датчиков
 *          → UART_SendBatch()    — отправка пакета по USART1 DMA
 *
 *          Неисправные датчики: fault_mask возвращается ICM_InitAllSensors().
 *          Система продолжает работать. В UART-пакете их данные = нули.
 */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "icm45686_spi.h"     /* ICM_BusesInit, ICM_InitAllSensors, DMA-транспорт */
#include "icm45686_data.h"    /* g_fifo_batch_ready, ICM_ParseAllFIFO */
#include "uart_telemetry.h"   /* UART_Telemetry_Init, UART_SendBatch */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Маска неисправных датчиков: бит N = датчик N не ответил на WHO_AM_I.
 * 0 = все датчики исправны. Устанавливается при старте, не меняется в рантайме. */
static volatile uint32_t s_init_fault_mask = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */

  /* Шаг 1: Сбросить CS-пины и fault-флаги всех датчиков */
  ICM_BusesInit();

  /* Шаг 2: Инициализация датчиков (fault-tolerant).
   * Возвращает маску неисправных датчиков (0 = все ОК).
   * Система продолжает работать при любом результате. */
  s_init_fault_mask = ICM_InitAllSensors();

  /* Шаг 3: Инициализация DMA-телеметрии по USART1 */
  UART_Telemetry_Init();

  /* Шаг 4: Запустить TIM6 — генерирует прерывания каждые 3.125 мс (320 Гц тактов по 10 пакетов = 3200 Гц ODR) */
  LL_TIM_EnableCounter(TIM6);
  LL_TIM_EnableIT_UPDATE(TIM6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Ожидание готовности новой пачки данных (флаг устанавливается в ISR DMA SPI5) */
    if (g_fifo_batch_ready != 0U)
    {
        /* Снять флаг ДО разбора — новый цикл DMA может стартовать пока парсим */
        g_fifo_batch_ready = 0U;

        /* Разобрать FIFO-буферы всех датчиков.
         * Неисправные (g_sensor_fault_mask) → нули в g_sensor_batches. */
        ICM_ParseAllFIFO();

        /* Отправить пакет по USART1 DMA (неблокирующий).
         * Если UART ещё занят передачей — пакет пропускается. */
        UART_SendBatch();
    }

    /* USER CODE END 3 */
  }
}

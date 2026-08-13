/* =============================================================================
 * stm32h7xx_it.c
 *
 * Обработчики прерываний для MIIB (18× ICM-45686, SPI1/5/4 + UART1).
 *
 * DMA-карта:
 *   DMA1 Stream0  — USART1 RX  (не используется в текущей прошивке)
 *   DMA1 Stream1  — USART1 TX  → UART_DMA_TxComplete
 *   DMA1 Stream2  — SPI1  RX   → ICM_DMA_RxComplete_SPI1
 *   DMA1 Stream3  — SPI1  TX   (только сброс флагов)
 *   DMA2 Stream0  — SPI4  RX   → ICM_DMA_RxComplete_SPI4
 *   DMA2 Stream1  — SPI4  TX   (только сброс флагов)
 *   DMA2 Stream2  — SPI5  RX   → ICM_DMA_RxComplete_SPI5
 *   DMA2 Stream3  — SPI5  TX   (только сброс флагов)
 *
 * SPI IRQ:
 *   SPI1/4/5_IRQHandler — EOT → ICM_SPI_Eot_SPIx
 *   (CS поднимается только после EOT, не после DMA TC)
 *
 * TIM6:
 *   UPDATE → ICM_StartBurstRead()  (~3.125 мс = 320 Гц)
 * =============================================================================
 */

#include "main.h"
#include "stm32h7xx_it.h"
#include "icm45686_spi.h"
#include "uart_telemetry.h"

/* ===========================================================================
 *  Системные fault-handlers
 * ========================================================================== */

void NMI_Handler(void)
{
    while (1) {}
}

void HardFault_Handler(void)
{
    while (1) {}
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}

/* ===========================================================================
 * UART1 TX DMA: DMA1 Stream1
 *
 * DMA1 Stream1 используется только модулем uart_telemetry.c для USART1_TX.
 *
 * При TC:
 *   - UART_DMA_TxComplete() освобождает переданный слот TX ring-buffer;
 *   - при наличии следующего пакета сам запускает очередную DMA-передачу.
 *
 * При TE / DME / FE:
 *   - флаг очищается;
 *   - соответствующий error-counter увеличивается;
 *   - для отладки рекомендуется поставить breakpoint.
 * ========================================================================== */
void DMA1_Stream1_IRQHandler(void)
{
    /* ---------------------------------------------------------------
     * Transfer error.
     *
     * При TE DMA stream может быть аппаратно отключён. Просто очистить
     * флаг недостаточно для продолжения передачи; остановись здесь
     * breakpoint-ом и проверь DMA конфигурацию / D2 SRAM / DMAMUX.
     * --------------------------------------------------------------- */
    if (LL_DMA_IsActiveFlag_TE1(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TE1(DMA1);
        g_uart_dma_te_count++;
    }

    /* ---------------------------------------------------------------
     * Direct mode error.
     * --------------------------------------------------------------- */
    if (LL_DMA_IsActiveFlag_DME1(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_DME1(DMA1);
        g_uart_dma_dme_count++;
    }

    /* ---------------------------------------------------------------
     * FIFO error.
     * --------------------------------------------------------------- */
    if (LL_DMA_IsActiveFlag_FE1(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_FE1(DMA1);
        g_uart_dma_fe_count++;
    }

    /* ---------------------------------------------------------------
     * Transfer complete.
     *
     * Сначала очищаем аппаратный DMA flag, затем вызываем consumer
     * очереди. UART_DMA_TxComplete() запускает следующий пакет, если
     * очередь не пуста.
     * --------------------------------------------------------------- */
    if (LL_DMA_IsActiveFlag_TC1(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TC1(DMA1);
        UART_DMA_TxComplete();
    }
}

/* ===========================================================================
 *  SPI1 RX DMA: DMA1 Stream2
 * ========================================================================== */
void DMA1_Stream2_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE2(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TE2(DMA1);
        ICM_DMA_Error_SPI1();
        return;
    }

    if (LL_DMA_IsActiveFlag_TC2(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TC2(DMA1);
        ICM_DMA_RxComplete_SPI1();
    }
}

/* ===========================================================================
 *  SPI1 TX DMA: DMA1 Stream3 — только сброс флагов (логика на RX TC)
 * ========================================================================== */
void DMA1_Stream3_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE3(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TE3(DMA1);
    }

    if (LL_DMA_IsActiveFlag_TC3(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TC3(DMA1);
    }
}

/* ===========================================================================
 *  TIM6 UPDATE — запуск DMA-цикла опроса
 *
 *  Период ≈ 3.125 мс (PSC=274, ARR=3124 @ APB1 timer 275 МГц)
 *  → 320 Гц = ODR 3200 / 10 пакетов в FIFO.
 * ========================================================================== */
void TIM6_DAC_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6) != 0U)
    {
        LL_TIM_ClearFlag_UPDATE(TIM6);
        ICM_StartBurstRead();
    }
}

/* ===========================================================================
 *  [NEW] TIM7 UPDATE — system watchdog tick.
 *
 *  TIM7 уже инициализирован в MX_TIM7_Init() (main.c), но раньше не имел
 *  назначенного IRQ handler. Используем его как источник тактов watchdog'а
 *  на частоте заметно выше 100 Гц (рекомендуется настроить TIM7 на ~1 кГц
 *  в CubeMX/.ioc), чтобы обнаруживать stuck DMA/EOT задолго до следующего
 *  TIM6 burst и не ждать целый 10-мс цикл до восстановления шины.
 *  ВАЖНО: приоритет NVIC для TIM7 должен быть НИЖЕ приоритета SPI/DMA IRQ,
 *  чтобы watchdog не мог прервать штатную acquisition-транзакцию.
 * ========================================================================== */
void TIM7_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM7) != 0U)
    {
        LL_TIM_ClearFlag_UPDATE(TIM7);
        ICM_WatchdogTick();
    }
}

/* ===========================================================================
 *  SPI4 RX DMA: DMA2 Stream0
 * ========================================================================== */
void DMA2_Stream0_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE0(DMA2) != 0U)
    {
        LL_DMA_ClearFlag_TE0(DMA2);
        ICM_DMA_Error_SPI4();
        return;
    }

    if (LL_DMA_IsActiveFlag_TC0(DMA2) != 0U)
    {
        LL_DMA_ClearFlag_TC0(DMA2);
        ICM_DMA_RxComplete_SPI4();
    }
}

/* ===========================================================================
 *  SPI4 TX DMA: DMA2 Stream1 — только сброс флагов
 * ========================================================================== */
void DMA2_Stream1_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE1(DMA2) != 0U)
    {
        LL_DMA_ClearFlag_TE1(DMA2);
    }

    if (LL_DMA_IsActiveFlag_TC1(DMA2) != 0U)
    {
        LL_DMA_ClearFlag_TC1(DMA2);
    }
}

/* ===========================================================================
 *  SPI5 RX DMA: DMA2 Stream2
 * ========================================================================== */
void DMA2_Stream2_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE2(DMA2) != 0U)
    {
        LL_DMA_ClearFlag_TE2(DMA2);
        ICM_DMA_Error_SPI5();
        return;
    }

    if (LL_DMA_IsActiveFlag_TC2(DMA2) != 0U)
    {
        LL_DMA_ClearFlag_TC2(DMA2);
        ICM_DMA_RxComplete_SPI5();
    }
}

/* ===========================================================================
 *  SPI5 TX DMA: DMA2 Stream3 — только сброс флагов
 * ========================================================================== */
void DMA2_Stream3_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE3(DMA2) != 0U)
    {
        LL_DMA_ClearFlag_TE3(DMA2);
    }

    if (LL_DMA_IsActiveFlag_TC3(DMA2) != 0U)
    {
        LL_DMA_ClearFlag_TC3(DMA2);
    }
}

/* ===========================================================================
 *  SPIx_IRQHandler — EOT (End Of Transfer)
 *
 *  Архитектура (см. icm45686_spi.c):
 *    DMA RX TC  → ICM_NextSensor: выкл DMA, вкл EOT IRQ, CS остаётся LOW
 *    SPI EOT    → ICM_SPI_Eot_SPIx → ICM_OnSpiEot: CS HIGH, SPE=0, next
 *
 *  Приоритет NVIC SPI IRQ должен быть НЕ ниже DMA IRQ
 *  (чтобы EOT не «терялся» за длинным DMA handler).
 *
 *  OVR сбрасываем, чтобы не зациклить IRQ.
 * ========================================================================== */
void SPI1_IRQHandler(void)
{
    /* Проверяем ОБА условия: прерывание разрешено И флаг выставлен */
    if ((LL_SPI_IsEnabledIT_EOT(SPI1) != 0U) &&
        (LL_SPI_IsActiveFlag_EOT(SPI1) != 0U))
    {
        ICM_SPI_Eot_SPI1();
    }

    if (LL_SPI_IsActiveFlag_OVR(SPI1) != 0U)
    {
        LL_SPI_ClearFlag_OVR(SPI1);
    }
}

void SPI4_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI4) != 0U) &&
        (LL_SPI_IsActiveFlag_EOT(SPI4) != 0U))
    {
        ICM_SPI_Eot_SPI4();
    }

    if (LL_SPI_IsActiveFlag_OVR(SPI4) != 0U)
    {
        LL_SPI_ClearFlag_OVR(SPI4);
    }
}

void SPI5_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI5) != 0U) &&
        (LL_SPI_IsActiveFlag_EOT(SPI5) != 0U))
    {
        ICM_SPI_Eot_SPI5();
    }

    if (LL_SPI_IsActiveFlag_OVR(SPI5) != 0U)
    {
        LL_SPI_ClearFlag_OVR(SPI5);
    }
}

void SPI2_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI2) != 0U) && (LL_SPI_IsActiveFlag_EOT(SPI2) != 0U))
    {
        ICM_SPI_Eot_SPI2();
    }
    if (LL_SPI_IsActiveFlag_OVR(SPI2) != 0U) { LL_SPI_ClearFlag_OVR(SPI2); }
}

void SPI3_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI3) != 0U) && (LL_SPI_IsActiveFlag_EOT(SPI3) != 0U))
    {
        ICM_SPI_Eot_SPI3();
    }
    if (LL_SPI_IsActiveFlag_OVR(SPI3) != 0U) { LL_SPI_ClearFlag_OVR(SPI3); }
}

void SPI6_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI6) != 0U) && (LL_SPI_IsActiveFlag_EOT(SPI6) != 0U))
    {
        ICM_SPI_Eot_SPI6();
    }
    if (LL_SPI_IsActiveFlag_OVR(SPI6) != 0U) { LL_SPI_ClearFlag_OVR(SPI6); }
}

void DMA1_Stream0_IRQHandler(void) {}
void DMA1_Stream4_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE4(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TE4(DMA1);
        ICM_DMA_Error_SPI2();
        return;
    }
    if (LL_DMA_IsActiveFlag_TC4(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TC4(DMA1);
        ICM_DMA_RxComplete_SPI2();
    }
}

void DMA1_Stream5_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE5(DMA1) != 0U) { LL_DMA_ClearFlag_TE5(DMA1); }
    if (LL_DMA_IsActiveFlag_TC5(DMA1) != 0U) { LL_DMA_ClearFlag_TC5(DMA1); }
}

void DMA1_Stream6_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE6(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TE6(DMA1);
        ICM_DMA_Error_SPI3();
        return;
    }
    if (LL_DMA_IsActiveFlag_TC6(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TC6(DMA1);
        ICM_DMA_RxComplete_SPI3();
    }
}

void DMA1_Stream7_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE7(DMA1) != 0U) { LL_DMA_ClearFlag_TE7(DMA1); }
    if (LL_DMA_IsActiveFlag_TC7(DMA1) != 0U) { LL_DMA_ClearFlag_TC7(DMA1); }
}

void BDMA_Channel0_IRQHandler(void)
{
    if (LL_BDMA_IsActiveFlag_TE0(BDMA) != 0U)
    {
        LL_BDMA_ClearFlag_TE0(BDMA);
        ICM_DMA_Error_SPI6();
        return;
    }
    if (LL_BDMA_IsActiveFlag_TC0(BDMA) != 0U)
    {
        LL_BDMA_ClearFlag_TC0(BDMA);
        ICM_DMA_RxComplete_SPI6();
    }
}

void BDMA_Channel1_IRQHandler(void)
{
    if (LL_BDMA_IsActiveFlag_TE1(BDMA) != 0U) { LL_BDMA_ClearFlag_TE1(BDMA); }
    if (LL_BDMA_IsActiveFlag_TC1(BDMA) != 0U) { LL_BDMA_ClearFlag_TC1(BDMA); }
}

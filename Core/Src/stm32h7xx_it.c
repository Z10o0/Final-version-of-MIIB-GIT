/* =============================================================================
 * stm32h7xx_it.c
 *
 * Обработчики прерываний для MIIB (36× ICM-45686, SPI1/5/4 + SPI3/2/6 + UART1).
 *
 * DMA-карта (нижняя плата):
 *   DMA1 Stream0  — USART1 RX  (не используется в текущей прошивке)
 *   DMA1 Stream1  — USART1 TX  → UART_DMA_TxComplete
 *   DMA1 Stream2  — SPI1  RX   → ICM_DMA_RxComplete_SPI1
 *   DMA1 Stream3  — SPI1  TX   (только сброс флагов)
 *   DMA2 Stream0  — SPI4  RX   → ICM_DMA_RxComplete_SPI4
 *   DMA2 Stream1  — SPI4  TX   (только сброс флагов)
 *   DMA2 Stream2  — SPI5  RX   → ICM_DMA_RxComplete_SPI5
 *   DMA2 Stream3  — SPI5  TX   (только сброс флагов)
 *
 * DMA-карта (верхняя плата):
 *   DMA1 Stream4  — SPI2  RX   → ICM_DMA_RxComplete_SPI2
 *   DMA1 Stream5  — SPI2  TX   (только сброс флагов)
 *   DMA1 Stream6  — SPI3  RX   → ICM_DMA_RxComplete_SPI3
 *   DMA1 Stream7  — SPI3  TX   (только сброс флагов)
 *   BDMA Channel0 — SPI6  RX   → ICM_DMA_RxComplete_SPI6  (домен D3/SRAM4)
 *   BDMA Channel1 — SPI6  TX   (только сброс флагов)
 *
 * SPI IRQ (EOT):
 *   SPI1/4/5_IRQHandler  — нижняя плата → ICM_SPI_Eot_SPIx
 *   SPI2/3/6_IRQHandler  — верхняя плата → ICM_SPI_Eot_SPIx
 *   CS поднимается ТОЛЬКО после EOT, не после DMA TC.
 *
 * Таймеры:
 *   TIM6 UPDATE → ICM_StartBurstRead()  (~3.125 мс = 320 Гц)
 *   TIM7 UPDATE → ICM_WatchdogTick()    (watchdog stuck-DMA/EOT, ~1 кГц)
 *                 NVIC приоритет TIM7 НИЖЕ SPI/DMA IRQ!
 * =============================================================================
 */

#include "main.h"
#include "stm32h7xx_it.h"
#include "icm45686_spi.h"
#include "uart_telemetry.h"

/* ===========================================================================
 *  Системные fault-handlers
 * ========================================================================== */

void NMI_Handler(void)      { while (1) {} }
void HardFault_Handler(void){ while (1) {} }
void MemManage_Handler(void){ while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void){ while (1) {} }
void SVC_Handler(void)      {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void)   {}
void SysTick_Handler(void)  {}

/* ===========================================================================
 *  USART1 TX DMA: DMA1 Stream1
 *
 *  При TC: UART_DMA_TxComplete() освобождает слот TX ring-buffer и при
 *  наличии следующего пакета сам запускает очередную DMA-передачу.
 *  При TE/DME/FE: флаг очищается, счётчик инкрементируется.
 * ========================================================================== */
void DMA1_Stream1_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE1(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TE1(DMA1);
        g_uart_dma_te_count++;
    }
    if (LL_DMA_IsActiveFlag_DME1(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_DME1(DMA1);
        g_uart_dma_dme_count++;
    }
    if (LL_DMA_IsActiveFlag_FE1(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_FE1(DMA1);
        g_uart_dma_fe_count++;
    }
    if (LL_DMA_IsActiveFlag_TC1(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TC1(DMA1);
        UART_DMA_TxComplete();
    }
}

/* ===========================================================================
 *  НИЖНЯЯ ПЛАТА — DMA1 Stream2-3 (SPI1) / DMA2 Stream0-3 (SPI4, SPI5)
 * ========================================================================== */

/* SPI1 RX: DMA1 Stream2 */
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

/* SPI1 TX: DMA1 Stream3 — только сброс флагов */
void DMA1_Stream3_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE3(DMA1) != 0U) { LL_DMA_ClearFlag_TE3(DMA1); }
    if (LL_DMA_IsActiveFlag_TC3(DMA1) != 0U) { LL_DMA_ClearFlag_TC3(DMA1); }
}

/* SPI4 RX: DMA2 Stream0 */
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

/* SPI4 TX: DMA2 Stream1 — только сброс флагов */
void DMA2_Stream1_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE1(DMA2) != 0U) { LL_DMA_ClearFlag_TE1(DMA2); }
    if (LL_DMA_IsActiveFlag_TC1(DMA2) != 0U) { LL_DMA_ClearFlag_TC1(DMA2); }
}

/* SPI5 RX: DMA2 Stream2 */
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

/* SPI5 TX: DMA2 Stream3 — только сброс флагов */
void DMA2_Stream3_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE3(DMA2) != 0U) { LL_DMA_ClearFlag_TE3(DMA2); }
    if (LL_DMA_IsActiveFlag_TC3(DMA2) != 0U) { LL_DMA_ClearFlag_TC3(DMA2); }
}

/* ===========================================================================
 *  ВЕРХНЯЯ ПЛАТА — DMA1 Stream4-7 (SPI2, SPI3) + BDMA Ch0-1 (SPI6)
 * ========================================================================== */

/* USART1 RX: DMA1 Stream0 — не используется */
void DMA1_Stream0_IRQHandler(void) {}

/* SPI2 RX: DMA1 Stream4 */
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

/* SPI2 TX: DMA1 Stream5 — только сброс флагов */
void DMA1_Stream5_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE5(DMA1) != 0U) { LL_DMA_ClearFlag_TE5(DMA1); }
    if (LL_DMA_IsActiveFlag_TC5(DMA1) != 0U) { LL_DMA_ClearFlag_TC5(DMA1); }
}

/* SPI3 RX: DMA1 Stream6 */
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

/* SPI3 TX: DMA1 Stream7 — только сброс флагов */
void DMA1_Stream7_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE7(DMA1) != 0U) { LL_DMA_ClearFlag_TE7(DMA1); }
    if (LL_DMA_IsActiveFlag_TC7(DMA1) != 0U) { LL_DMA_ClearFlag_TC7(DMA1); }
}

/* SPI6 RX: BDMA Channel0 (буфер обязан лежать в SRAM4 / .RAM_D3) */
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

/* SPI6 TX: BDMA Channel1 — только сброс флагов */
void BDMA_Channel1_IRQHandler(void)
{
    if (LL_BDMA_IsActiveFlag_TE1(BDMA) != 0U) { LL_BDMA_ClearFlag_TE1(BDMA); }
    if (LL_BDMA_IsActiveFlag_TC1(BDMA) != 0U) { LL_BDMA_ClearFlag_TC1(BDMA); }
}

/* ===========================================================================
 *  SPIx EOT — нижняя плата (SPI1, SPI4, SPI5)
 *
 *  Архитектура: DMA RX TC → ICM_OnDmaRxComplete (выкл DMA, вкл EOT IRQ,
 *  CS остаётся LOW) → SPI EOT → ICM_OnSpiEot (CS HIGH, SPE=0, next sensor).
 *  Приоритет NVIC SPI IRQ НЕ ниже DMA IRQ, иначе EOT теряется.
 *  OVR сбрасываем, чтобы не зациклить IRQ.
 * ========================================================================== */
void SPI1_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI1) != 0U) &&
        (LL_SPI_IsActiveFlag_EOT(SPI1) != 0U))
    {
        ICM_SPI_Eot_SPI1();
    }
    if (LL_SPI_IsActiveFlag_OVR(SPI1) != 0U) { LL_SPI_ClearFlag_OVR(SPI1); }
}

void SPI4_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI4) != 0U) &&
        (LL_SPI_IsActiveFlag_EOT(SPI4) != 0U))
    {
        ICM_SPI_Eot_SPI4();
    }
    if (LL_SPI_IsActiveFlag_OVR(SPI4) != 0U) { LL_SPI_ClearFlag_OVR(SPI4); }
}

void SPI5_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI5) != 0U) &&
        (LL_SPI_IsActiveFlag_EOT(SPI5) != 0U))
    {
        ICM_SPI_Eot_SPI5();
    }
    if (LL_SPI_IsActiveFlag_OVR(SPI5) != 0U) { LL_SPI_ClearFlag_OVR(SPI5); }
}

/* ===========================================================================
 *  SPIx EOT — верхняя плата (SPI2, SPI3, SPI6)
 * ========================================================================== */
void SPI2_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI2) != 0U) &&
        (LL_SPI_IsActiveFlag_EOT(SPI2) != 0U))
    {
        ICM_SPI_Eot_SPI2();
    }
    if (LL_SPI_IsActiveFlag_OVR(SPI2) != 0U) { LL_SPI_ClearFlag_OVR(SPI2); }
}

void SPI3_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI3) != 0U) &&
        (LL_SPI_IsActiveFlag_EOT(SPI3) != 0U))
    {
        ICM_SPI_Eot_SPI3();
    }
    if (LL_SPI_IsActiveFlag_OVR(SPI3) != 0U) { LL_SPI_ClearFlag_OVR(SPI3); }
}

void SPI6_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI6) != 0U) &&
        (LL_SPI_IsActiveFlag_EOT(SPI6) != 0U))
    {
        ICM_SPI_Eot_SPI6();
    }
    if (LL_SPI_IsActiveFlag_OVR(SPI6) != 0U) { LL_SPI_ClearFlag_OVR(SPI6); }
}

/* ===========================================================================
 *  TIM6 UPDATE — запуск DMA-цикла опроса всех 6 шин параллельно
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
 *  TIM7 UPDATE — watchdog tick для обнаружения stuck DMA / EOT
 *
 *  Рекомендуемая частота ~1 кГц (PSC=274, ARR=999 @ APB1 275 МГц).
 *  ВАЖНО: приоритет NVIC TIM7 НИЖЕ приоритета всех SPI/DMA IRQ —
 *  watchdog не должен прерывать штатную acquisition-транзакцию.
 *  ICM_WatchdogTick() вызывает ICM_RecoverBus() при таймауте 600 мкс.
 * ========================================================================== */
void TIM7_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM7) != 0U)
    {
        LL_TIM_ClearFlag_UPDATE(TIM7);
        ICM_WatchdogTick();
    }
}

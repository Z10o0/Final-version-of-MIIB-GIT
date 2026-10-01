/* =============================================================================
 * stm32h7xx_it.c
 *
 * Обработчики прерываний MIIB:
 * 36 ICM-45686, SPI1/5/4 + SPI3/2/6 + USART1 DMA TX.
 *
 * DMA-карта:
 *   DMA1 Stream0  — USART1 RX
 *   DMA1 Stream1  — USART1 TX
 *   DMA1 Stream2  — SPI1 RX
 *   DMA1 Stream3  — SPI1 TX
 *   DMA1 Stream4  — SPI2 RX
 *   DMA1 Stream5  — SPI2 TX
 *   DMA1 Stream6  — SPI3 RX
 *   DMA1 Stream7  — SPI3 TX
 *   DMA2 Stream0  — SPI4 RX
 *   DMA2 Stream1  — SPI4 TX
 *   DMA2 Stream2  — SPI5 RX
 *   DMA2 Stream3  — SPI5 TX
 *   BDMA Channel0 — SPI6 RX, D3/SRAM4
 *   BDMA Channel1 — SPI6 TX, D3/SRAM4
 *
 * Таймеры:
 *   TIM6 UPDATE — acquisition trigger 400 Гц, период 2,5 мс.
 *   TIM7 UPDATE — watchdog stuck DMA/EOT, 1 кГц.
 * =============================================================================
 */

#include "main.h"
#include "stm32h7xx_it.h"
#include "icm45686_spi.h"
#include "uart_telemetry.h"

/* ============================================================================
 * System exception handlers
 * ========================================================================== */

void NMI_Handler(void)
{
    while (1)
    {
    }
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
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

/* ============================================================================
 * USART1 TX — DMA1 Stream1
 * ========================================================================== */

void DMA1_Stream1_IRQHandler(void)
{
    /*
     * FIFO mode у DMA1 Stream1 отключён. Возможный FE очищается,
     * но не учитывается как реальная ошибка передачи.
     */
    if (LL_DMA_IsActiveFlag_FE1(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_FE1(DMA1);
    }

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

    if (LL_DMA_IsActiveFlag_TC1(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TC1(DMA1);
        UART_DMA_TxComplete();
    }
}

/* ============================================================================
 * SPI1 — DMA1 Stream2 RX, Stream3 TX
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

/* ============================================================================
 * SPI4 — DMA2 Stream0 RX, Stream1 TX
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

/* ============================================================================
 * SPI5 — DMA2 Stream2 RX, Stream3 TX
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

/* ============================================================================
 * USART1 RX — DMA1 Stream0
 * ========================================================================== */

void DMA1_Stream0_IRQHandler(void)
{
}

/* ============================================================================
 * SPI2 — DMA1 Stream4 RX, Stream5 TX
 * ========================================================================== */

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
    if (LL_DMA_IsActiveFlag_TE5(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TE5(DMA1);
    }

    if (LL_DMA_IsActiveFlag_TC5(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TC5(DMA1);
    }
}

/* ============================================================================
 * SPI3 — DMA1 Stream6 RX, Stream7 TX
 * ========================================================================== */

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
    if (LL_DMA_IsActiveFlag_TE7(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TE7(DMA1);
    }

    if (LL_DMA_IsActiveFlag_TC7(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TC7(DMA1);
    }
}

/* ============================================================================
 * SPI6 — BDMA Channel0 RX, Channel1 TX
 * ========================================================================== */

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
    if (LL_BDMA_IsActiveFlag_TE1(BDMA) != 0U)
    {
        LL_BDMA_ClearFlag_TE1(BDMA);
    }

    if (LL_BDMA_IsActiveFlag_TC1(BDMA) != 0U)
    {
        LL_BDMA_ClearFlag_TC1(BDMA);
    }
}

/* ============================================================================
 * SPI EOT handlers
 *
 * DMA RX TC завершает работу DMA, но CS остаётся LOW.
 * CS поднимается только после фактического SPI EOT.
 * ========================================================================== */

void SPI1_IRQHandler(void)
{
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

void SPI2_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI2) != 0U) &&
        (LL_SPI_IsActiveFlag_EOT(SPI2) != 0U))
    {
        ICM_SPI_Eot_SPI2();
    }

    if (LL_SPI_IsActiveFlag_OVR(SPI2) != 0U)
    {
        LL_SPI_ClearFlag_OVR(SPI2);
    }
}

void SPI3_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI3) != 0U) &&
        (LL_SPI_IsActiveFlag_EOT(SPI3) != 0U))
    {
        ICM_SPI_Eot_SPI3();
    }

    if (LL_SPI_IsActiveFlag_OVR(SPI3) != 0U)
    {
        LL_SPI_ClearFlag_OVR(SPI3);
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

void SPI6_IRQHandler(void)
{
    if ((LL_SPI_IsEnabledIT_EOT(SPI6) != 0U) &&
        (LL_SPI_IsActiveFlag_EOT(SPI6) != 0U))
    {
        ICM_SPI_Eot_SPI6();
    }

    if (LL_SPI_IsActiveFlag_OVR(SPI6) != 0U)
    {
        LL_SPI_ClearFlag_OVR(SPI6);
    }
}

/* ============================================================================
 * TIM6 — acquisition trigger
 *
 * TIM6CLK = 275 МГц:
 * PSC=274, ARR=2499 -> 400 Гц -> период 2,5 мс.
 *
 * Один tick запускает чтение восьми FIFO-пакетов каждого датчика.
 * Парсинг, усреднение и UART выполняются в main loop.
 * ========================================================================== */

void TIM6_DAC_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6) != 0U)
    {
        LL_TIM_ClearFlag_UPDATE(TIM6);
        ICM_StartBurstRead();
    }
}

/* ============================================================================
 * TIM7 — watchdog stuck DMA/EOT
 *
 * TIM7CLK = 275 МГц:
 * PSC=274, ARR=999 -> 1 кГц.
 *
 * Приоритет TIM7 ниже приоритета SPI/DMA/TIM6.
 * ========================================================================== */

void TIM7_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM7) != 0U)
    {
        LL_TIM_ClearFlag_UPDATE(TIM7);
        ICM_WatchdogTick();
    }
}

/*
 * MIIB: 36 ICM-45686.
 *
 * DMA1:
 * Stream0: USART1 RX, не используется.
 * Stream1: USART1 TX.
 * Stream2: SPI1 RX.
 * Stream3: SPI1 TX.
 * Stream4: SPI2 RX.
 * Stream5: SPI2 TX.
 * Stream6: SPI3 RX.
 * Stream7: SPI3 TX.
 *
 * DMA2:
 * Stream0: SPI4 RX.
 * Stream1: SPI4 TX.
 * Stream2: SPI5 RX.
 * Stream3: SPI5 TX.
 *
 * BDMA:
 * Channel0: SPI6 RX.
 * Channel1: SPI6 TX.
 *
 * TIM6:
 * 2.5 мс = 400 Гц = ODR 3200 / 8 raw FIFO-пакетов.
 * ISR только запускает acquisition.
 *
 * TIM7:
 * watchdog 1 кГц.
 * NVIC priority ниже SPI/DMA/TIM6.
 *
 * Парсинг, усреднение и построение RS-кадра выполняются в main loop.
 */

#include "main.h"
#include "stm32h7xx_it.h"

#include "icm45686_spi.h"
#include "uart_telemetry.h"

/* ============================================================================
 * System handlers
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
 * USART1 TX: DMA1 Stream1
 * ========================================================================== */

void DMA1_Stream1_IRQHandler(void)
{
    /*
     * FE не считается ошибкой для текущего Direct-mode stream.
     * Флаг очищается, IT_FE не включается в UART_Telemetry_Init().
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
 * Нижняя плата: SPI1 / SPI4 / SPI5
 * ========================================================================== */

/* SPI1 RX. */
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

/* SPI1 TX: только сброс флагов. */
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

/* SPI4 RX. */
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

/* SPI4 TX: только сброс флагов. */
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

/* SPI5 RX. */
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

/* SPI5 TX: только сброс флагов. */
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
 * Верхняя плата: SPI2 / SPI3 / SPI6
 * ========================================================================== */

/* USART1 RX: не используется. */
void DMA1_Stream0_IRQHandler(void)
{
}

/* SPI2 RX. */
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

/* SPI2 TX: только сброс флагов. */
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

/* SPI3 RX. */
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

/* SPI3 TX: только сброс флагов. */
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

/* SPI6 RX: BDMA, буфер находится в D3/SRAM4. */
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

/* SPI6 TX: только сброс флагов. */
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
 * TIM6: acquisition 400 Гц
 *
 * PSC=274, ARR=2499 при TIM6CLK=275 МГц.
 * Период 2.5 мс.
 *
 * Парсер, усреднение и построение RS-кадра здесь не выполняются.
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
 * TIM7: watchdog 1 кГц
 *
 * PSC=274, ARR=999 при TIM7CLK=275 МГц.
 * NVIC priority ниже SPI/DMA/TIM6.
 * ========================================================================== */

void TIM7_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM7) != 0U)
    {
        LL_TIM_ClearFlag_UPDATE(TIM7);
        ICM_WatchdogTick();
    }
}

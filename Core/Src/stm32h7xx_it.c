#include "main.h"
#include "stm32h7xx_it.h"
#include "icm45686_spi.h"
#include "uart_telemetry.h"

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

/* UART1 TX DMA: DMA1 Stream1 */
void DMA1_Stream1_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE1(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TE1(DMA1);
    }

    if (LL_DMA_IsActiveFlag_TC1(DMA1) != 0U)
    {
        LL_DMA_ClearFlag_TC1(DMA1);
        UART_DMA_TxComplete();
    }
}

/* SPI1 RX DMA: DMA1 Stream2 */
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

/* SPI1 TX DMA: DMA1 Stream3, TC не используется */
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

/*
 * TIM6 UPDATE.
 * CubeMX должен настроить частоту UPDATE = ICM_POLL_RATE_HZ = 320 Гц
 * для ODR=3200 Гц и ICM_FIFO_POLL_PACKETS=10.
 */
void TIM6_DAC_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6) != 0U)
    {
        LL_TIM_ClearFlag_UPDATE(TIM6);
        ICM_StartBurstRead();
    }
}

/* SPI4 RX DMA: DMA2 Stream0 */
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

/* SPI4 TX DMA: DMA2 Stream1, TC не используется */
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

/* SPI5 RX DMA: DMA2 Stream2 */
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

/* SPI5 TX DMA: DMA2 Stream3, TC не используется */
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

void SPI1_IRQHandler(void)
{
}

void SPI2_IRQHandler(void)
{
}

void SPI3_IRQHandler(void)
{
}

void SPI4_IRQHandler(void)
{
}

void SPI5_IRQHandler(void)
{
}

void SPI6_IRQHandler(void)
{
}

void DMA1_Stream0_IRQHandler(void)
{
}

void DMA1_Stream4_IRQHandler(void)
{
}

void DMA1_Stream5_IRQHandler(void)
{
}

void DMA1_Stream6_IRQHandler(void)
{
}

void DMA1_Stream7_IRQHandler(void)
{
}

void BDMA_Channel0_IRQHandler(void)
{
}

void BDMA_Channel1_IRQHandler(void)
{
}

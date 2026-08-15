/*
******************************************************************************
** File    : STM32H723ZGTX_FLASH.ld
** Target  : STM32H723ZGTx  (1024K FLASH, 560K RAM total)
******************************************************************************
*/

ENTRY(Reset_Handler)

/* Стек — конец AXI SRAM D1 (растёт вниз) */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);

_Min_Heap_Size  = 0x200;   /* 512  байт — куча  */
_Min_Stack_Size = 0x800;   /* 2048 байт — стек  */

/* ============================================================
 * КАРТА ПАМЯТИ STM32H723ZGT6
 * ============================================================
 *
 *  Регион    Адрес        Размер  Доступ DMA         Назначение
 *  --------  -----------  ------  -----------------  --------------------------
 *  ITCMRAM   0x00000000    64K    — (только ядро)    код критичных ISR
 *  DTCMRAM   0x20000000   128K    — (только ядро)    быстрые локальные данные
 *  FLASH     0x08000000  1024K    —                  программный код (RO)
 *  RAM_D1    0x24000000   320K    DMA1, DMA2          .data / .bss / стек
 *                                                     + .RAM_D1_DMA (UART TX)
 *  RAM_D2    0x30000000    32K    DMA1, DMA2          .RAM_D2  (SPI1-SPI5 DMA)
 *  RAM_D3    0x38000000    16K    BDMA only           .RAM_D3  (SPI6 BDMA)
 *
 * ============================================================
 *
 * РАСПРЕДЕЛЕНИЕ .RAM_D2 (32K итого):
 *   g_fifo_data[5][6][N]   RX-буферы SPI1-SPI5        ~21 504 байт
 *   g_tx_spi1..g_tx_spi5   TX-шаблоны (5 × N байт)    ~  1 280 байт
 *   выравнивания aligned(32)                           ~    224 байт
 *   Итого .RAM_D2:                                     ~23 008 байт  < 32K ✓
 *
 * РАСПРЕДЕЛЕНИЕ .RAM_D1_DMA (в RAM_D1):
 *   g_uart_tx_queue[16][690]  UART TX ring-buffer       11 040 байт
 *   DMA1 Stream1 видит RAM_D1 напрямую — ограничений нет.
 *
 * ПРИЧИНА ПЕРЕНОСА:
 *   Было: g_uart_tx_queue в .RAM_D2 → overflow +2656 байт.
 *   Стало: g_uart_tx_queue в .RAM_D1_DMA → RAM_D2 свободна, RAM_D1 > 200K.
 *
 * ============================================================ */

MEMORY
{
    ITCMRAM (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
    DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
    FLASH    (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM_D1  (xrw) : ORIGIN = 0x24000000, LENGTH = 320K
    RAM_D2  (xrw) : ORIGIN = 0x30000000, LENGTH = 32K
    RAM_D3  (xrw) : ORIGIN = 0x38000000, LENGTH = 16K
}

SECTIONS
{
    /* ── Таблица векторов прерываний ── */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* ── Программный код ── */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP (*(.init))
        KEEP (*(.fini))
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    /* ── Константы и строки (read-only) ── */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    /* ── C++ exception tables ── */
    .ARM.extab (READONLY) :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM (READONLY) :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    /* ── Таблицы конструкторов/деструкторов C++ ── */
    .preinit_array (READONLY) :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >FLASH

    .init_array (READONLY) :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >FLASH

    .fini_array (READONLY) :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >FLASH

    /* ── Инициализированные данные: LMA во FLASH, VMA в RAM_D1 ── */
    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        *(.RamFunc)
        *(.RamFunc*)
        . = ALIGN(4);
        _edata = .;
    } >RAM_D1 AT> FLASH

    /* ── Неинициализированные данные: обнуляются стартапом ──
     * DMA-буферы сюда НЕ попадают — они в .RAM_D1_DMA / .RAM_D2 / .RAM_D3 */
    . = ALIGN(4);
    .bss :
    {
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >RAM_D1

    /* ── Куча и стек ── */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE ( end  = . );
        PROVIDE ( _end = . );
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >RAM_D1

    /* ══════════════════════════════════════════════════════════
     * .RAM_D1_DMA — DMA-буферы в AXI SRAM (RAM_D1), доступные
     *               DMA1 и DMA2 без ограничений.
     *
     * NOLOAD: не загружается из FLASH, стартап не трогает.
     * Инициализация — явным memset() в UART_Telemetry_Init().
     *
     * Переменные с __attribute__((section(".RAM_D1_DMA"), aligned(32))):
     *   g_uart_tx_queue[16][690] — UART TX ring-buffer
     *                              16 × 690 = 11 040 байт
     *                              DMA1 Stream1 (USART1 TX) видит RAM_D1
     *
     * Причина переноса из .RAM_D2:
     *   RAM_D2 = 32K; SPI DMA-буферы занимают ~23K → overflow +2656 байт.
     *   RAM_D1 = 320K; после .data/.bss/стека свободно > 200K.
     * ══════════════════════════════════════════════════════════ */
    .RAM_D1_DMA (NOLOAD) :
    {
        . = ALIGN(32);
        *(.RAM_D1_DMA)
        *(.RAM_D1_DMA*)
        . = ALIGN(32);
    } >RAM_D1

    /* ══════════════════════════════════════════════════════════
     * .RAM_D2 — DMA RX/TX буферы для SPI1, SPI2, SPI3, SPI4, SPI5
     *
     * NOLOAD: не загружается из FLASH, стартап не трогает.
     * ICM_BusesInit() инициализирует их явно через memset().
     *
     * Переменные с __attribute__((section(".RAM_D2"), aligned(32))):
     *   g_fifo_data[5][6][N]  — RX-буферы (5 шин × 6 датчиков)
     *   g_tx_spi1..g_tx_spi5  — TX-шаблоны команды FIFO_DATA|READ
     *
     * aligned(32) обязательно для корректной работы D-кэша
     * при вызове SCB_InvalidateDCache_by_Addr().
     * ══════════════════════════════════════════════════════════ */
    .RAM_D2 (NOLOAD) :
    {
        . = ALIGN(32);
        *(.RAM_D2)
        *(.RAM_D2*)
        . = ALIGN(32);
    } >RAM_D2

    /* ══════════════════════════════════════════════════════════
     * .RAM_D3 — BDMA буферы ТОЛЬКО для SPI6
     *
     * КРИТИЧНО: BDMA обслуживает исключительно домен D3 и
     * физически не имеет доступа к RAM_D1 или RAM_D2.
     * Буферы SPI6 ОБЯЗАНЫ лежать здесь, иначе BDMA будет
     * читать/писать неверные адреса без ошибки шины.
     *
     * Переменные с __attribute__((section(".RAM_D3"), aligned(32))):
     *   g_fifo_data_spi6[6][N] — RX-буфер (6 датчиков × N байт)
     *   g_tx_spi6              — TX-шаблон команды FIFO_DATA|READ
     * ══════════════════════════════════════════════════════════ */
    .RAM_D3 (NOLOAD) :
    {
        . = ALIGN(32);
        *(.RAM_D3)
        *(.RAM_D3*)
        . = ALIGN(32);
    } >RAM_D3

    /* ── Удалить отладочную информацию стандартных библиотек ── */
    /DISCARD/ :
    {
        libc.a   ( * )
        libm.a   ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
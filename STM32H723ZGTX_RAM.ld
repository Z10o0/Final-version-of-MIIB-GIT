/*
******************************************************************************
** File    : STM32H723ZGTX_RAM-2.ld  (Debug in RAM)
** Target  : STM32H723ZGTx
**
** Весь код загружается в RAM_EXEC (AXI SRAM) через отладчик.
** Flash не используется — прошивка не сохраняется после сброса питания.
******************************************************************************
*/

ENTRY(Reset_Handler)

/* Стек — конец DTCMRAM */
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);

_Min_Heap_Size  = 0x200;   /* 512  байт — куча  */
_Min_Stack_Size = 0x400;   /* 1024 байт — стек  */

MEMORY
{
    RAM_EXEC (xrw) : ORIGIN = 0x24000000, LENGTH = 320K
    DTCMRAM  (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
    ITCMRAM  (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
    RAM_D2   (xrw) : ORIGIN = 0x30000000, LENGTH = 32K
    RAM_D3   (xrw) : ORIGIN = 0x38000000, LENGTH = 16K
}

SECTIONS
{
    /* ── Таблица векторов прерываний — первой в RAM_EXEC ── */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >RAM_EXEC

    /* ── Программный код ── */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        *(.RamFunc)    /* уже в RAM — копировать не нужно */
        *(.RamFunc*)
        KEEP (*(.init))
        KEEP (*(.fini))
        . = ALIGN(4);
        _etext = .;
    } >RAM_EXEC

    /* ── Константы и строки ── */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >RAM_EXEC

    /* ── C++ exception tables ── */
    .ARM.extab (READONLY) :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >RAM_EXEC

    .ARM (READONLY) :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >RAM_EXEC

    /* ── Таблицы конструкторов/деструкторов C++ ── */
    .preinit_array (READONLY) :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >RAM_EXEC

    .init_array (READONLY) :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >RAM_EXEC

    .fini_array (READONLY) :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >RAM_EXEC

    /* ── Инициализированные данные: LMA в RAM_EXEC, VMA в DTCMRAM ── */
    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >DTCMRAM AT> RAM_EXEC

    /* ── Неинициализированные данные ── */
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
    } >DTCMRAM

    /* ── Куча и стек ── */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE ( end  = . );
        PROVIDE ( _end = . );
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >DTCMRAM

    /* ══════════════════════════════════════════════════════════
     * .RAM_D2 — DMA RX/TX буферы SPI1..SPI5
     * NOLOAD: отладчик не загружает, ICM_BusesInit() инициализирует
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
     * КРИТИЧНО: BDMA видит только домен D3 (SRAM4, 0x38000000)
     * ══════════════════════════════════════════════════════════ */
    .RAM_D3 (NOLOAD) :
    {
        . = ALIGN(32);
        *(.RAM_D3)
        *(.RAM_D3*)
        . = ALIGN(32);
    } >RAM_D3

    /DISCARD/ :
    {
        libc.a   ( * )
        libm.a   ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
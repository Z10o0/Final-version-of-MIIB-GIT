/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7 series
**                1024Kbytes FLASH and 560Kbytes RAM
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** Copyright (c) 2025 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
****************************************************************************
*/

/* Точка входа — символ Reset_Handler из startup_stm32h723zgtx.s */
ENTRY(Reset_Handler)

/* Вершина стека = конец AXI SRAM D1.
 * ВАЖНО: стек растёт вниз, поэтому _estack = последний байт RAM_D1.
 * DMA-буферы размещены в конце RAM_D1 через .RAM_D1_noinit (NOLOAD),
 * поэтому стек не пересекается с ними — линкер выдаст ошибку если не хватит места. */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);

/* Минимальные размеры кучи и стека.
 * Линкер выдаст ошибку если ._user_heap_stack не поместится в RAM_D1. */
_Min_Heap_Size  = 0x200;   /* 512 байт  — куча  */
_Min_Stack_Size = 0x800;   /* 2048 байт — стек (увеличен: ISR-стек для 3 DMA ISR одновременно) */

/* ============================================================
 * КАРТА ПАМЯТИ STM32H723ZGT6
 * ============================================================
 *
 *  ITCMRAM  0x00000000  64K   — только ядро (код критичных ISR)
 *  DTCMRAM  0x20000000  128K  — только ядро (стек по умолчанию у HAL)
 *                               !! DMA1/DMA2 НЕ видят DTCM !!
 *  FLASH    0x08000000  1024K — программный код (RO)
 *  RAM_D1   0x24000000  320K  — AXI SRAM: .data, .bss, стек, DMA-буферы
 *                               DMA1 и DMA2 имеют полный доступ
 *  RAM_D2   0x30000000  32K   — SRAM D2 (AHB): резерв / BDMA
 *  RAM_D3   0x38000000  16K   — SRAM D3 (APB4): резерв / BDMA
 *
 * В данном проекте:
 *   .data, .bss, стек, куча → RAM_D1 (основная рабочая SRAM)
 *   .RAM_D1_noinit           → RAM_D1 (DMA-буферы без инициализации)
 *   Код                      → FLASH
 * ============================================================ */
MEMORY
{
  ITCMRAM (xrw)  : ORIGIN = 0x00000000, LENGTH = 64K
  DTCMRAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH    (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM_D1   (xrw) : ORIGIN = 0x24000000, LENGTH = 320K
  RAM_D2   (xrw) : ORIGIN = 0x30000000, LENGTH = 32K
  RAM_D3   (xrw) : ORIGIN = 0x38000000, LENGTH = 16K
}

/* ============================================================
 * СЕКЦИИ ВЫВОДА
 * ============================================================ */
SECTIONS
{
  /* ── Таблица векторов прерываний — первая в FLASH ── */
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

  /* ── Константы и строки (read-only data) ── */
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

  /* ── Инициализированные данные: хранятся в FLASH, копируются в RAM_D1 при старте ──
   * startup_stm32h723zgtx.s копирует [_sidata .. _sidata+(_edata-_sdata)] → [_sdata.._edata]
   * AT> FLASH означает: LMA (load address) в FLASH, VMA (runtime address) в RAM_D1 */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)    /* Функции, явно размещённые в RAM для быстрого выполнения */
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >RAM_D1 AT> FLASH

  /* ── Неинициализированные данные: обнуляются startup при старте ──
   * startup_stm32h723zgtx.s обнуляет [_sbss.._ebss].
   * Сюда попадают global/static переменные без явного начального значения.
   *
   * ВАЖНО: сюда НЕ должны попасть DMA-буферы (g_fifo_data, g_bus_spi*)!
   * Они размещаются в .RAM_D1_noinit ниже — startup их НЕ трогает. */
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

  /* ── Куча и стек: проверка что хватает места ── */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end  = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_D1

  /* ══════════════════════════════════════════════════════════════
   * .RAM_D1_noinit — DMA-буферы в AXI SRAM D1
   * ══════════════════════════════════════════════════════════════
   *
   * NOLOAD: секция НЕ загружается из FLASH и НЕ обнуляется startup.
   * Переменные в этой секции содержат мусор после сброса — это нормально:
   *   ICM_BusesInit() явно обнуляет g_fifo_data через memset().
   *
   * Сюда помещаются:
   *   g_fifo_data   — DMA RX-буферы FIFO (3 шины × 6 датчиков × N байт)
   *   g_bus_spi1    — дескриптор шины SPI1 (содержит tx_buf для DMA TX)
   *   g_bus_spi5    — дескриптор шины SPI5
   *   g_bus_spi4    — дескриптор шины SPI4
   *
   * aligned(32) — кратно размеру cache line Cortex-M7.
   * Это обязательно если в будущем включите D-кэш (SCB_EnableDCache):
   * без выравнивания SCB_InvalidateDCache_by_Addr() будет работать некорректно.
   *
   * Расположение в памяти:
   *   0x24000000  ← начало RAM_D1
   *   [.data]     ← инициализированные переменные
   *   [.bss]      ← обнуляемые переменные
   *   [heap+stack]
   *   [.RAM_D1_noinit] ← DMA-буферы (в конце RAM_D1)
   *   0x24050000  ← конец RAM_D1 (320K = 0x50000 байт)
   * ══════════════════════════════════════════════════════════════ */
.RAM_D1 (NOLOAD) :
{
  . = ALIGN(32);
  *(.RAM_D1)
  *(.RAM_D1*)
  . = ALIGN(32);
} >RAM_D1

  /* ── Удалить отладочную информацию стандартных библиотек ── */
  /DISCARD/ :
  {
    libc.a   ( * )
    libm.a   ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
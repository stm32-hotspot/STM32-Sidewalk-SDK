/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32WBA6xxG Device from STM32WBA series
**                      1024Kbytes FLASH
**                      256Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
******************************************************************************
** @attention
**
** Copyright (c) 2022-2025 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
******************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

_Min_Heap_Size    = 0x0000;               /* required amount of heap  */
_Stack_Alignment  = 8;                    /* Should equal to the stack frame size */
_Stack_Guard_Size = 4 * _Stack_Alignment; /* Safety gap between the stack and other regions */
_Stack_Size       = 0x700;                /* required amount of stack */

/* Memories definition */
MEMORY
{
  RAM1        (xrw) : ORIGIN = 0x20000000,          LENGTH = 192K
  RAM2        (xrw) : ORIGIN = 0x20070000,          LENGTH = 64K
  FLASH       (xr ) : ORIGIN = 0x08000000,          LENGTH = 936K
  MFG_DATA    ( r ) : ORIGIN = 0x080EA000,          LENGTH = 8K
  LITTLE_FS   ( rw) : ORIGIN = 0x080EC000,          LENGTH = 64K
  SNVMA_ARBTR ( rw) : ORIGIN = 0x080FC000,          LENGTH = 16K
}

__FLASH_region_start__       = ORIGIN(FLASH);
__FLASH_region_end__         = (ORIGIN(FLASH) + LENGTH(FLASH));
__FLASH_region_size__        = LENGTH(FLASH);

__MFG_DATA_region_start__    = ORIGIN(MFG_DATA);
__MFG_DATA_region_end__      = (ORIGIN(MFG_DATA) + LENGTH(MFG_DATA));
__MFG_DATA_region_size__     = LENGTH(MFG_DATA);

__LITTLE_FS_region_start__   = ORIGIN(LITTLE_FS);
__LITTLE_FS_region_end__     = (ORIGIN(LITTLE_FS) + LENGTH(LITTLE_FS));
__LITTLE_FS_region_size__    = LENGTH(LITTLE_FS);

__SNVMA_ARBTR_region_start__ = ORIGIN(SNVMA_ARBTR);
__SNVMA_ARBTR_region_end__   = (ORIGIN(SNVMA_ARBTR) + LENGTH(SNVMA_ARBTR));
__SNVMA_ARBTR_region_size__  = LENGTH(SNVMA_ARBTR);

/* Sections */
SECTIONS
{
  /* The startup code into "FLASH" Rom type memory */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data into "FLASH" Rom type memory */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data into "FLASH" Rom type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab (READONLY) : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM (READONLY) : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array (READONLY) : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array (READONLY) : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array (READONLY) : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* Put user mode stack at the very beginning of RAM, this ensures that stack overflow will immediately trigger hard fault due to access to non-implemented memory space */
  .stack ORIGIN(RAM1) :
  {
    . = ALIGN(_Stack_Alignment);
    PROVIDE ( __start_STACK = . );
    . = . + _Stack_Size;
    . = ALIGN(_Stack_Alignment);
    PROVIDE ( _estack = . ); /* Highest address of the user mode stack */
    PROVIDE ( __end_STACK = . );
  } >RAM1

  /* Add guard area for user mode stack to allow stack underflow detection */
  .stack_guard _estack (NOLOAD) :
  {
    PROVIDE (__start_STACK_GUARD = . );
    . = . + _Stack_Guard_Size;
    . = ALIGN(_Stack_Alignment);
    PROVIDE ( _bootstack = . ); /* Highest address of the Reset Handler stack */
    PROVIDE (__end_STACK_GUARD = . );
  } >RAM1

  /* Non-initialized RAM - data here will survive MCU resets */
  . = ALIGN(4);
  .no_init (NOLOAD) :
  {
    . = ALIGN(4);
  } >RAM1

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */

    . = ALIGN(4);
    .dma_lln = .;
    PROVIDE (__start_DMA_LLN = . );
    *(.dma_lln)
    *(.dma_lln*)
    . = ALIGN(4);
    PROVIDE (__end_DMA_LLN = . );

    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */

  } >RAM1 AT> FLASH

  /* Uninitialized data section into "RAM" Ram type memory */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM1

  /* User_heap section, used to check that there is enough "RAM" Ram  type memory left */
  ._user_heap (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( __start_HEAP = . );
    . = . + _Min_Heap_Size;
    . = ALIGN(8);
    PROVIDE ( __end_HEAP = ORIGIN(RAM1) + LENGTH(RAM1) );
  } >RAM1

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

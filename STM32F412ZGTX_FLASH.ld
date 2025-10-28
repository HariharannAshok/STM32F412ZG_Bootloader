/* -------------------------------------------------------------------------
 * STM32F412ZG User Application Linker Script
 * App base: 0x08008000 (after 32 KB bootloader)
 * Flash size: 960 KB (0xF0000)
 * RAM size : 256 KB (0x40000)
 * ------------------------------------------------------------------------- */

/* Entry Point ------------------------------------------------------------ */
ENTRY(Reset_Handler)

/* Define top of stack (end of RAM) */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Minimal heap / stack sizes */
_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

/* -------------------------------------------------------------------------
 * Memory Regions
 * ------------------------------------------------------------------------- */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08008000, LENGTH = 960K   /* sectors 2–11 */
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 256K
}

/* -------------------------------------------------------------------------
 * Sections
 * ------------------------------------------------------------------------- */
SECTIONS
{
  /* --- Vector Table & Startup Code ------------------------------------- */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))        /* Vector table */
    . = ALIGN(4);
  } > FLASH

  /* --- Program Code ---------------------------------------------------- */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;                 /* End of code */
  } > FLASH

  /* --- Read-only Data -------------------------------------------------- */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  /* --- Exception Handling Tables -------------------------------------- */
  .ARM.extab :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } > FLASH

  .ARM :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } > FLASH

  /* --- C Runtime Initialization Tables -------------------------------- */
  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } > FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } > FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } > FLASH

  /* --- Initialized Data (in RAM, loaded from FLASH) ------------------- */
  .data : AT (ADDR(.fini_array) + SIZEOF(.fini_array))
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } > RAM

  /* Address in FLASH where .data values are stored */
  _sidata = LOADADDR(.data);

  /* --- Zero-initialized Data ------------------------------------------ */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > RAM

  /* --- Heap & Stack Reservation --------------------------------------- */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end  = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } > RAM

  /* --- Remove Unused Sections ----------------------------------------- */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  /* --- Attributes Section --------------------------------------------- */
  .ARM.attributes 0 : { *(.ARM.attributes) }
}

/* -------------------------------------------------------------------------
 * Memory Overflow Checks
 * ------------------------------------------------------------------------- */
ASSERT( SIZEOF(.text) + SIZEOF(.rodata) + SIZEOF(.ARM.extab) + SIZEOF(.ARM) 
        + SIZEOF(.init_array) + SIZEOF(.fini_array) < LENGTH(FLASH),
        "ERROR: FLASH overflow!" )

ASSERT( SIZEOF(.bss) + SIZEOF(._user_heap_stack) < LENGTH(RAM),
        "ERROR: RAM overflow!" )

/******************************************************************************
 *
 * Default Linker Command file for the Texas Instruments TM4C1294NCPDT
 *
 * This is derived from revision 15071 of the TivaWare Library.
 *
 *****************************************************************************/

/* Prevent the interrupt vector table from being removed as an unused symbol. */
--retain=g_pfnVectors

MEMORY
{
    /* 1 MiB on-chip executable/read-only program Flash, starting at reset. */
    FLASH (RX) : origin = 0x00000000, length = 0x00100000
    /* 256 KiB on-chip SRAM used for variables, heap, stack, and RAM vectors. */
    SRAM (RWX) : origin = 0x20000000, length = 0x00040000
}

/* The following command line options are set as part of the CCS project.    */
/* If you are building using the command line, or for some reason want to    */
/* define them here, you can uncomment and modify these lines as needed.     */
/* If you are using CCS for building, it is probably better to make any such */
/* modifications in your CCS project and leave this file alone.              */
/*                                                                           */
/* --heap_size=0                                                             */
/* --stack_size=256                                                          */
/* --library=rtsv7M4_T_le_eabi.lib                                           */

/* Section allocation in memory */

SECTIONS
{
    /* Cortex-M4 reset/interrupt vectors must begin at Flash address zero. */
    .intvecs:   > 0x00000000
    /* Executable code, constants, and C initialization tables remain in Flash. */
    .text   :   > FLASH
    .const  :   > FLASH
    .cinit  :   > FLASH
    .pinit  :   > FLASH
    .init_array : > FLASH

    /* DriverLib's writable vector table begins at the first SRAM address. */
    .vtable :   > 0x20000000
    /* Initialized variables, zeroed variables, heap, and call stack use SRAM. */
    .data   :   > SRAM
    .bss    :   > SRAM
    .sysmem :   > SRAM
    .stack  :   > SRAM
}

/*
 * Initial MSP value used by the vector table.  This legacy expression exposes
 * 512 bytes above __stack even though the current CCS link option reserves a
 * 2048-byte .stack section.  Keep this expression synchronized with the CCS
 * --stack_size setting if the effective interrupt/call stack is enlarged.
 */
__STACK_TOP = __stack + 512;

/******************************************************************************
 *
 * Default Linker Command file for the Texas Instruments TM4C123GH6PM
 *
 * This is derived from revision 15071 of the TivaWare Library.
 *
 *****************************************************************************/

--retain=g_pfnVectors

/* The starting address of the application.  Normally the interrupt vectors  */
/* must be located at the beginning of the application.                      */

//#define APP_BASE 0x00000000
//#define FLASH_LENGTH 0x00040000

#define APP_BASE 0x00002800
#define FLASH_LENGTH 0x0003d800

#define RAM_BASE 0x20000000

MEMORY
{
//    FLASH (RX) : origin = 0x00000000, length = 0x00040000
//    SRAM (RWX) : origin = 0x20000000, length = 0x00008000
    FLASH (RX) : origin = APP_BASE, length = FLASH_LENGTH
    SRAM (RWX) : origin = 0x20000000, length = 0x00008000
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
//    .intvecs:   > 0x00000000
    .intvecs:   > APP_BASE
    .text   :   > FLASH
    .const  :   > FLASH
    .cinit  :   > FLASH
    .pinit  :   > FLASH
    .init_array : > FLASH

    .vtable :   > 0x20000000
    .data   :   > SRAM
    .bss    :   > SRAM
    .sysmem :   > SRAM
    .stack  :   > SRAM
}

__STACK_TOP = __stack + 512;

/*
    @file   linkerscript_stm32f103c8.ld
    @author Ahmed Radwan Ibrahim
    @brief  Linker script for stm32f103c8 Device 
*/

/* tha last address in stack section */
_estack = ORIGIN(RAM) + LENGTH(RAM); /*the end adress of the RAM */

_Min_Heap_Size = 0x200;   /*min Amount of heap*/
_Min_Stack_Size = 0x400; /*min Amount of stack*/

/*memory definations*/
MEMORY{
    RAM   (xrw)  : ORIGIN = 0x20000000, LENGTH = 20K
    FLASH (xr)   : ORIGIN = 0x08000000, LENGTH = 64K
}

/*sections allocation*/
SECTIONS{
        /* .isr_vector section contain the ISR vector table ROM type memory*/
    .isr_vector :
    {
        . = ALIGN(4);
        *(.isr_vector)   /*any .isr_vector section */
        . = ALIGN(4);
        
    } >FLASH
    /* .text section contain the program code ROM type memory*/
    .text :
    {
        . = ALIGN(4);
        *(.text)   /*any .text section */
        *(.text*)  /*any .text--- section*/
        . = ALIGN(4);
        _etext = .; /* global sympol contain the end adress of . text section*/
    } >FLASH

    /* .rodata section contain constant data ROM type memory*/
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH
    

    /* .data section contain intialized global or static local data data loded from ROM to RAM by startup code*/
    
    _sidata = LOADADDR(.data); /*global sympol contain the start adress of .data section in ROM we used LOADDERR to take the adrress which .data in RAM will start load from ROM */

    .data :
    {
        _sdata = .;/*global sympol contain the start adress of .data section in RAM*/
        . = ALIGN(4);
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;/*global sympol contain the end adress of .data section section in RAM*/
    } >RAM AT>FLASH

    /* .bss section contain unintialized global or static local data data RAM memory type*/
    .bss :
    {
        _sbss = .;         /* Define a global symbol at bss start */
        . = ALIGN(4);
        *(.bss)
        *(.bss*)
        . = ALIGN(4);
        _ebss = .;         /* Define a global symbol at bss end */
    } >RAM

    /* .User_heap_stack section, used to check that there is enough "RAM" Ram  type memory left */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
     } >RAM
    

}

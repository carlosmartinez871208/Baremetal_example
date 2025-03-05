/*
** Linker Script created by Carlos Martinez.
*/

/* Entry point */
ENTRY(Reset_Handler)

/* Memory definitions */
MEMORY
{
    FLASH (rx): ORIGIN = 0x08000000, LENGTH = 512K  /* Flash size of stm32f401 microcontroller */
    SRAM (rwx): ORIGIN = 0x20000000, LENGTH = 96K   /* SRAM size of stm32f401 microcontroller */
}

/* From vector table (Reference manual page 202) 
** First element of the vector table is reserved.
*/
_estack = ORIGIN(SRAM) + LENGTH(SRAM);

/* Indicate required heap and stack size */
/* Generate a link error if heap and stack don't fit into RAM */
__max_heap_size = 0x200;  /* 512 bytes */
__max_stack_size = 0x400; /* 1024 bytes or 1 Kb */

/* From stm32f401 reference manual at page 41:
** Due to its fixed memory map, the code area starts from address 0x0000 0000 (accessed
** through the ICode/DCode buses) while the data area (SRAM) starts from address
** 0x2000 0000 (accessed through the system bus). The Cortex®-M4 with FPU CPU always
** fetches the reset vector on the ICode bus, which implies to have the boot space available
** only in the code area (typically, Flash memory). STM32F4xx microcontrollers implement a
** special mechanism to be able to boot from other memories (like the internal SRAM).
**
** |------------------------- |-------------------|-------------------------------------------------|
** | Boot Mode selection pins |                   |                                                 |
** |--------------------------|     Boot mode     |                     Aliasing                    |
** |    BOOT1   |    BOOT0    |                   |                                                 |
** |--------------------------|-------------------|-------------------------------------------------|
** |      X     |      0      | Main flash memory | Main flash memory is selected as the boot space |
** |------------|-------------|-------------------|-------------------------------------------------|
** |      0     |      1      | System memory     | System memory is selected as the boot space     |
** |------------|-------------|-------------------|-------------------------------------------------|
** |      1     |      1      | Embedded SRAM     | Embedded SRAM is selected as the boot space     |
** |------------|-------------|-------------------|-------------------------------------------------|
**
** The BOOT pins are also resampled when the device exits the Standby mode. Consequently,
** they must be kept in the required Boot mode configuration when the device is in the Standby
** mode. After this startup delay is over, the CPU fetches the top-of-stack value from address
** 0x0000 0000, then starts code execution from the boot memory starting from 0x0000 0004.
**
** In this case it boots from Main Flash memory 0x0800 0000 address and starts booting at 0x0800 0004
*/

/* Sections */
SECTIONS
{
    /* The program code and other data goes into FLASH */
    .text :
    {
        . = ALIGN(4); /* This align memoery every 4 bytes  */
        _stext = .; /* '.' indicates contains the current output location counter and it is asigned to global symbol: _stext (start of text section) symbol. */
        *(.isr_vector_table) /* Merge all '.isr_vector_table' sections of input files. */
        *(.text)             /* Merge all '.text' sections of inputs files. */
        *(.ctors)            /* Merge all '.ctors' constructor input files */
        *(.dtors)            /* Merge all '.ctors' constructor input files */
        *(.rodata)           /* Merge all '.rodata' sections of inputs files. */
        . = ALIGN(4); /* This align memoery every 4 bytes  */
        _etext = .; /* '.' indicates contains the current output location counter and it is asigned to global symbol: _etext (end of text section) symbol. */
    } > FLASH

    /* Initialized data sections goes into RAM, load LMA copy after code */
    .data :
    {
        . = ALIGN(4); /* This align memory every 4 bytes  */
        _sdata = .; /* '.' indicates contains the current output location counter and it is asigned to global symbol: _sdata (start of data section) symbol. */
        *(.data)    /* Merge all '.data' sections of inputs files. */
        . = ALIGN(4);
        _edata = .; /* '.' indicates contains the current output location counter and it is asigned to global symbol: _edata (end of data section) symbol. */
    } > SRAM AT> FLASH /*> (VMA) AT> (LMA) */
                       /* Load memory (LMA) is flash and then the VMA is SRAM. */
                       /* 'AT>' tells the linker to load a SRAM data from flash where it is located. */

    /* Uninitilized data section */
    .bss :
    {
        . = ALIGN(4); /* This align memoery every 4 bytes  */
        _sbss = .; /* '.' indicates contains the current output location counter and it is asigned to global symbol: _sbss (start of bss section) symbol. */
        *(.bss)    /* Merge all '.bss' sections of inputs files. */
        . = ALIGN(4); /* This align memoery every 4 bytes  */
        _ebss = .; /* '.' indicates contains the current output location counter and it is asigned to global symbol: _ebss (end of bss section) symbol. */
    } > SRAM
}
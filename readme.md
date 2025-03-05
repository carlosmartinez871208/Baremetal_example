# Embedded build systems.

## STM32F401
### Turn on an LED.
From **STM nucleo 401 Datasheet**.
The chosen one is: STM32 I/O PA5 (Port A, Pin 5 from microcontroller), pin21 (from Development kit).

From **STM32F401 Datasheet**.
We can find memory map at page 51.

So, we need to check the address direction for peripherals.

In this case the address is: 0x4000 0000. 

    #define PERIPHERAL_BASE (0x40000000ul)

Now, for ARM M4 processor microcontroller, it is required to allow clock.

It is required to know which **bus** is used to enabled clock to **GPIO PORTA**.

At page 14, we can find block diagram for microcontroller.

So, **AHB1 84** MHz bus connects to **GPIO PORTA**, **AHB1 84** peripheral is at address: 0x4002 0000.

    #define AHB1_PERIPHERAL_OFFSET (0x00020000ul)
    #define AHB1_PERIPHERAL_BASE (PERIPHERAL_BASE + AHB1_PERIPHERAL_OFFSET)

    #define GPIOA_OFFSET (0x0000ul)
    #define GPIOA_BASE (AHB1_PERIPHERAL_BASE + GPIOA_OFFSET)

There is another peripheral that has a register that is used to enable clock for AHB1: **RCC**, this peripheral is also connected to **AHB1** bus at address: 0x4002 3800.

    #define RCC_OFFSET (0x3800ul)
    #define RCC_BASE (AHB1_PERIPHERAL_BASE + RCC_OFFSET)

### From reference manual.
**Enable clock for GPIOA**

#### Very important:

    const int x; /* x is a constant data variable */
    int conts x; /* same as: const int x */

    const int* x; /* x is a non-constant pointer to constant data */
    int const* x; /* same as const int* x */

    int* const x; /* x is a constant pointer to non-constant data */

    const int* const x; /* x is a constant pointer to constant data */

And finally, it is posible to combine different type qualifiers. 

For example:

    volatile const int* x; /* x is a non-constant, non-volatile pointer to volatile constant data */
    volatile int const* x; /* same as volatile const int* x */
    int volatile const* x; /* same as volatile const int* x */

Application example:

    volatile unsigned int* x; /* x is a non volatile pointer to volatile unsigned int data */
    (*(volatile unsigned int*)(0x8000 0000)) /* Its a de-reference of an non-volatile pointer to volatile unsigned int data casting */

#### Continuing...

There is a register form RCC that enables clock to GPIOA.

At page 118 we can find the description for that register.

> Address offset from RCC_BASE: 0x30.

> Reset value: 0x0000 0000.

Access: no wait state, word, half-word and byte access.

    #define RCC_AHB1ENR_OFFSET (0x30ul)
    #define RCC_AHB1ENR (*(volatile unsigned int*)(RCC_BASE + RCC_AHB1ENR_OFFSET))

So, GPIOAEN is located at bit 0 from RCC_AHB1ENR register.

> : I/O port A clock is disable.

> 1: I/O port A clock is enable.

    #define GPIOA_ENABLE  (0x1u)
    #define GPIOA_DISABLE (0x0u)

    #define GPIOAEN (GPIOA_ENABLE << 0) /* Or (GPIOA_DISABLE << 0) */

**GPIO Mode**

Within the GPIOs, there is a register called GPIOx_MODER, x(A,B,C,...).

At page 158,there is the description for GPIOx_MODER, x(A,B,C,...).

**Reset values:**

> 0x0C00 0000: for A.

> 0x0000 0280: for B.

> 0x0000 0000: for other ports.

**Port x configuration bit.**

> 00: Input (reset stable).

> 01: General purpose output mode.

> 10: Alternate function mode.

> 11: Analog mode.

In this case we need to configure PA5, this means configure MODER (Mode register 5).

    #define GPIOA_MODER_OFFSET (0x0ul)
    #define GPIOA_MODER (*(volatile unsigned int*)(GPIOA_BASE + GPIOA_MODER_OFFSET))

    #define GPIOx_MODER_INPUT (0x0u)
    #define GPIOx_MODER_OUTPUT (0x1u)
    #define GPIOx_MODER_ALTERNATE (0x2u)
    #define GPIOx_MODER_ANALOG (0x3)

    /*
    ** (1u<<10) /* Set bit 10 to 1. */
    ** &=~(1u<<11) /* Set bit 11 to 0. */
    */

    #define MODER5 (GPIOx_MODER_OUTPUT << 10) /* Could be any of the other three options */

At page 160, GPIO port output data register (GPIOx_ODR).

> Address offset: 0x14.

> Reset value: 0x0000 0000.

    #define GPIOA_ODR_OFFSET (0x14ul)
    #define GPIOA_ODR (*(volatile unsigned int*)(GPIOA_BASE + GPIOA_ODR_OFFSET))

    #define ODR5 (1U << 5)
    #define PA5 (ODR5)
    #define LED_PIN (PA5)

### Code.

    #define PERIPHERAL_BASE (0x40000000ul)

    #define AHB1_PERIPHERAL_OFFSET (0x00020000ul)
    #define AHB1_PERIPHERAL_BASE (PERIPHERAL_BASE + AHB1_PERIPHERAL_OFFSET)

    #define GPIOA_OFFSET (0x0000ul)
    #define GPIOA_BASE (AHB1_PERIPHERAL_BASE + GPIOA_OFFSET)

    #define RCC_OFFSET (0x3800ul)
    #define RCC_BASE (AHB1_PERIPHERAL_BASE + RCC_OFFSET)

    #define RCC_AHB1ENR_OFFSET (0x30ul)
    #define RCC_AHB1ENR (*(volatile unsigned int*)(RCC_BASE + RCC_AHB1ENR_OFFSET))

    #define GPIOA_ENABLE  (0x1u)
    #define GPIOA_DISABLE (0x0u)

    #define GPIOAEN (GPIOA_ENABLE << 0) /* Or (GPIOA_DISABLE << 0) */

    #define GPIOA_MODER_OFFSET (0x0ul)
    #define GPIOA_MODER (*(volatile unsigned int*)(GPIOA_BASE + GPIOA_MODER_OFFSET))

    #define GPIOx_MODER_INPUT (0x0u)
    #define GPIOx_MODER_OUTPUT (0x1u)
    #define GPIOx_MODER_ALTERNATE (0x2u)
    #define GPIOx_MODER_ANALOG (0x3)

    /*
    ** (1u<<10) /* Set bit 10 to 1. */
    ** &=~(1u<<11) /* Set bit 11 to 0. */
    */

    #define MODER5 (GPIOx_MODER_OUTPUT << 10) /* Could be any of the other three options */

    #define GPIOA_ODR_OFFSET (0x14ul)
    #define GPIOA_ODR (*(volatile unsigned int*)(GPIOA_BASE + GPIOA_ODR_OFFSET))

    #define ODR5 (1U << 5)
    #define PA5 (ODR5)
    #define LED_PIN (PA5)

    int main (void)
    {
        /* Enable clock access to GPIOA */
        RCC_AHB1ENR |= GPIOAEN;
        /* Set PA5 as output*/
        GPIOA_MODER |= MODER5;
        while(true)
        {
            /* Set PA5 High */
            GPIOA_ODR |= LED_PIN;
        }
    }

### Same program using structures.

First of all define __IO

    #define __IO volatile

    #define PERIPHERAL_BASE (0x40000000ul)

    #define AHB1_PERIPHERAL_OFFSET (0x00020000ul)
    #define AHB1_PERIPHERAL_BASE (PERIPHERAL_BASE + AHB1_PERIPHERAL_OFFSET)

    #define GPIOA_OFFSET (0x0000ul)
    #define GPIOA_BASE (AHB1_PERIPHERAL_BASE + GPIOA_OFFSET)

    #define RCC_OFFSET (0x3800ul)
    #define RCC_BASE (AHB1_PERIPHERAL_BASE + RCC_OFFSET)

    #define GPIOA_ENABLE  (0x1u)
    #define GPIOA_DISABLE (0x0u)

    #define GPIOAEN (GPIOA_ENABLE << 0) /* Or (GPIOA_DISABLE << 0) */

    #define GPIOx_MODER_INPUT (0x0u)
    #define GPIOx_MODER_OUTPUT (0x1u)
    #define GPIOx_MODER_ALTERNATE (0x2u)
    #define GPIOx_MODER_ANALOG (0x3)

    /*
    ** (1u<<10) /* Set bit 10 to 1. */
    ** &=~(1u<<11) /* Set bit 11 to 0. */
    */

    #define MODER5 (GPIOx_MODER_OUTPUT << 10) /* Could be any of the other three options */

    #define ODR5 (1U << 5)
    #define PA5 (ODR5)
    #define LED_PIN (PA5)

Create RCC Type

    typedef struct
    {
        __IO uint32_t NOTUSED[12]; 
        __IO uint32_t AHB1ENR; /* AHB1 peripheral clock register */
    }RCC_Map;

Create GPIO Type

    typedef struct
    {
        __IO uint32_t MODER;   /* Mode Register,                    offset: 0x00 */
        __IO uint32_t NOTUSED[4];
        __IO uint32_t ODR;     /* Output Data Register,             offset: 0x14 */
    }GPIO_Map;

Assigning address to the struct.

    #define RCC   ((RCC_Map*) RCC_BASE);    /* Casting to RCC_Map given address. */
    #define GPIOA ((GPIO_Map*) GPIOA_BASE); /* Casting to GPIOA_Map given address. */

### Code.

    #define __IO volatile

    #define PERIPHERAL_BASE (0x40000000ul)

    #define AHB1_PERIPHERAL_OFFSET (0x00020000ul)
    #define AHB1_PERIPHERAL_BASE (PERIPHERAL_BASE + AHB1_PERIPHERAL_OFFSET)

    #define GPIOA_OFFSET (0x0000ul)
    #define GPIOA_BASE (AHB1_PERIPHERAL_BASE + GPIOA_OFFSET)

    #define RCC_OFFSET (0x3800ul)
    #define RCC_BASE (AHB1_PERIPHERAL_BASE + RCC_OFFSET)

    #define GPIOA_ENABLE  (0x1u)
    #define GPIOA_DISABLE (0x0u)

    #define GPIOAEN (GPIOA_ENABLE << 0) /* Or (GPIOA_DISABLE << 0) */

    #define GPIOx_MODER_INPUT (0x0u)
    #define GPIOx_MODER_OUTPUT (0x1u)
    #define GPIOx_MODER_ALTERNATE (0x2u)
    #define GPIOx_MODER_ANALOG (0x3)

    /*
    ** (1u<<10) /* Set bit 10 to 1. */
    ** &=~(1u<<11) /* Set bit 11 to 0. */
    */

    #define MODER5 (GPIOx_MODER_OUTPUT << 10) /* Could be any of the other three options */

    #define ODR5 (1U << 5)
    #define PA5 (ODR5)
    #define LED_PIN (PA5)

    typedef struct
    {
        __IO uint32_t NOTUSED[12]; 
        __IO uint32_t AHB1ENR; /* AHB1 peripheral clock register */
    }RCC_Map;

    typedef struct
    {
        __IO uint32_t MODER;   /* Mode Register,                    offset: 0x00 */
        __IO uint32_t NOTUSED[4];
        __IO uint32_t ODR;     /* Output Data Register,             offset: 0x14 */
    }GPIO_Map;

    #define RCC   ((RCC_Map*) RCC_BASE);    /* Casting to RCC_Map given address. */
    #define GPIOA ((GPIO_Map*) GPIOA_BASE); /* Casting to GPIOA_Map given address. */

    int main (void)
    {
        /* Enable clock access to GPIOA */
        RCC->AHB1ENR |= GPIOAEN;
        /* Set PA5 as output*/
        GPIOA->MODER |= MODER5;
        while(1)
        {
            /* Set PA5 High */
            GPIOA->ODR |= LED_PIN;
        }
    }

### Compilation.

> Preprocesing: In this stage *.h files are expanded, macros are substituted by values, no compilation warnings.

> 1. input: *.c/*.c++ source code files.
> 2. output: *.i files.

> Compilation: In this stage *.i files are compiled to assembler language.

> 1. input: *i files.
> 2. output: *s files.

> Assembling: In this stage *.s files are processed and produces a relocatable object file.

> 1. inputs: *.s files.
> 2. outputs: *.o files.

> Linking: Is this stage relocated object files are processed to produces and Executable Linked File (ELF).

> 1. inputs: *.o files.
> 2. outputs: elf.

### To compile a file or a project, GNU tool will be used:

    arm-none-eabi-cpp main.c -o main.o

#### Compiler flags.

-c: Compile and assemble, no linking performed.

-o: Compile, assemble and link to output file.

-g: Generate debugging information in executable.

-Wall: Enable all warning messages.

-Werror: Treat all warnings as errors.

-I[DIR]: include this DIR to look for header files.

-ansi:         Specify which standar version to use.
-std=STANDARD

-v: Verbose output from GCC.

-mcpu = NAME: Specifies the target ARM processor and architecture.

    -mcpu = cortex-m4

-march = NAME: Specifies the target ARM architecture.

    -march = armv7-m

-mtune = NAME: Speficies the target ARM processor.

    -mtune = cortex-m4

-mthumb: To generate code in Thumb Instruction Set Architecture (ISA).

-marm: To generate code in ARM Instruction Set Architecture (ISA).

-mlittle-endian: To generate code for little endian mode.

-mbig-endian: To generate code for big endia mode.

### List the symbols from object files.

    arm-none-eabi-nm main.o

### List the sections sizes of object and executable files.

    arm-none-eabi-size main.o

### Analyze an object file

    arm-none-eabi-objdump /* Dumps information about an object files */

### Display information from ELF files.

    arm-none-eabi-readelf

### Linker Script

It is given to the linker in roder to specify the memory layout and to initialize the various memoery sections used by the firmware.

The linker script is reponsible for making sure various portions of the firmware are in their proper place and also for associating meaninful labels with specific regions of memory used by the startup code.

The script has 4 features:

> 1. Memory layout: Available memory types.
> 2. Section definitions: Placing specific parts at specific locations.
> 3. Options: Commands e.g. ENTRY POINT.
> 4. Symbols: Variables to inject into program at link time.

In order to allocate progra memory, the linker needs to know the types of memory, their addresses and sizes.

**MEMORY** definition it is used to provide this information.

    MEMORY
    {
        name [(attr)] : ORIGIN = origin, LENGHT = len
    }

E.G.

    MEMORY
    {
        FLASH(rx) : ORIGIN = 0x08000000, LENGHT = 512 Kb
        SRAM(rwx) : ORIGIN = 0x20000000, LENGTH = 96 Kb
    }

**SECTIONS** where code and data are organized into.

Symbols of the same memory region are placed in the same section.

    SECTIONS
    {
        ....
    }

E.G.

    SECTIONS
    {
        .text :
        {
            *(.text) /* Merge all .text sections of input files */
        } > FLASH
    }

There are three relevant sections:

.text: Placed in the FLASH.

.data: Place in the SRAM.

.bss:  Place in the SRAM.

#### Flash Memory
It is read only memory.

Within the datasheet at page 51, it is observed that Flash memory starts at 0x0800 0000 address and ends at 0x0807 FFFF.

It means that flash memory size is: 0x0807 FFFF - 0x0800 0000 = 0x7 FFFF = 524 287 therefore 524 287/1024 = 511.999 = 512 Kb.

Program code is stored here.

Also contains a vector table at address 0x0800 0004, it consists of pointer to the various exception handlers.

**Flash memory sections.**

> 1. .data

> 2. .rodata

> 3. .text

> 4. .isr_vector_tbl

#### SRAM Memory
It is read and write memory.

Within the datasheet at page 51, it is observed that Flash memory starts at 0x2000 0000 address and ends at 0x3FFF FFFF.

It means that flash memory size is: 0x3FFF FFFF - 0x2000 0000 = 0x1FFF FFFF = 526 870 911 therefore 524 870 9111/1 048 576 = 511.999 = 512 Mb.

Variables and stack are stored here.

SRAM (96 Kb aliased by bit-handing). 0x2000 0000 - 0x2001 7FFF.

**SRAM Memory Sections**

> 1. .bss

> 2. data

#### Linker Script commands.

> **ENTRY**: Sets program entry address.

> **MEMORY**: Describes the locations and sizes of the different memories available.

> **ALIGN**: Inserts padding to align location.

> **SECTIONS**: Used to map input sections to output sections and descrobes how to place output sections in memory.

> **KEEP**: Tell sthe linker to keep the specified section even if no symbols in that section are referenced.

> **AT>**: This is a directive, it tells the linker to load a section's data to somewhere other than the address it is located.

#### Linker Constants.
The linker considers an integer with:
> 0 as Octal.

> 0x as Hexadecimal.

Suffixes K and M may be used to scale a constant by 1024 or by 1024*1024 respectively.

#### Symbols.

> 1. Symbols have a name and a value.

> 2. Symbols values are unsigned integers.

> 3. Symbols are generated by the compiler for each function and variable.

> 4. The value represent the target address where the variable or function is stored.

> 5. When a symbol is refered by name in the linker command file or in an assembly file, it returns an integer value.

e.g.

    x (variable name) = 3500 (value)
    y = x /* y = 3500 */

> 1. The compiler fetches 3500.

> 2. To achieve this, the compiler generates a linker symbol calles x with the value &x.

> 3. Although C/C++ variables and the linker symbol have the same name but **they don't represent the same thing**.

> 4. In C/C++ x is a variable name with address &x and value 3500.

> 5. For the linker simbols x is an address, and that address contains 3500.

e.g.

    int x = 3500;

    void blink(void)
    {
        led_on();
        wait();
        led_off();
    }

After compilation: *.o file

    |--------------------------|
    |       Symbol table       |
    |--------------------------|
    |  Symbol   |    Address   |
    |-----------|--------------|
    |     x     |  0x20000000  |
    |-----------|--------------|
    |   blink   |  0x08000000  |
    |--------------------------|

> Values may be assigned to symbols. This will define them and place them into the **Symbol table**.

**Location counter**

> 1. It is a special linker symbol written as "." dot.

> 2. This symbol always contains the current output location counter.

> 3. This symbol can only appear inside the **SECTION** command, because it always refers to a location in an output section.

> 4. This symbol can be used to define and know the boundaries of various sections.

> 5. Since symbols are addresses, this symbol is incremented by the size of the output section.

> 6. This symbol **may not be moved backward** i.e. assigned a value lower than the current output location. Doing this may lead to creating areas of ***overlapping*** LMAs.

**Example of linker script to load program code at address 0x0800 0000 and data at 0x2000 0000.**

> 1. If the address of an putput section is not specified, the address is set from the current value of the location counter.

> 2. The location counter is incremented by the size of the output section.

> 3. At the start of the **SECTIONS** command, the location counter has the value of **0**.

> 4. The second line defines an output section **.text**.

> 5. Since the location counter is **0x0800 0000** when the output section **.text** is defined, the linker will set the address of the **.text** section in the output file to be **0x0800 0000**.

> 6. The linker will place the **.data** output section at address **0x2000 0000**.

    SECTIONS
    {
        . = 0x08000000; /* Set value of location counter */
        .text:
        {
            *(.text) /* Merge all .text sections of */
                     /* Input files */
        }
        . = 0x20000000;
        .data:
        {
            *(.data) /* Merge all .data section of */
                     /* Inout files */
        }
        .
    }

#### Constructors and destructors. 
Within **.text** it is place a section for constructors and destructors. This is specically for C++.

Cosntructors:

    *(.ctors)

Destructors:

    *(.dtors)

#### Linker Script for stm32f401 microcontroller: stm32f401.ld

    /* Entry point */
    ENTRY(Reset_Handler)

    /* Memory definitions */
    MEMORY
    {
        FLASH(rx) : ORIGIN = 0x08000000, LENGHT = 512 K /* Flash size of stm32f401 microcontroller */
        SRAM(rwx) : ORIGIN = 0x20000000, LENGTH = 96 K   /* SRAM size of stm32f401 microcontroller */
    }

    /* From vector table (Reference manual page 202) 
    ** First element of the vector table is reserved.
    */
    _estack = ORIGIN(SRAM) + LENGTH(SRAM)

    /* Indicate required heap and stack size */

    __max_heap_size = 0x200  /* 512 bytes */
    __max_stack_size = 0x400 /* 1024 bytes or 1 Kb */

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
        /* First section of output file. */
        .text :
        {
            . = ALIGN(4);
            _stext = .; /* '.' indicates contains the current output location counter and it is asigned to global symbol: _stext (start of text section) symbol. */
            *(.isr_vector_table) /* Merge all '.isr_vector_table' sections of input files. */
            *(.text)             /* Merge all '.text' sections of inputs files. */
            *(.ctors)            /* Merge all '.ctors' constructor input files */
            *(.dtors)            /* Merge all '.dtors' destructor input files */
            *(.rodata)           /* Merge all '.rodata' sections of inputs files. */
            . = ALIGN(4);
            _etext = .; /* '.' indicates contains the current output location counter and it is asigned to global symbol: _etext (end of text section) symbol. */
        }>FLASH

        /* Second section of output file. */
        .data :
        {
            . = ALIGN(4);
            _sdata = .; /* '.' indicates contains the current output location counter and it is asigned to global symbol: _sdata (start of data section) symbol. */
            *(.data)    /* Merge all '.data' sections of inputs files. */
            . = ALIGN(4);
            _edata = .; /* '.' indicates contains the current output location counter and it is asigned to global symbol: _edata (end of data section) symbol. */
        } > SRAM AT> FLASH /*> (VMA) AT> (LMA) */
                           /* Load memory (LMA) is flash and then the VMA is SRAM. */
                           /* 'AT>' tells the linker to load a SRAM data from flash where it is located. */

        /* Second section of output file. */
        .bss :
        {
            . = ALIGN(4);
            _sbss = .; /* '.' indicates contains the current output location counter and it is asigned to global symbol: _sbss (start of bss section) symbol. */
            *(.bss)    /* Merge all '.bss' sections of inputs files. */
            . = ALIGN(4);
            _ebss = .; /* '.' indicates contains the current output location counter and it is asigned to global symbol: _ebss (end of bss section) symbol. */
        } > SRAM
    }

### Startup code.
#### Reset Handler.
This function copies the initial values for variables from the FLASH where linker places them to the SRAM: Copy .data section from FLASH to SRAM.

It also zeroz out the uninitialized data portion of the SRAM.

#### Interrupt Vector Table.
This is an array that holds memory address of interrupt handler functions.

In order to allow application code to conveniently redefine the various interrupts handlers, every required vector is assigned an overideable **_weak** alias to a default function which loops forever.

#### LMA: Load Memory Address.
The address at which a section is loaded.

#### VMA: Virtual Memory Address.
The address of a section during execution.

#### Reset Handler.
Copy .data section from FLASH to SRAM.



#### Startup file for stm32f401 microcontroller: stm32f401_startup.c

    #ifndef uint32_t
     typedef unsigned long uint32_t;
    #endif 

    /* Into linker script can be found _estack
    ** it is used as the first vector table element.
    */
    extern uint32_t _estack;

    /* Global symbols */
    extern uint32_t _stext;
    extern uint32_t _etext;
    extern uint32_t _sdata;
    extern uint32_t _edata;
    extern uint32_t _sbss;
    extern uint32_t _ebss;

    /* Default Handler prototype */
    void Default_Handler(void);

    /* Reset Handler prototype */
    void Reset_Handler (void);

    /* Interrupt and exceptions handlers: page 601 (GCC-14.2)
    ** Provide 'weak' aliases for each exception handler to the deault handler.
    ** As they are 'weak' aliasses, any function with the same name will override
    ** this definition.
    ** The alias attribute causes the declaration to be emitted as an alias for another
    ** symbol, which must have been previously declared with the same type, and for
    ** variables, also the same size and aligment.
    ** Declaring an alias with different type than the target is undefined and my be
    ** diagnosed.
    ** e.g.
    ** void __f () {/* Do something */}
    ** void f () __attribute__ ((weak,alias ("__f")))
    */
    /* Microcontroller exception */
    void NMI_Handler                      (void)__attribute__((weak,alias("Default_Handler")));
    void HardFault_Handler                (void)__attribute__((weak,alias("Default_Handler")));
    void MemManage_Handler                (void)__attribute__((weak,alias("Default_Handler")));
    void BusFault_Handler                 (void)__attribute__((weak,alias("Default_Handler")));
    void UsageFault_Handler               (void)__attribute__((weak,alias("Default_Handler")));
    void SVCall_Handler                   (void)__attribute__((weak,alias("Default_Handler")));
    void DebugMonitor_Handler             (void)__attribute__((weak,alias("Default_Handler")));
    void PendSV_Handler                   (void)__attribute__((weak,alias("Default_Handler")));
    void Systick_Handler                  (void)__attribute__((weak,alias("Default_Handler")));

    /* Interrupst */
    void WWDOG_IRQHandler                 (void)__attribute__((weak,alias("Default_Handler")));
    void EXTI16_PVD_IRQHandler            (void)__attribute__((weak,alias("Default_Handler")));
    void EXTI21_TAMP_STAMP_IRQHandler     (void)__attribute__((weak,alias("Default_Handler")));
    void EXTI22_RTC_WKUP_IRQHandler       (void)__attribute__((weak,alias("Default_Handler")));
    void FLASH_IRQHandler                 (void)__attribute__((weak,alias("Default_Handler")));
    void RCC_IRQHandler                   (void)__attribute__((weak,alias("Default_Handler")));
    void EXTI0_IRQHandler                 (void)__attribute__((weak,alias("Default_Handler")));
    void EXTI1_IRQHandler                 (void)__attribute__((weak,alias("Default_Handler")));
    void EXTI2_IRQHandler                 (void)__attribute__((weak,alias("Default_Handler")));
    void EXTI3_IRQHandler                 (void)__attribute__((weak,alias("Default_Handler")));
    void EXTI4_IRQHandler                 (void)__attribute__((weak,alias("Default_Handler")));
    void DMA1_Stream0_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void DMA1_Stream1_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void DMA1_Stream2_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void DMA1_Stream3_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void DMA1_Stream4_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void DMA1_Stream5_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void DMA1_Stream6_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void ADC_IRQHandler                   (void)__attribute__((weak,alias("Default_Handler")));
    void EXTI9_5_IRQHandler               (void)__attribute__((weak,alias("Default_Handler")));
    void TIM1_BRK_TIM9_IRQHandler         (void)__attribute__((weak,alias("Default_Handler")));
    void TIM1_UP_TIM10_IRQHandler         (void)__attribute__((weak,alias("Default_Handler")));
    void TIM1_TRG_COM_TIM11_IRQHandler    (void)__attribute__((weak,alias("Default_Handler")));
    void TIM1_CC_IRQHandler               (void)__attribute__((weak,alias("Default_Handler")));
    void TIM2_IRQHandler                  (void)__attribute__((weak,alias("Default_Handler")));
    void TIM3_IRQHandler                  (void)__attribute__((weak,alias("Default_Handler")));
    void TIM4_IRQHandler                  (void)__attribute__((weak,alias("Default_Handler")));
    void I2C1_EV_IRQHandler               (void)__attribute__((weak,alias("Default_Handler")));
    void I2C1_ER_IRQHandler               (void)__attribute__((weak,alias("Default_Handler")));
    void I2C2_EV_IRQHandler               (void)__attribute__((weak,alias("Default_Handler")));
    void I2C2_ER_IRQHandler               (void)__attribute__((weak,alias("Default_Handler")));
    void SPI1_IRQHandler                  (void)__attribute__((weak,alias("Default_Handler")));
    void SPI2_IRQHandler                  (void)__attribute__((weak,alias("Default_Handler")));
    void USART1_IRQHandler                (void)__attribute__((weak,alias("Default_Handler")));
    void USART2_IRQHandler                (void)__attribute__((weak,alias("Default_Handler")));
    void EXTI15_10_IRQHandler             (void)__attribute__((weak,alias("Default_Handler")));
    void EXTI17_RTC_Alarm_IRQHandler      (void)__attribute__((weak,alias("Default_Handler")));
    void EXTI18_OTG_FS_WKUP_IRQHandler    (void)__attribute__((weak,alias("Default_Handler")));
    void DMA1_Stream7_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void SDIO_IRQHandler                  (void)__attribute__((weak,alias("Default_Handler")));
    void TIM5_IRQHandler                  (void)__attribute__((weak,alias("Default_Handler")));
    void SPI3_IRQHandler                  (void)__attribute__((weak,alias("Default_Handler")));
    void DMA2_Stream0_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void DMA2_Stream1_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void DMA2_Stream2_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void DMA2_Stream3_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void DMA2_Stream4_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void OTG_FS_IRQHandler                (void)__attribute__((weak,alias("Default_Handler")));
    void DMA2_Stream5_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void DMA2_Stream6_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void DMA2_Stream7_IRQHandler          (void)__attribute__((weak,alias("Default_Handler")));
    void USART6_IRQHandler                (void)__attribute__((weak,alias("Default_Handler")));
    void I2C3_EV_IRQHandler               (void)__attribute__((weak,alias("Default_Handler")));
    void I2C3_ER_IRQHandler               (void)__attribute__((weak,alias("Default_Handler")));
    void FPU_IRQHandler                   (void)__attribute__((weak,alias("Default_Handler")));
    void SPI4_IRQHandler                  (void)__attribute__((weak,alias("Default_Handler")));

    /* From GNU Compiler Collection: pages 674 and 675 (GCC-14.2)
    ** Normally, the compiler places the objects it generates in sections like data and bss.
    ** Sometimes, however, you need additional sections, or you need certain
    ** particular variables to appear in special sections, for example to map to special
    ** hardware.
    ** The section attribute specifies that a variable (or function) lives
    ** in a particular section. For example, this small program uses several specific
    ** section names:
    ** e.g.
    ** int init_data __attribute__ ((section ("INITDATA")));
    */
    uint32_t vector_table[] __attribute__((section (".isr_vector_table"))) = {\
        (uint32_t) &_estack,
        (uint32_t) &Reset_Handler,
        (uint32_t) &NMI_Handler,
        (uint32_t) &HardFault_Handler,
        (uint32_t) &MemManage_Handler,
        (uint32_t) &BusFault_Handler,
        (uint32_t) &UsageFault_Handler,
        0u,
        0u,
        0u,
        0u,
        (uint32_t) &SVCall_Handler,
        (uint32_t) &DebugMonitor_Handler,
        0u,
        (uint32_t) &PendSV_Handler,
        (uint32_t) &Systick_Handler,
        (uint32_t) &WWDOG_IRQHandler,
        (uint32_t) &EXTI16_PVD_IRQHandler,
        (uint32_t) &EXTI21_TAMP_STAMP_IRQHandler,
        (uint32_t) &EXTI22_RTC_WKUP_IRQHandler,
        (uint32_t) &FLASH_IRQHandler,
        (uint32_t) &RCC_IRQHandler,
        (uint32_t) &EXTI0_IRQHandler,
        (uint32_t) &EXTI1_IRQHandler,
        (uint32_t) &EXTI2_IRQHandler,
        (uint32_t) &EXTI3_IRQHandler,
        (uint32_t) &EXTI4_IRQHandler,
        (uint32_t) &DMA1_Stream0_IRQHandler,
        (uint32_t) &DMA1_Stream1_IRQHandler,
        (uint32_t) &DMA1_Stream2_IRQHandler,
        (uint32_t) &DMA1_Stream3_IRQHandler,
        (uint32_t) &DMA1_Stream4_IRQHandler,
        (uint32_t) &DMA1_Stream5_IRQHandler,
        (uint32_t) &DMA1_Stream6_IRQHandler,
        (uint32_t) &ADC_IRQHandler,
        0u,
        0u,
        0u,
        0u,
        (uint32_t) &EXTI9_5_IRQHandler,
        (uint32_t) &TIM1_BRK_TIM9_IRQHandler,
        (uint32_t) &TIM1_UP_TIM10_IRQHandler,
        (uint32_t) &TIM1_TRG_COM_TIM11_IRQHandler,
        (uint32_t) &TIM1_CC_IRQHandler,
        (uint32_t) &TIM2_IRQHandler,
        (uint32_t) &TIM3_IRQHandler,
        (uint32_t) &TIM4_IRQHandler,
        (uint32_t) &I2C1_EV_IRQHandler,
        (uint32_t) &I2C1_ER_IRQHandler,
        (uint32_t) &I2C2_EV_IRQHandler,
        (uint32_t) &I2C2_ER_IRQHandler,
        (uint32_t) &SPI1_IRQHandler,
        (uint32_t) &SPI2_IRQHandler,
        (uint32_t) &USART1_IRQHandler,
        (uint32_t) &USART2_IRQHandler,
        0u,
        (uint32_t) &EXTI15_10_IRQHandler,
        (uint32_t) &EXTI17_RTC_Alarm_IRQHandler,
        (uint32_t) &EXTI18_OTG_FS_WKUP_IRQHandler,
        0u,
        0u,
        0u,
        0u,
        (uint32_t) &DMA1_Stream7_IRQHandler,
        0u,
        (uint32_t) &SDIO_IRQHandler,
        (uint32_t) &TIM5_IRQHandler,
        (uint32_t) &SPI3_IRQHandler,
        0u,
        0u,
        0u,
        0u,
        (uint32_t) &DMA2_Stream0_IRQHandler,
        (uint32_t) &DMA2_Stream1_IRQHandler,
        (uint32_t) &DMA2_Stream2_IRQHandler,
        (uint32_t) &DMA2_Stream3_IRQHandler,
        (uint32_t) &DMA2_Stream4_IRQHandler,
        0u,
        0u,
        0u,
        0u,
        0u,
        0u,
        (uint32_t) &OTG_FS_IRQHandler,
        (uint32_t) &DMA2_Stream5_IRQHandler,
        (uint32_t) &DMA2_Stream6_IRQHandler,
        (uint32_t) &DMA2_Stream7_IRQHandler,
        (uint32_t) &USART6_IRQHandler,
        (uint32_t) &I2C3_EV_IRQHandler,
        (uint32_t) &I2C3_ER_IRQHandler,
        0u,
        0u,
        0u,
        0u,
        0u,
        0u,
        0u,
        (uint32_t) &FPU_IRQHandler,
        0u,
        0u,
        (uint32_t) &SPI4_IRQHandler
    };

    /* Defining default handler:*/
    void Default_Handler(void)
    {
        while(1)
        {
            /* Do nothing */
        }
    }

    /* Entry point: Reset_Handler */
    void Reset_Handler (void)
    {
        uint32_t data_mem_size = (uint32_t)&_edata - (uint32_t)&_sdata; /* Size of data section */
        uint32_t bss_mem_size  = (uint32_t)&_ebss - (uint32_t)&_sbss;   /* Size of bss section */

        uint32_t *p_src_mem    = (uint32_t*)&_etext;
        uint32_t *p_dest_mem   = (uint32_t*)&_sdata;

        for(uint32_t i=0u;i<data_mem_size;i++)
        {
            /* Copy data section from FLASH to SRAM */
            *p_dest_mem++ = * p_src_mem++;
        }

        p_dest_mem = (uint32_t*)&_sbss;
        for(uint32_t i=0;i<bss_mem_size; i++)
        {
            /* Initialize .bss section to 0u*/
            *p_dest_mem++ = 0u;
        }
        /* Calling system initialization */

        /*Calling main function.*/
        (void)main();
    }

## Building.

main.c

    arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb -std=gnu11 main.c -o main.o
    
stm32f401_startup.c

    arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb -std=gnu11 stm32f401_startup.c -o stm32f401_startup.o

Linking.

    arm-none-eabi-gcc -nostdlib -T stm32f401_ls.ld *.o -o stm32f401 -Wl,-Map=stm32f401.map
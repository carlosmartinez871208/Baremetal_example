/*
** Startup file created by Carlos Martinez.
*/

#ifndef uint32_t
 typedef unsigned long uint32_t;
#endif 

/* Peripheral Initialization for System Initilization */
#define PERIPHERAL_BASE (0x40000000ul)

#define DBGMCU_BASE (0xE0042000ul) /* See reference manual */

#define APB1_PERIPHERAL_OFFSET (0x00000000ul)
#define APB1_PERIPHERAL_BASE (PERIPHERAL_BASE + APB1_PERIPHERAL_OFFSET)

#define PWR_OFFSET (0x00007000ul)
#define PWR_BASE (APB1_PERIPHERAL_BASE + PWR_OFFSET)

#define PWR_CR_OFFSET (0x0ul)
#define PWR_CR (*(volatile unsigned int*)(PWR_BASE + PWR_CR_OFFSET))

#define AHB1_PERIPHERAL_OFFSET (0x00020000ul)
#define AHB1_PERIPHERAL_BASE (PERIPHERAL_BASE + AHB1_PERIPHERAL_OFFSET)

#define RCC_OFFSET (0x3800ul)
#define RCC_BASE (AHB1_PERIPHERAL_BASE + RCC_OFFSET)

#define RCC_APB1LPENR_OFFSET (0x60ul)
#define RCC_APB1LPENR (*(volatile unsigned int*)(RCC_BASE + RCC_APB1LPENR_OFFSET))

#define RCC_CR_OFFSET (0x0ul)
#define RCC_CR (*(volatile unsigned int*)(RCC_BASE + RCC_CR_OFFSET))

#define RCC_PLLCFGR_OFFSET (0x4ul)
#define RCC_PLLCFGR (*(volatile unsigned int*)(RCC_BASE + RCC_PLLCFGR_OFFSET))

#define RCC_CFGR_OFFSET (0x8ul)
#define RCC_CFGR (*(volatile unsigned int*)(RCC_BASE + RCC_CFGR_OFFSET))

#define FLASH_OFFSET (0x3C00)
#define FLASH_BASE (AHB1_PERIPHERAL_BASE + FLASH_OFFSET)

#define FLASH_ACR_OFFSET (0x0ul)
#define FLASH_ACR (*(volatile unsigned int*)(FLASH_BASE + FLASH_ACR_OFFSET))

#define DBGMCU_CR_OFFSET (0x4ul)
#define DBGMCU_CR (*(volatile unsigned int*)(DBGMCU_BASE + DBGMCU_CR_OFFSET))

#define RCC_APB1LPENR_PWRLPEN (1ul << 28) 
#define PWR_CR_VOS (1ul < 15)
#define RCC_CR_HSIRDY (0x000000002ul)
#define PWR_CR_PLS (4ul << 5) /* Select voltage threshold detected by the Power voltage Detector: 2.6 V. */
#define RCC_CR_PLLON (1ul << 24)
#define RCC_CR_PLLRDY (1ul << 25)
/*
    ** Configures Main PLL
    ** HSI as clock input
    ** fvco = 336MHz
    ** fpllout = 84MHz
    ** fusb = 48MHz
    ** PLLM = 16
    ** PLLN = 336
    ** PLLP = 4
    ** PLLQ = 7
    */
#define RCC_PLLCFGR_PLL (0x02000000ul|(7ul << 24)|(1ul << 16)|(336ul << 6)|(16ul << 0))
/*
** FLASH Configuration block
** - enable instruction cache
** - enable prefetch
** - set latency to 2 Wait States (3 CPU cycles)
*/
#define FLASH_ACR_CFG ((1ul << 9)|(1ul << 8)|(2ul))
#define RCC_CFGR_SW (2ul)
#define RCC_CFGR_HPRE (0xFul << 4)
#define RCC_CFGR_PPRE1 (0x7ul << 10)
#define RCC_CFGR_PPRE2 (0x7ul << 13)

#define DBGMCU_CR_SLEEP (0x1ul)
#define DBGMCU_CR_STOP  (0x1ul << 1)
#define DBGMCU_CR_STANBY (0x1ul << 2)

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

/* System init function */
void SystemInit (void);

/* main function prototype */
extern int main (void);

/* Reset Handler prototype */
void Reset_Handler (void);

/* Interrupt and exceptions handlers: from GNU collections page 601 (GCC-14.2)
** Provide 'weak' aliases for each exception handler to the deault handler.
** As they are 'weak' aliasses, any function with the same name will override
** this definition.
** The alias attribute causes the declaration to be emitted as an alias for another
** symbol, which must have been previously declared with the same type, and for
** variables, also the same size and aligment.
** Declaring an alias with different type than the target is undefined and my be
** diagnosed.
** e.g.
** void __f () {}
** void f () __attribute__ ((weak,alias ("__f")))
*/
/* Exceptions */
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
    (void)SystemInit();
    /*Calling main function.*/
    (void)main();
    while(1);
}

void SystemInit (void)
{
    /* Power interface clock enable during sleep */
    RCC_APB1LPENR |= RCC_APB1LPENR_PWRLPEN;
    /* Regulator voltage scaling output selection: Scale 2. */
    PWR_CR |= PWR_CR_VOS;
    /* Wait until HSI ready. */
    while(0u == (RCC_CR & RCC_CR_HSIRDY));
    /* Store calibration value. */
    PWR_CR |= PWR_CR_PLS;
    /* Disble main PLL. */
    RCC_CR &= ~(RCC_CR_PLLON);
    /* Wait until PLL ready is unlocked. */
    while (0u != (RCC_CR & RCC_CR_PLLRDY));
    /* Configure main PLL. */
    RCC_PLLCFGR = RCC_PLLCFGR_PLL;
    /* PLL ON. */
    RCC_CR |= RCC_CR_PLLON;
    /* Wait until PLL ready is locked. */
    while (0u == (RCC_CR & RCC_CR_PLLRDY));
    /* Flash configuration block. */
    FLASH_ACR |= FLASH_ACR_CFG;
    /* Set clock source to PLL. */
    RCC_CFGR |= RCC_CFGR_SW;
    /* Check system clock switch status. */
    while (RCC_CFGR_SW != (RCC_CFGR & RCC_CFGR_SW));
    /* Set HCLK (AHB1) prescaler (DIV1) */
    RCC_CFGR &= ~RCC_CFGR_HPRE;
    /* Set APB1 Low speed prescaler (DIV2) */
    RCC_CFGR |= RCC_CFGR_PPRE1;
    /* Set APB2 High speed prescaler DIV1 */
    RCC_CFGR &= ~RCC_CFGR_PPRE2;
    /* Allow debug even during sleep modes */
    DBGMCU_CR |= (DBGMCU_CR_SLEEP|DBGMCU_CR_STOP|DBGMCU_CR_STANBY);
    /* System core clock is: 84 000 000 Hz. */
}
/*
** main.c file made by Carlos Martinez
*/
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
** (1u<<10)    // Set bit 10 to 1.
** &=~(1u<<11) // Set bit 11 to 0.
*/

#define MODER5 (GPIOx_MODER_OUTPUT << 10) /* Could be any of the other three options */

#define GPIOA_ODR_OFFSET (0x14ul)
#define GPIOA_ODR (*(volatile unsigned int*)(GPIOA_BASE + GPIOA_ODR_OFFSET))

#define ODR5 (1U << 5)
#define PA5 (ODR5)
#define LED_PIN (PA5)

const unsigned int delay = 1000000u;

unsigned int speed;

int main (void)
{
    /* Enable clock access to GPIOA */
    RCC_AHB1ENR |= GPIOAEN;
    /* Set PA5 as output*/
    GPIOA_MODER |= MODER5;
    while(1u)
    {
        /* Set PA5 High */
        GPIOA_ODR |= LED_PIN;
        for(int i=0;i<delay;i++);
        /* Set PA5 low */
        GPIOA_ODR &= ~LED_PIN;
        for(int i=0;i<delay;i++);
    }
}
#include "sys.h"

void power_control_pin_on(void)
{
    GPIOE->ODR |= 1<<2;
}

void power_control_pin_off(void)
{
    GPIOE->ODR &= (~(1<<2));
}

void power_control_pin_init(void)
{
    RCC->APB2ENR |= 1<<6;

    GPIOE->CRL &= 0XFFFFF0FF;
    GPIOE->CRL |= 0X00000500;
    power_control_pin_off();
}


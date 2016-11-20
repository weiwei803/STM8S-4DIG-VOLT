/* VERSION 2.00 */
/*  09.12.2012  */

#include "STM8_GPIO.h"

void PIN_ON(GPIO_TypeDef * GPIOx,u8 PINx)
{
	GPIOx->ODR=GPIOx->IDR|PINx;
}

void PIN_ON_LATCH(GPIO_TypeDef * GPIOx,u8 PINx)
{
	GPIOx->ODR=GPIOx->ODR|PINx;
}

void PIN_OFF(GPIO_TypeDef * GPIOx,u8 PINx)								
{
	GPIOx->ODR=GPIOx->IDR&(~(PINx));
}

void PIN_OFF_LATCH(GPIO_TypeDef * GPIOx,u8 PINx)								
{
	GPIOx->ODR=GPIOx->ODR&(~(PINx));
}

u8 PIN_SYG(GPIO_TypeDef * GPIOx, u8 PINx)
{
        return GPIOx->IDR&PINx;
//        if (GPIOx->IDR&PINx!=0) return 1;
//          else return 0;
}

//inline 
void PIN_IN (GPIO_TypeDef * GPIOx,u8 PINx)
{
        GPIOx->DDR&=(~PINx);
}

void PIN_OUT_PP (GPIO_TypeDef * GPIOx,u8 PINx)
{
        GPIOx->DDR|=PINx;
}

void PIN_INV(GPIO_TypeDef * GPIOx, u8 PINx)
{
	GPIOx->ODR=GPIOx->IDR^PINx;
}



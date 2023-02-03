#include "LED.h"


uint16_t LED_ALL=0xFFFF;

void LEDx_ON(uint16_t n)
{
	LED_ALL&=(0xFEFF<<(n-1))|(0xFEFF>>(16-(n-1)));
	GPIOC->ODR=LED_ALL;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

 
void LEDx_OFF(uint16_t n)
{
	LED_ALL|=(0x0100<<(n-1))|(0x0100>>(16-(n-1)));
	GPIOC->ODR=LED_ALL;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void LED_All_Close(void)
{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);
		
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

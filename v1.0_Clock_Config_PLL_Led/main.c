#include "stm32f446xx.h"

#define PLL_M 4
#define PLL_N 180
#define PLL_P 0

void SysClockConfig(void);
void GPIO_Config(void);
void delay(uint32_t count);

int main(void){
	SysClockConfig();
	GPIO_Config();
	
	while(1){
		GPIOA->BSRR |= (1 << 5); // set PA5
		delay(10000000);
		GPIOA->BSRR |= (1 << 21); // reset PA5
		delay(10000000);
		
	}
		
	return 0;

}

void delay(uint32_t count){
	while(count){
		count--;
	}
}

void GPIO_Config(void){
	/********* How to configure GPIO **************
	1. Enable the GPIO clock (In this program we use GPIOA)
	2. Set the pin 5 as output
	3. Configure the output mode
	***********************************************/
	
	// 1. Enable the GPIO clock (In this program we use GPIOA)
	RCC->AHB1ENR |= (1 << 0);
	
	// 2. Set the pin 5 as output
	GPIOA->MODER |= (1 << 10);
	
	// 3. Configure the output mode
	GPIOA->OTYPER = 0; //output type push-pull
	GPIOA->OSPEEDR = 0; //Low speed mode selected
	
	
}

void SysClockConfig(void){
	
	/********** How to configure clock setup *******************
	1. Enable HSE (External Crystal) and wait for the HSE become ready
	2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
	3. Configure the flash prefetch and latency related settings
	4. Configure the Prescalars, Hclk, pclk1 and pclk2
	5. Configure the main PLL is in "RCC PLL configuration register (RCC_PLL CFGR)"
	6. Enable the PLL and wait for it become ready
	7. Select the clock source and wait for it become ready
	****************************************************************/
	
	// Enable HSE (External Crystal) and wait for the HSE become ready
	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY));
	
	// 2. Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;
	
	// 3. Configure the flash prefetch and latency related settings
	FLASH->ACR |= FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
	
	// 4. Configure the Prescalars, Hclk, pclk1 and pclk2
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
	
	// 5. Configure the main PLL is in "RCC PLL configuration register (RCC_PLL CFGR)"
	RCC->PLLCFGR = (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | (RCC_PLLCFGR_PLLSRC_HSE);
	
	// 6. Enable the PLL and wait for it become ready
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));
	
	// 7. Select the clock source and wait for it become ready
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	
}

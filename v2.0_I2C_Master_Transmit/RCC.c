#include "RCC.h"

void sysClockConfig(void){
	
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
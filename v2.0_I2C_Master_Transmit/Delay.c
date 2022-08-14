
#include "Delay.h"

void TIM6Config(void){
	/*************Steps to Follow*************************
	1. Enable timer clock
	2. Set the prescalar and ARR
	3. Enable the timer and wait for the update flag to set
	****************************************************************/
	
	// 1. Enable timer clock
	RCC->APB1ENR |= (1 << 4);
	
	// 2. Set the prescalar and ARR
	TIM6->PSC = 90-1; // prescalar set to 1MHz ~ 1uS because APB1 timer clock is 90MHz
	TIM6->ARR = 0xffff; // set ARR value full 16 bit to count
	
	// 3. Enable the timer and wait for the update flag to set
	TIM6->CR1 |= (1 << 0); // Enable the counter
	while(!(TIM6->SR & (1 << 0))); // update interrupt flag


}

void Delay_us(uint16_t us){
	// 1. timer6 counter set to 0
	TIM6->CNT = 0;
	
	// 2. wait the counter value until reach the entered value.
	/// As each count take 1us, total wait time will be the requried value
	while(TIM6->CNT < us);
	
}

void Delay_ms(uint16_t ms){
	for(uint16_t i=1; i<ms; i++){
		Delay_us(1000); // delay for 1 ms
	}
}

#include "RCC.h"
#include "Delay.h"

void Uart2Config(void){
	/******STEPS TO FOLLOW***************
	1. Enable the uart clock and gpio clock
	2. Configure the uart pins for alternate function
	3. Enable the USART by writting the UE bit of USART_CR1 to 1
	4. Program the M bit of USART_CR1 to define the world length
	5. Select the desire boud rate by using USART_BRR register
	6. Enable the Transmitter / Receiver by setting the TE/RE bits of USART_CR1 register
	***********************************/
	
	// 1. Enable the uart clock and gpio clock
	RCC->APB1ENR |= (1 << 17);
	RCC->AHB1ENR |= (1 << 0);
	
	// 2. Configure the uart pins for alternate function
	GPIOA->MODER |= (2 << 4);
	GPIOA->MODER |= (2 << 6);
	
	GPIOA->OSPEEDR |= (3 << 4) | (3 << 6);
	
	GPIOA->AFR[0] |= (7 << 8);
	GPIOA->AFR[0] |= (7 << 12);
	
	// 3. Enable the USART by writting the UE bit of USART_CR1 to 1
	USART2->CR1 = 0x00; //cleare all
	USART2->CR1 |= (1 << 13);
	
	// 4. Program the M bit of USART_CR1 to define the world length
	USART2->CR1 &= ~(1 << 12);
	
	// 5. Select the desire boud rate by using USART_BRR register
	USART2->BRR = (7 << 0) | (24 << 4); // boud rate 115200 at 45MHz
	
	// 6. Enable the Transmitter / Receiver by setting the TE/RE bits of USART_CR1 register
	USART2->CR1 |= (1 << 2); //receiver enable
	USART2->CR1 |= (1 << 3); //transmitter enable
	
}

void UART2_SendChar(uint8_t c){
	/************* STEPS TO FOLLOW ************
	1. write the data to send in the usart_dr register (this clear the TXE bit). repeat this
	for each data to be transmitted in case of single buffer.
	2. after writing the last data into usart_dr register, wait until TC=1. this indicates
	that the transmission of the last frame is complete. this is required for instance when
	the usart is disable or enters the halt mode to avoid corrupting the last transmission.
	*******************************************/
	USART2->DR = c;
	while(!(USART2->SR & (1 << 6)));
}

void UART2_SendString(char *string){
	
	while(*string){
		UART2_SendChar(*string++);
	}
}

uint8_t UART2_GetChar(void){
	/*****Steps to Follow*******
	1. Wait for the RXNE bit to set. It indicates that the data has been received and can read.
	2. Read the data from USART_DR register. this also clears the RXNE bit.
	****************************/
	
	uint8_t temp;
	while(!(USART2->SR & (1 << 5)));
	temp = USART2->DR;
	return temp;
}

int main(void){
	sysClockConfig();
	TIM6Config();
	Uart2Config();
	
	
	while(1){
		//UART2_SendString("Hasan");
		
		uint8_t data  = UART2_GetChar();
		UART2_SendChar(data);
		//UART2_SendChar('f');
		
		//Delay_ms(1000);
	}

	return 0;
}

#include "I2C.h"

void I2C_Config(void){
  // 1. Enable the i2c clock and gpio clock
	RCC->APB1ENR |= (1 << 21); // enable i2c1
	RCC->AHB1ENR |= (1 << 1); //enable port b
	
	// 2. configure the i2c pin for alternate function
	GPIOB->MODER |= (2 << 16) | (2 << 18); //configure the pin PB8 and PB9 as SCL and SDA
	GPIOB->OTYPER |= (1 << 8) | (1 << 9); //output type open-drain
	GPIOB->OSPEEDR |= (3 << 16) | (3 << 18); // high speed
	GPIOB->PUPDR |= (1 << 16) | (1 << 18); // output  pull-up
	GPIOB->AFR[1] |= (4 << 0) | (4 << 4); // alternet function AF4 for 8 and 9 bit of GPIOB
	
	// 3. Reset the i2c
	I2C1->CR1 |= (1 << 15);
	I2C1->CR1 &= ~(1 << 15);
	
	// 4. Program the peripheral input clock in I2C_CR2 register in order to generate current timming
	I2C1->CR2 |= (45 << 0); //45MHz as APB clock frequency
	
	// 5. Configure the clock control register
	I2C1->CCR &= ~(1 << 15); // Sm mode
	I2C1->CCR &= ~(1 << 14); // duty is also 0, as we not use the fast mode
	I2C1->CCR = 225 << 0; // CCR = (Tr(scl) + Tw(sclh)) / Tpclk1 = 1000ns + 4000ns / 22.22ns = 225
	I2C1->TRISE = 46; // TRISE = (Tr(SCL) / Tpclk1) + 1 = (1000 ns / 22.22 ns) + 1 = 46
	
	// 6. Configure the 12c_cr1 register to enable the peripharal
	I2C1->CR1 |= (1 << 0);



}

void I2C_Start(void){
 // 1. Send the start condition
	I2C1->CR1 |= (1 << 8);
	
	// 2. wait for the SB bit to set (bit 0 in SR1). This indicate that start condition is generated
	while(!(I2C1->SR1 & (1 << 0))); 
}

void I2C_Write(uint8_t data){
	// 1. check if TxE bit is set
	while(!(I2C1->SR1 & (1 << 7)));
	// 2. send the data to DR register
	I2C1->DR = data;
	// 3. wait for the BTF bit is set. that indicate that the end of last data transfer
	while(!(I2C1->SR1 & (1 << 2))); // wait until BTF is set
	
}

void I2C_Address(uint8_t address){
	// 1. send the address to DR register
	I2C1->DR = address;
	// 2. Address sent (master mode)
	while(!(I2C1->SR1 & (1 << 1)));
	// 3. clear by reading sr1 and sr2 register
	uint8_t temp = I2C1->SR1 | I2C1->SR2;
	
}

void I2C_Stop(void){
	// stop condition generation
	I2C1->CR1 |= (1 << 9);
}

void I2C_WriteMulti(uint8_t *data, uint8_t size){
	while(!(I2C1->SR1 & (1 << 7))); // wait for TxE bit to set
	while(size){
		while(!(I2C1->SR1 & (1 << 7))); // wait for TxE bit to set
		I2C1->DR = (volatile uint32_t) *data++; // send data to DR register
		size--;
	}
	while(!(I2C1->SR1 & (1 << 2))); // wait for BTF bit to set
	
}
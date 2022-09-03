
#include "I2C.h"

//static void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr);
//static void I2C_ExecuteAddressPhaseRead(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr);
//static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle );
//static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle );
//static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle );

/*
static void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}


static void I2C_ExecuteAddressPhaseRead(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	//check for device mode
	if(I2C1->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ManageAcking(DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = I2C1->SR1;
				dummy_read = I2C1->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = I2C1->SR1;
			dummy_read = I2C1->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = I2C1->SR1;
		dummy_read = I2C1->SR2;
		(void)dummy_read;
	}


}

void I2C_ManageAcking(uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		I2C1->CR1 |= ( 1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		I2C1->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}

 void I2C_GenerateStopCondition()
{
	I2C1->CR1 |= ( 1 << I2C_CR1_STOP);
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	I2C1->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	I2C1->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = 0;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	I2C1->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	I2C1->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = 0;
	pI2CHandle->TxLen = 0;
}
*/


void I2C_Config(I2C_TypeDef *pI2Cx){
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
	pI2Cx->CR1 |= (1 << 15);
	pI2Cx->CR1 &= ~(1 << 15);
	
	// 4. Program the peripheral input clock in I2C_CR2 register in order to generate current timming
	pI2Cx->CR2 |= (45 << 0); //45MHz as APB clock frequency
	
	// 5. Configure the clock control register
	pI2Cx->CCR &= ~(1 << 15); // Sm mode
	pI2Cx->CCR &= ~(1 << 14); // duty is also 0, as we not use the fast mode
	pI2Cx->CCR = 225 << 0; // CCR = (Tr(scl) + Tw(sclh)) / Tpclk1 = 1000ns + 4000ns / 22.22ns = 225
	pI2Cx->TRISE = 46; // TRISE = (Tr(SCL) / Tpclk1) + 1 = (1000 ns / 22.22 ns) + 1 = 46
	
	// 6. Configure the 12c_cr1 register to enable the peripharal
	pI2Cx->CR1 |= (1 << 0);



}

void I2C_Start(I2C_TypeDef *pI2Cx){
	/**** STEPS FOLLOWED  ************
1. Enable the ACK
2. Send the START condition 
3. Wait for the SB ( Bit 0 in SR1) to set. This indicates that the start condition is generated
*/
	pI2Cx->CR1 |= (1<<10);  // 1. Enable the ACK
 // 2. Send the start condition
	pI2Cx->CR1 |= (1 << 8);
	
	// 3. wait for the SB bit to set (bit 0 in SR1). This indicate that start condition is generated
	while(!(pI2Cx->SR1 & (1 << 0))); 
}

void I2C_Write(I2C_TypeDef *pI2Cx, uint8_t data){
	// 1. check if TxE bit is set
	while(!(pI2Cx->SR1 & (1 << 7)));
	// 2. send the data to DR register
	pI2Cx->DR = data;
	// 3. wait for the BTF bit is set. that indicate that the end of last data transfer
	while(!(pI2Cx->SR1 & (1 << 2))); // wait until BTF is set
	
}

void I2C_Address(I2C_TypeDef *pI2Cx, uint8_t address){
	// 1. send the address to DR register
	pI2Cx->DR = address;
	// 2. Address sent (master mode)
	while(!(pI2Cx->SR1 & (1 << 1)));
	// 3. clear by reading sr1 and sr2 register
	uint8_t temp = pI2Cx->SR1 | pI2Cx->SR2;
	
}

void I2C_Stop(I2C_TypeDef *pI2Cx){
	// stop condition generation
	pI2Cx->CR1 |= (1 << 9);
}

void I2C_WriteMulti(I2C_TypeDef *pI2Cx, uint8_t *data, uint8_t size){
	while(!(pI2Cx->SR1 & (1 << 7))); // wait for TxE bit to set
	while(size){
		while(!(pI2Cx->SR1 & (1 << 7))); // wait for TxE bit to set
		pI2Cx->DR = (volatile uint32_t) *data++; // send data to DR register
		size--;
	}
	while(!(pI2Cx->SR1 & (1 << 2))); // wait for BTF bit to set
	
}

void I2C_Read(I2C_TypeDef *pI2Cx, uint8_t address, uint8_t *buffer, uint8_t size){
	uint8_t remaning = size;
	if(size == 1){
		/**** If single byte need to read ****/
		// (a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
		pI2Cx->DR = address;
		while(!(pI2Cx->SR1 & (1 << 1)));
		
		//(b) the Acknowledge disable is made during EV6 (before ADDR flag is cleared) and the STOP condition generation is made after EV6
		pI2Cx->CR1 &= ~(1 << 10);
		uint8_t temp = pI2Cx->SR1 | pI2Cx->SR2;
		pI2Cx->CR1 |= (1 << 9);
		
		// (c) Wait for the RXNE (Receive Buffer not Empty) bit to set
		while(!(pI2Cx->SR1 & (1 << 6)));
		
		// (d) Read the data from the DR
		buffer[size-remaning] = pI2Cx->DR;
	} else {
		/*** If Multiple BYTES needs to be read ***/
		// (a) Write the slave Address, and wait for the ADDR bit (bit 1 in SR1) to be set
		pI2Cx->DR = address;
		while(!(pI2Cx->SR1 & (1 << 1)));
		
		// (b) Clear the ADDR bit by reading the SR1 and SR2 Registers
		uint8_t temp = pI2Cx->SR1 | pI2Cx->SR2;
		
		while(remaning > 2){
			// (c) Wait for the RXNE (Receive Buffer not Empty) bit to set
			while(!(pI2Cx->SR1 & (1 << 6)));
		
			// (d) Read the data from the DR
			buffer[size-remaning] = pI2Cx->DR;
			
			//e) Generate the Acknowlegment by settint the ACK (bit 10 in CR1)
			pI2Cx->CR1 |= (1 <<10);
			remaning--;
		}
		
			// Wait for the RXNE (Receive Buffer not Empty) bit to set
			while(!(pI2Cx->SR1 & (1 << 6)));
		
			// Read the data from the DR
			buffer[size-remaning] = pI2Cx->DR; // Read the SECOND LAST BYTE
		
		// (f) To generate the nonacknowledge pulse after the last received data byte, the ACK bit must be cleared just after reading the 
		// second last data byte (after second last RxNE event)
		pI2Cx->CR1 &= ~(1 <<10); // clear the ACK bit
		
		// (g) In order to generate the Stop/Restart condition, software must set the STOP/START bit 
	   // after reading the second last data byte (after the second last RxNE event)
		pI2Cx->CR1 |= (1 << 9); // Stop I2C
		remaning --;
		
		// read the last byte
		while(!(pI2Cx->SR1 & (1 << 6)));
		buffer[size-remaning] = pI2Cx->DR;
	
	}

}

/*
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{

	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data in to DR
		I2C1->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;

	}

}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = I2C1->DR;
		pI2CHandle->RxLen--;

	}


	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(DISABLE);
		}

			//read DR
			*pI2CHandle->pRxBuffer = I2C1->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition();

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}


void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_Start();

		//Implement the code to enable ITBUFEN Control Bit
		I2C1->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		I2C1->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		I2C1->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

int8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_Start();

		//Implement the code to enable ITBUFEN Control Bit
		I2C1->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		I2C1->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		I2C1->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;
	

	temp1   = I2C1->CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2   = I2C1->CR2 & ( 1 << I2C_CR2_ITBUFEN);

	temp3  = I2C1->SR1 & ( 1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//I2C_ExecuteAddressPhaseWrite(pI2CHandle->DevAddr);
		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			//I2C_ExecuteAddressPhaseRead(pI2CHandle->DevAddr);
		}
	}

	temp3  = I2C1->SR1 & ( 1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3  = I2C1->SR1 & ( 1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set .
			if(I2C1->SR1 & ( 1 << I2C_SR1_TXE) )
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0 )
				{
					//1. generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition();

					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);

				}
			}

		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			;
		}
	}

	temp3  = I2C1->SR1 & ( 1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode
	if(temp1 && temp3)
	{
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )

		I2C1->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}


	temp3  = I2C1->SR1 & ( 1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(I2C1->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if(I2C1->SR2 & ( 1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		    }
		}
	}

	temp3  = I2C1->SR1 & ( 1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//check device mode .
		if(I2C1->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//The device is master

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}

		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(I2C1->SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (I2C1->CR2) & ( 1 << I2C_CR2_ITERREN);


///Check for Bus error
	temp1 = (I2C1->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		I2C1->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

///Check for arbitration lost error
	temp1 = (I2C1->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		I2C1->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

//Check for ACK failure  error

	temp1 = (I2C1->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		I2C1->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

//Check for Overrun/underrun error
	temp1 = (I2C1->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		I2C1->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

//Check for Time out error
	temp1 = (I2C1->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		I2C1->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}
*/

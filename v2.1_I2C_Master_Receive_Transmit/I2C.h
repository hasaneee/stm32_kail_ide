
#include "stdint.h"
#include "RCC.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;

}I2C_Config_t;

/*
 *Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_TypeDef *pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
  uint32_t        RxSize;		/* !< To store Rx size  > */
  uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_Handle_t;

/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 			1
#define I2C_BUSY_IN_TX 			2


/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE        1
#define I2C_ACK_DISABLE       0

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET



void I2C_Config(I2C_TypeDef *pI2Cx);
void I2C_Start(I2C_TypeDef *pI2Cx);
void I2C_Write(I2C_TypeDef *pI2Cx, uint8_t data);
void I2C_Address(I2C_TypeDef *pI2Cx, uint8_t address);
void I2C_Stop(I2C_TypeDef *pI2Cx);
void I2C_WriteMulti(I2C_TypeDef *pI2Cx, uint8_t *data, uint8_t size);
void I2C_Read(I2C_TypeDef *pI2Cx, uint8_t address, uint8_t *buffer, uint8_t size);
//void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
//void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
//uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
//int8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
//void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
//void I2C_GenerateStopCondition();
//void I2C_ManageAcking(uint8_t EnorDi);
//void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
//void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
//void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);



/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);


#include "RF98.h"
#include "main.h"
extern SPI_HandleTypeDef hspi2;

RHMode _mode;
uint16_t _cad_timeout = 10000;
uint8_t LoRa_buff[RH_RF98_FIFO_SIZE] = {0};

#if Header == Header_used
char _txHeaderTo = 0;
#endif

#define Set_Pin(port, pin) HAL_GPIO_WritePin(port, pin, 1)
#define Reset_Pin(port, pin) HAL_GPIO_WritePin(port, pin, 0)

char MODEM_CONFIG_TABLE[5][3] =
{
    //  1d,     1e,      26
    { 0x72,   0x74,    0x00}, // Bw125Cr45Sf128 (the chip default)
    { 0x92,   0x74,    0x00}, // Bw500Cr45Sf128
    { 0x48,   0x94,    0x00}, // Bw31_25Cr48Sf512
    { 0x78,   0xc4,    0x00}, // Bw125Cr48Sf4096
		{ 0x73,   0x74,    0x00}, // IH_Bw125Cr45Sf128 (the chip default + Implicit header)
};

HAL_StatusTypeDef err;
HAL_StatusTypeDef RF98_write(char reg, char wValue)
{
	char buff[2]={0};
	
	buff[0] = W | reg;
	buff[1] = wValue;

	HAL_GPIO_WritePin(CS_RFM96_GPIO_Port, CS_RFM96_Pin, GPIO_PIN_RESET);
	err = HAL_SPI_Transmit(&hspi2, (uint8_t*)&buff, 2, 100);
	HAL_GPIO_WritePin(CS_RFM96_GPIO_Port, CS_RFM96_Pin, GPIO_PIN_SET);
	
	return err;
}


HAL_StatusTypeDef RF98_write_burst(char reg, uint8_t* data)
{
	int length = 0;
	uint8_t cmd = W | reg;
	HAL_StatusTypeDef err;
	
	length = strlen((const char*)data);
	
	HAL_GPIO_WritePin(CS_RFM96_GPIO_Port, CS_RFM96_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)&cmd, 1, 100);
	err = HAL_SPI_Transmit(&hspi2, (uint8_t*)&LoRa_buff, length, 100);
	HAL_GPIO_WritePin(CS_RFM96_GPIO_Port, CS_RFM96_Pin, GPIO_PIN_SET);
	
	return err;
}


char RF98_read(char reg)
{
	char buff = R & reg;

	HAL_GPIO_WritePin(CS_RFM96_GPIO_Port, CS_RFM96_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)&buff, 1, 100);
	HAL_SPI_Receive(&hspi2, (uint8_t*)&buff, 1, 100);
	HAL_GPIO_WritePin(CS_RFM96_GPIO_Port, CS_RFM96_Pin, GPIO_PIN_SET);
	
	return buff;
}


HAL_StatusTypeDef RF98_read_burst(char reg, char* buffer, int length)
{
	buffer[0] = R & reg;
	HAL_StatusTypeDef err;
	
	HAL_GPIO_WritePin(CS_RFM96_GPIO_Port, CS_RFM96_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)buffer, 1, 100);
	err = HAL_SPI_Receive(&hspi2, (uint8_t*)buffer, length, 100);
	HAL_GPIO_WritePin(CS_RFM96_GPIO_Port, CS_RFM96_Pin, GPIO_PIN_SET);
	
	return err;
}


void RF98_Reset(void)
{
	HAL_GPIO_WritePin(RES_RFM69_GPIO_Port, RES_RFM69_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(RES_RFM69_GPIO_Port, RES_RFM69_Pin, GPIO_PIN_SET);
}

uint8_t rbuff = 0;
bool RF98_Init(void)
{
	RF98_Reset();

	// Set sleep mode, so we can also set LORA mode:
	RF98_sleep();
	
    RF98_write(RH_RF98_REG_01_OP_MODE, RH_RF98_MODE_SLEEP | RH_RF98_LONG_RANGE_MODE);
    HAL_Delay(20); // Wait for sleep mode to take over from say, CAD

    // Check we are in sleep mode, with LORA set
		rbuff = RF98_read(RH_RF98_REG_01_OP_MODE);
    if (rbuff != (RH_RF98_MODE_SLEEP | RH_RF98_LONG_RANGE_MODE))
    {
        return false; // No device present?
    }

    // Set up FIFO
    // We configure so that we can use the entire 256 byte FIFO for either receive
    // or transmit, but not both at the same time
    RF98_write(RH_RF98_REG_0E_FIFO_TX_BASE_ADDR, 0);
    RF98_write(RH_RF98_REG_0F_FIFO_RX_BASE_ADDR, 0);

    // Packet format is preamble + explicit-header + payload + crc
    // Explicit Header Mode
    // payload is TO + FROM + ID + FLAGS + message data
    // RX mode is implmented with RXCONTINUOUS
    // max message data length is 255 - 4 = 251 octets

    RF98_setModeIdle();

    // Set up default configuration
    // No Sync Words in LORA mode.
	RF98_setModemConfig(Bw125Cr45Sf128); // Radio default
    RF98_setPreambleLength(8); // Default is 8
    // An innocuous ISM frequency, same as RF22's
    RF98_setFrequency(433.0);
    // Lowish power
    RF98_setTxPower(13, false);

    return true;
}


bool RF98_setModemConfig(ModemConfigChoice index)
{
	RF98_write(RH_RF98_REG_1D_MODEM_CONFIG1, MODEM_CONFIG_TABLE[index][0]);
	RF98_write(RH_RF98_REG_1E_MODEM_CONFIG2, MODEM_CONFIG_TABLE[index][1]);
	RF98_write(RH_RF98_REG_26_MODEM_CONFIG3, MODEM_CONFIG_TABLE[index][2]);

    return true;
}


void RF98_setPreambleLength(uint16_t bytes)
{
    RF98_write(RH_RF98_REG_20_PREAMBLE_MSB, bytes >> 8);
    RF98_write(RH_RF98_REG_21_PREAMBLE_LSB, bytes & 0xff);
}


bool RF98_setFrequency(float centre)
{
    // Frf = FRF / FSTEP
    uint64_t frf = (uint32_t)((uint32_t)centre * 1000000.0) / (uint32_t)RH_RF98_FSTEP;
    RF98_write(RH_RF98_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    RF98_write(RH_RF98_REG_07_FRF_MID, (frf >> 8) & 0xff);
    RF98_write(RH_RF98_REG_08_FRF_LSB, frf & 0xff);

    return true;
}


void RF98_setTxPower(int8_t power, bool useRFO)
{
    // Sigh, different behaviours depending on whther the module use PA_BOOST or the RFO pin
    // for the transmitter output
    if (useRFO)
    {
        if (power > 14)power = 14;
        if (power < -1)power = -1;
        RF98_write(RH_RF98_REG_09_PA_CONFIG, RH_RF98_MAX_POWER | (power + 1));
    }
    else
    {
        if (power > 23)power = 23;
        if (power < 5)power = 5;

        // For RH_RF98_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
        // RH_RF98_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
        // for 21 and 23dBm
        if (power > 20)
        {
            RF98_write(RH_RF98_REG_4D_PA_DAC, RH_RF98_PA_DAC_ENABLE);
            power -= 3;
        }
        else
        {
            RF98_write(RH_RF98_REG_4D_PA_DAC, RH_RF98_PA_DAC_DISABLE);
        }

        // RFM98/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
        // pin is connected, so must use PA_BOOST
        // Pout = 2 + OutputPower.
        // The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
        // but OutputPower claims it would be 17dBm.
        // My measurements show 20dBm is correct
        RF98_write(RH_RF98_REG_09_PA_CONFIG, RH_RF98_PA_SELECT | (power-5));
    }
}


bool RF98_receive(uint8_t* data)
{
	int len = 0;
	
	if(_mode == RHModeRx)
	{
		while(!RF98_available()){}

		if(RF98_Check_PayloadCRCError())
			return false;
		

		len = RF98_read(RH_RF98_REG_13_RX_NB_BYTES);

		// Reset the fifo read ptr to the beginning of the packet
		RF98_write(RH_RF98_REG_0D_FIFO_ADDR_PTR, RF98_read(RH_RF98_REG_10_FIFO_RX_CURRENT_ADDR));
		
		RF98_read_burst(RH_RF98_REG_00_FIFO, (char*)data, len);	

		RF98_setModeIdle();
			
		RF98_Clear_IRQ();

		return true;
	}
	else
		return false;
}


bool RF98_receive_Timeout(uint8_t* buf, uint16_t timeout)
{
	int len = 0;

	if(_mode == RHModeRx)
	{
		if(!RF98_available_Timeout(timeout))
		{
			return false;
		}

		if(RF98_Check_PayloadCRCError())
			return false;


		len = RF98_read(RH_RF98_REG_13_RX_NB_BYTES);

		// Reset the fifo read ptr to the beginning of the packet
		RF98_write(RH_RF98_REG_0D_FIFO_ADDR_PTR, RF98_read(RH_RF98_REG_10_FIFO_RX_CURRENT_ADDR));

		RF98_read_burst(RH_RF98_REG_00_FIFO, (char*)buf, len);

		RF98_setModeIdle();

		RF98_Clear_IRQ();

		if(RF98_Check_PayloadCRCError())
		{
			return false;
		}
		else
		{
			return true;
		}
	}
	else 
		return false;
}


bool RF98_send(uint8_t* data)
{
	int len = strlen((char*)data);

	#if Header == Header_used
	uint16_t header_len = sizeof(_txHeaderTo);
	#endif
	
    if (len > RH_RF98_MAX_MESSAGE_LEN)
			return false;

    RF98_waitPacketSent(); // Make sure we dont interrupt an outgoing message
    RF98_setModeIdle();
		
	if (!RF98_waitCAD())
		return false;  // Check channel activity

	RF98_setModeIdle();

    // Position at the beginning of the FIFO
    RF98_write(RH_RF98_REG_0D_FIFO_ADDR_PTR, 0);

    // The headers
	#if Header == Header_used
	RF98_write(RH_RF98_REG_00_FIFO, _txHeaderTo);
	#endif

    // The message data
    RF98_write_burst(RH_RF98_REG_00_FIFO, data);
		
	#if Header == No_header
	RF98_write(RH_RF98_REG_22_PAYLOAD_LENGTH, len);
	#else
	RF98_write(RH_RF98_REG_22_PAYLOAD_LENGTH, len + header_len);
	#endif
		
//		uint8_t rBuff[15] = {0};
//		uint16_t lenght_ = RF98_read(RH_RF98_REG_10_FIFO_RX_CURRENT_ADDR);
//		RF98_write(RH_RF98_REG_0D_FIFO_ADDR_PTR, lenght_);
//		RF98_read_burst(RH_RF98_REG_00_FIFO,(char*)rBuff, len);

    RF98_setModeTx(); // Start the transmitter
	while(!RF98_Check_TxDone()){}
		
	RF98_setModeIdle();

	RF98_Clear_IRQ();
    return true;
}


bool RF98_waitCAD(void)
{
	if (!_cad_timeout)
		return true;

    // Wait for any channel activity to finish or timeout
    // Sophisticated DCF function...
    // DCF : BackoffTime = random() x aSlotTime
    // 100 - 1000 ms
    // 10 sec timeout
			
	RF98_setModeCAD();
		
    unsigned long t = HAL_GetTick();
    while (!RF98_Check_CADDone())
    {
      if (HAL_GetTick() - t > _cad_timeout) 
	     return false;
    }

    return true;
}


bool RF98_waitPacketSent(void)
{
    if (_mode == RHModeTx)
    {
    	while(!RF98_Check_TxDone());
    }

    return true;
}


bool RF98_available(void)
{	
	while(!RF98_Check_RxDone())
	{
		
	}

	if(RF98_Check_ValidHeader())
	{
			RF98_Clear_IRQ();
			return true;
	}
	else 
		return false;
}


bool RF98_available_Timeout(uint16_t timeout)
{
	unsigned long t = HAL_GetTick();

	while(!RF98_Check_RxDone())
	{
		if (HAL_GetTick() - t > timeout)
			return false;
	}

	if(RF98_Check_ValidHeader())
	{
			RF98_Clear_IRQ();
			return true;
	}
	else
		return false;
}


void RF98_setModeCAD(void)
{
	if (_mode != RHModeCad)
	{
			RF98_write(RH_RF98_REG_01_OP_MODE, RH_RF98_MODE_CAD);
			RF98_write(RH_RF98_REG_40_DIO_MAPPING1, 0x80); // Interrupt on CadDone
			_mode = RHModeCad;
	}
}

uint8_t aux = 0;
void RF98_setModeIdle(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeIdle)
    {
				aux = RF98_read(RH_RF98_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF98_MODE_STDBY;
        RF98_write(RH_RF98_REG_01_OP_MODE, aux);
        _mode = RHModeIdle;
    }
}


bool RF98_sleep(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeSleep)
    {
				aux = RF98_read(RH_RF98_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF98_MODE_SLEEP;
        RF98_write(RH_RF98_REG_01_OP_MODE, aux);
        _mode = RHModeSleep;
    }
    return true;
}


void RF98_setModeRx_Single(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeRx)
    {
				aux = RF98_read(RH_RF98_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF98_MODE_RXSINGLE;
        RF98_write(RH_RF98_REG_01_OP_MODE, aux);
        RF98_write(RH_RF98_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
        _mode = RHModeRx;
    }
}


void RF98_setModeRx_Continuous(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeRx)
    {
				aux = RF98_read(RH_RF98_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF98_MODE_RXCONTINUOUS;
        RF98_write(RH_RF98_REG_01_OP_MODE, aux);
        RF98_write(RH_RF98_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
        _mode = RHModeRx;
    }
}


void RF98_setModeTx(void)
{
//	uint8_t aux = 0;
    if (_mode != RHModeTx)
    {
				aux = RF98_read(RH_RF98_REG_01_OP_MODE);
				aux &= 0xF8;
				aux |= RH_RF98_MODE_TX;
        RF98_write(RH_RF98_REG_01_OP_MODE, aux);
        RF98_write(RH_RF98_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
        _mode = RHModeTx;
    }
}


bool RF98_Check_RxTimeout(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF98_RX_TIMEOUT) >> 7;
	
	return reg_read;
}


bool RF98_Check_RxDone(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF98_RX_DONE) >> 6;
	
	return reg_read;
}


bool RF98_Check_PayloadCRCError(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF98_PAYLOAD_CRC_ERROR) >> 5;
	
	return reg_read;
}


bool RF98_Check_ValidHeader(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_12_IRQ_FLAGS);
	
	reg_read = (reg_read & RH_RF98_VALID_HEADER) >> 4;

	return reg_read;
}


bool RF98_Check_TxDone(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_12_IRQ_FLAGS);
	
	reg_read = (reg_read & RH_RF98_TX_DONE) >> 3;

	return reg_read;
}


bool RF98_Check_CADDone(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF98_CAD_DONE) >> 2;

	return reg_read;
}

bool RF98_Check_FhssChannelChange(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF98_FHSS_CHANGE_CHANNEL) >> 1;
	
	return reg_read;
}


bool RF98_Check_CADDetect(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_12_IRQ_FLAGS);
	reg_read = (reg_read & RH_RF98_CAD_DETECTED);
	
	return reg_read;
}


void RF98_Clear_IRQ(void)
{
	uint8_t irq_flags = 0;
	
	RF98_write(RH_RF98_REG_12_IRQ_FLAGS, 0xFF);
	
	irq_flags = RF98_read(RH_RF98_REG_12_IRQ_FLAGS);
	if(irq_flags != 0)
		RF98_write(RH_RF98_REG_12_IRQ_FLAGS, 0xFF);
}


bool RF98_Check_ModemClear(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF98_MODEM_STATUS_CLEAR) >> 4;
	
	return reg_read;
}


bool RF98_Check_HeaderInfoValid(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF98_MODEM_STATUS_HEADER_INFO_VALID) >> 3;

	return reg_read;
}


bool RF98_Check_RxOnGoing(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF98_MODEM_STATUS_RX_ONGOING) >> 2;

	return reg_read;
}

bool RF98_Check_SignalSyncronized(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF98_MODEM_STATUS_SIGNAL_SYNCHRONIZED) >> 1;
	
	return reg_read;
}


bool RF98_Check_SignalDetect(void)
{
	char reg_read = 0;
	reg_read = RF98_read(RH_RF98_REG_18_MODEM_STAT);
	reg_read = (reg_read & RH_RF98_MODEM_STATUS_SIGNAL_DETECTED);
	
	return reg_read;
}


uint16_t ComputeCRC(uint16_t crc, uint8_t data, uint16_t polynomial)
{
	uint8_t i;
	for(i = 0; i < 8; i++)
	{
		if((((crc & 0x8000) >> 8) | (data & 0x80)) != 0)
		{
			crc <<= 1;
			crc |= polynomial;
		}
		else
		{
			crc <<= 1;
		}
		data <<= 1;
	}
	return crc;
}


uint16_t RF98_ComputeCRC(uint8_t *buffer, uint8_t bufferLength, uint8_t crcType)
{
	uint8_t i;
	uint16_t crc;
	uint16_t polynomial;

	polynomial = (crcType == CRC_TYPE_IBM) ? POLYNOMIAL_IBM : POLYNOMIAL_CCITT;
	crc = (crcType == CRC_TYPE_IBM) ? CRC_IBM_SEED : CRC_CCITT_SEED;

	for(i = 0; i < bufferLength; i++)
	{
		crc = ComputeCRC(crc, buffer[i], polynomial);
	}

	if(crcType == CRC_TYPE_IBM)
	{
		return crc;
	}
	else
	{
		return (uint16_t)(~crc);
	}
}




//========================================================================
//=======================Communication protocol===========================
//========================================================================
/* The communication protocol uses three kinds of symbols
 * 		+ '?' -> to make petitions to a certain node
 * 		+ 'O' -> to say something has been done correctly or to continue
 * 		+ 'X' -> to say something has gone wrong or to stop
 * 	All of them are followed with the name of the node, which is only a number.
 */

void Clear_Buffer(uint8_t* buffer)
{
	memset(buffer, 0, strlen((const char *)buffer));
}

bool RF98_Master_Receive_from_Node(uint16_t node)
{
	uint8_t command[3] = {0};
	uint8_t ACK_command[3] = {0};
	bool error = true; //No error happend
	bool end_flag = 0;

	//Prepare the ACK command
	sprintf((char*)ACK_command, "O%d", node);

	//We prepare the command to send, in this case a request for receiving from the master -> "?X"
	//where the X is the node ID
	sprintf((char*)command, "?%d", node);
	//Request a reception to the node X
	do
	{
		Clear_Buffer(LoRa_buff);
		Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
		HAL_Delay(2000);	//Wait to give time to the slave to change to receive mode
		strcpy((char*)LoRa_buff, (char*)command);
		RF98_send(LoRa_buff);		//Send the request "?x"
		Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);

		while(strcmp((char*)LoRa_buff, (char*)ACK_command) != 0)
		{
			//Receive the data
			Clear_Buffer(LoRa_buff);
			if(RF98_waitCAD())
			{
				Set_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
				RF98_setModeRx_Continuous();
				error = RF98_receive_Timeout(LoRa_buff, 7000); //Receive an answer and if we doesn't receive it in the time
															   //we send again the petition.
				HAL_Delay(1000);
				Reset_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
			}
		}
	} while(error == false);


	//Prepare the command Xx in order to check if the transaction has ended or has to stop
	sprintf((char*)command, "X%d", node);
	//Clear the LoRa buffer used for transactions to prevent errors

	Clear_Buffer(LoRa_buff);
	Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
	HAL_Delay(3000);	//Wait to give time to the slave to change to receive mode
	strcpy((char*)LoRa_buff, (char*)ACK_command);
	RF98_send(LoRa_buff);		//Send the ACK to indicate the slave to send the data
	Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
	Clear_Buffer(LoRa_buff);

	//Receive the data from the node x
	while(end_flag == 0)
	{
		do
		{
			//Receive the data
			Clear_Buffer(LoRa_buff);
			if(RF98_waitCAD())
			{
				Set_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
				RF98_setModeRx_Continuous();
				error = RF98_receive_Timeout(LoRa_buff, 7000);
				HAL_Delay(1000);
				Reset_Pin(RX_LED_GPIO_Port, RX_LED_Pin);

				if(strcmp((char *)LoRa_buff, (char *)command) == 0)
				{
					end_flag = 1;
					break;
				}
			}
			/*=====Insert here the code for processing the data received=====*/

			if(strcmp((char*)LoRa_buff, "This is text message!") == 0)
			{
				Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
				Set_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
				HAL_Delay(3000);
				Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
				Reset_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
			}
			/*===============================================================*/
			if(error == true)
			{
				//Send ACK to the node x
				Clear_Buffer(LoRa_buff);
				Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
				HAL_Delay(2000);	//Wait to give time to the slave to change to receive mode
				strcpy((char*)LoRa_buff, (char *)ACK_command); //Send the ACK indicating we want to continue the transmission
				RF98_send(LoRa_buff);
				Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
			}
		} while(error == false);
	}

	Clear_Buffer(LoRa_buff);
	return true;
}


bool RF98_Master_Receive_from_All_Nodes(void)
{
	for(int i = 0; i < Num_of_Nodes; i++)
	{
		RF98_Master_Receive_from_Node(i + 1);
	}
	return true;
}


bool RF98_Slave_Send(void)
{
	uint8_t command[3] = {0};
	uint8_t ACK_command[3] = {0};
	bool error = true; //No error happend

	//Prepare the ACK command
	sprintf((char*)ACK_command, "O%d", Node_ID);


	//We prepare the command to send, in this case a request for receiving from the master -> "?x"
	//where the x is the node ID
	sprintf((char*)command, "?%d", Node_ID);
	//Wait until a receive request is received from Master
	while(strcmp((char *)LoRa_buff, (char *)command) != 0)
	{
		//Receive the data
		Clear_Buffer(LoRa_buff);
		if(RF98_waitCAD())
		{
			Set_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
			RF98_setModeRx_Continuous();
			RF98_receive(LoRa_buff);
		}
	}
	HAL_Delay(1000);
	Reset_Pin(RX_LED_GPIO_Port, RX_LED_Pin);


	//If a receive request has happened from the Master, we send the ACK to tell the master we are going to send as
	//soon as he send us the ACK
	do
	{
		Clear_Buffer(LoRa_buff);
		Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
		HAL_Delay(2000);
		strcpy((char*)LoRa_buff, (char *)ACK_command);
		RF98_send(LoRa_buff);
		Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
		Clear_Buffer(LoRa_buff);
		//Wait until a receive The ACK from master, which tells us to start sending data
		while(strcmp((char *)LoRa_buff, (char *)ACK_command) != 0)
		{
			//Receive the data
			Clear_Buffer(LoRa_buff);
			if(RF98_waitCAD())
			{
				Set_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
				RF98_setModeRx_Continuous();
				error = RF98_receive_Timeout(LoRa_buff, 7000);
				HAL_Delay(1000);
				Reset_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
			}
		}
	} while(error == false);

	/*=====Insert here the code for send data / processing the data received=====*/

	do
	{
		//Send data to the master
		Clear_Buffer(LoRa_buff);
		Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
		HAL_Delay(3000);
		strcpy((char*) LoRa_buff, "This is text message!");
		RF98_send(LoRa_buff);
		Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);

		//Check for ACK
		Clear_Buffer(LoRa_buff);
		while(strcmp((char *)LoRa_buff, (char *)ACK_command) != 0)
		{
			//Receive the data
			if(RF98_waitCAD())
			{
				Set_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
				RF98_setModeRx_Continuous();
				error = RF98_receive_Timeout(LoRa_buff, 7000);
				HAL_Delay(2000);
				Reset_Pin(RX_LED_GPIO_Port, RX_LED_Pin);
			}
		}
	} while(error == false);


	/*===============================================================*/
	//Prepare the command Xx in order to tell the transaction has ended or has to stop
	Clear_Buffer(LoRa_buff);
	Set_Pin(TX_LED_GPIO_Port, TX_LED_Pin);
	sprintf((char*)LoRa_buff, "X%d", Node_ID);
	HAL_Delay(2000);
	RF98_send(LoRa_buff);
	Reset_Pin(TX_LED_GPIO_Port, TX_LED_Pin);

	Clear_Buffer(LoRa_buff);
	return true;
}


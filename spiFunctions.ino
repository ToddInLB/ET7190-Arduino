

void checkDelay() {
	bool printedYet = false;
	while (micros() < nextCommandTime) {
//		if (!printedYet) {
			//printDebug(F("*****checkDelay is waiting******** "), true);
//			printedYet = true;
//		}
		//NOP;
	}
}
void setDelay(int16_t waitTime) {
	nextCommandTime = micros() + waitTime;
}

void spiWriteRegister(byte Register, byte Val) {
	SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE1));
	checkDelay();
	digitalWrite(etPin, LOW);
	SPI.transfer(CMD_WR_REG); // Write Register     Addr from 0 to 0x7F
	delayMicroseconds(spiT1Val * 2); //Instruction Delay (T1)
	SPI.transfer(Register);
	SPI.transfer(Val);
	delayMicroseconds(2); //Data Transmission (T2)
	digitalWrite(etPin, HIGH);
	setDelay(COMMAND_DELAY);
	SPI.endTransaction();
}

void spiSetRegBit(byte Register, byte Val) {
	SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE1));
	checkDelay();
	digitalWrite(etPin, LOW);
	SPI.transfer(CMD_SET_REG_BIT); // Register Bit set £º Addr:0-63
	delayMicroseconds(spiT1Val); //Instruction Delay (T1)
	SPI.transfer(Register);
	SPI.transfer(Val);
	delayMicroseconds(2); //Data Transmission (T2)
	digitalWrite(etPin, HIGH);
	setDelay(COMMAND_DELAY);
	SPI.endTransaction();
}

void spiClrRegBit(byte Register, byte Val) {
	SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE1));
	checkDelay();
	digitalWrite(etPin, LOW);
	SPI.transfer(CMD_CLR_REG_BIT); // Register Bit set £º Addr:0-63
	delayMicroseconds(spiT1Val); //Instruction Delay (T1)
	SPI.transfer(Register);
	SPI.transfer(Val);
	delayMicroseconds(2); //Data Transmission (T2)
	digitalWrite(etPin, HIGH);
	setDelay(COMMAND_DELAY);
	SPI.endTransaction();
}

/*************************************************************************************
*
* SPI mode - write length (2-16) byte register set (buffer)
*
* Return Value: None
* PData: write data buffer pointer
* DataLen: data length (<= 16)
* Register: Register Group (buffer) address
* Note: 1. Write Buffer command delay time
* 2. Write the length of the group must register (buffer) length, if less
* XG7190 of insufficient length portion is written unknown, it may produce a runtime error
*
************************************************************************************/
void spiWriteRegisterBuff(byte RegBuff, byte DataLen, byte * pData) {
	SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE1));
	checkDelay();
	digitalWrite(etPin, LOW);
	SPI.transfer(CMD_WR_BUFF); // Continuously write data in the entire buffer(the buffer with length equal to or smaller than 16 bytes) 
	delayMicroseconds(spiT1Val); //Instruction Delay (T1)
	SPI.transfer(RegBuff);
	SPI.transfer(DataLen);
	for (byte i = 0; i < DataLen; i++) {
		SPI.transfer(*pData);
		pData++;
	}
	delayMicroseconds(2); //Data Transmission (T2)
	digitalWrite(etPin, HIGH);
	setDelay(COMMAND_DELAY);
	SPI.endTransaction();
}

/*************************************************************************************
*
* SPI mode - the length of 64-262 byte write register group (buffer), the command is only valid for the following five buffers,
* Use this command to write another buffer is ignored.
* (BC_CFTXB / BU_UTXB / BJ_IDIFRCON / BJ_HDFILTER / BU_LIDCON)
*
* Return Value: None
* Data source: CmdDataBuff
* DataLen: data length (> 16 <= 262) data type DWORD
* Register: Register Group (buffer) address
* PData: Pointer stored data
* Notes: 1. BJ_IDIFRCON (256bytes) / BJ_HDFILTER (256bytes) / BU_LIDCON (64bytes)
* The length must write the length of registers (buffers),
* If insufficient. XG7190 of insufficient length portion is written unknown, may produce a runtime error
* If you need to change within a single one of these three data buffers available CMD_WR_BUFF_BYTE command
* 2. BC_CFTXB / BU_UTXB variable length, effective length to be written as required
*
************************************************************************************/
void spiWriteLongBuff(byte RegBuff, word DataLen, byte * pBuff) {
	SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE1));
	checkDelay();
	digitalWrite(etPin, LOW);
	if (DataLen & 0xFF00) {
		SPI.transfer(CMD_WR_LB | 0x01); // Continuously write data in the entire long buffer  (the buffer length is bigger than 16 bytes)
	} else {
		SPI.transfer(CMD_WR_LB); // Continuously write data in the entire long buffer  (the buffer length is bigger than 16 bytes)
	}
	delayMicroseconds(51);// spiT1Val); //Instruction Delay (T1)
	SPI.transfer(RegBuff);
	WORD_BYTES tmpWordBytes;
	tmpWordBytes.Word_Data = DataLen;
	SPI.transfer(tmpWordBytes.byteData[0]);
	for (byte i = 0; i < DataLen; i++) {
		SPI.transfer(*pBuff);
		pBuff++;
	}
	delayMicroseconds(3); //Data Transmission (T2)
	digitalWrite(etPin, HIGH);
	setDelay(COMMAND_DELAY);
	SPI.endTransaction();
}
/*************************************************************************************
*
* SPI long way --SpiWriteByteOfLongBuff write a byte buffer
* This command is only valid for the following three buffer (BJ_IDIFRCON / BJ_HDFILTER / BU_LIDCON)
* Because it is under the control of a register set, the actual use, you may need to change a single one of the bytes of control.
* Use this command to write another set of registers (buffers) being ignored.
*
* Return Value: None
* Val: write data values
* Addr: address buffer offset 0
* Register: Register Group (buffer) address (BJ_IDIFRCON / BJ_HDFILTER / BU_LIDCON)
* Note: addr can not be larger than the actual size of the buffer
*
************************************************************************************/
void spiWriteByteOfLongBuff(byte RegBuff, byte addr, byte val) {
	SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE1));
	checkDelay();
	digitalWrite(etPin, LOW);
	SPI.transfer(CMD_WR_BUFF_BYTE); // Write one Byte in Long Buffer
	delayMicroseconds(spiT1Val); //Instruction Delay (T1)
	SPI.transfer(RegBuff);
	SPI.transfer(addr);
	SPI.transfer(val);
	delayMicroseconds(2); //Data Transmission (T2)
	digitalWrite(etPin, HIGH);
	setDelay(LONG_BUFFER_DELAY);
	SPI.endTransaction();
}

void spiFillByteOfLongBuff(byte RegBuff, byte val) {
	SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE1));
	checkDelay();
	digitalWrite(etPin, LOW);
	SPI.transfer(CMD_FILL_BUFF); // Fill the entire buffer with a fixed value
	delayMicroseconds(spiT1Val); //Instruction Delay (T1)
	SPI.transfer(RegBuff);
	SPI.transfer(val);
	delayMicroseconds(2); //Data Transmission (T2)
	digitalWrite(etPin, HIGH);
	setDelay(LONG_BUFFER_DELAY);
	SPI.endTransaction();
}

/*************************************************************************************
*
* UART mode - register read, watch read register delay 10us
* Return Value: Value register
* Register: Register Address
*
************************************************************************************/
byte spiReadRegister(byte Register) {
	byte data[4];
	byte retByte = 0;

	SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE1));
	checkDelay();
	digitalWrite(etPin, LOW);
	retByte = SPI.transfer(CMD_RD_REG | Register); // Read register
	delayMicroseconds(spiT1Val); //Instruction Delay (T1)
	data[0] = SPI.transfer(0x00);

	delayMicroseconds(2); //Data Transmission (T2)
	digitalWrite(etPin, HIGH);
	setDelay(COMMAND_DELAY);
	SPI.endTransaction();
	return data[0];
}
/*************************************************************************************
*
* SPI mode - read register group (buffer), the length 2-262 byte fixed-length register set
* In addition to the receive buffer BU_URX XG7190 UART bus can not be read using this method
* (64-262 buffer) XG7190 response time is not greater than 50us
* (16-byte buffer) XG7190 response time is not greater than 10us
*
* Return Value: Data length
* DataLen: data length (2 - 262) data type DWORD, the value must be read when the length of the buffer.
* Register: Register Group (buffer) address
* PBuff: stored data pointer
*
* Notes: 1. ET7190 UART bus receive buffer (BU_URX) can not use this method to read,
* Because BU_URX received no fixed length format, it should be followed according to the received first "word" to determine
* 2. CAN J1850 received regardless of how long the actual data, ET7190 have been transferred to full 16-byte buffer
* If the CAN / J1850 receive buffer (FIFO empty) No data, the return value of 16 bytes of all zeros
* 3. BU_URX ET7190 UART reception buffer read method, reference SpiRead_URX
*
* ************************************************************************************/
word spiReadRegisterBuff(byte RegBuff, word DataLen, byte* pBuff) {
	byte data[4];
	int16_t tmpDelay = 50;
	word tmpLen = 0;
	byte tmpLocBuffer[16];
	bufferToZero(tmpLocBuffer, 16);
	tmpLen = DataLen;

	SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE1));
	checkDelay();
	digitalWrite(etPin, LOW);
	SPI.transfer(CMD_RD_BUFF | RegBuff); // Read Buffer (Register Set)
	if (DataLen < 17) { tmpDelay = spiT1Val; }
	delayMicroseconds(tmpDelay); //Read Long Buffer Instruction Delay (T3)
	data[0] = SPI.transfer(0x00);
	if (data[0] != (CMD_RD_BUFF | RegBuff)) {
		delayMicroseconds(10);
		digitalWrite(etPin, HIGH);
		setDelay(COMMAND_DELAY);
		SPI.endTransaction();
		return 0;
	}
	for (byte i = 0; i < tmpLen; i++) {
		tmpLocBuffer[i] = SPI.transfer(0x00);
		if (i == 0x01 && RegBuff == BJ_JRXB) {
			tmpLen = 0;
			for (byte i2 = 0; i2 < 4; i2++) {
				bitWrite(tmpLen, i2, bitRead(tmpLocBuffer[i], i2));
			}
			tmpLen += 4;
			DataLen = tmpLen;
		}
		*pBuff = tmpLocBuffer[i];//SPI.transfer(0x00);
		pBuff++;
	}
	if ((currentDebug == typeSPIDetail) && (1 == 0)) {
		printDebug(F("  spiReadRegisterBuff data recieved :: "), true);
		for (byte i = 0; i < DataLen; i++) {
			printDebug((tmpLocBuffer[i]), false);
			printDebug(F(":"), false);
		}
		printDebug();
	}
	delayMicroseconds(3); //Data Transmission (T2)
	digitalWrite(etPin, HIGH);
	setDelay(COMMAND_DELAY);
	SPI.endTransaction();
	return DataLen;
}

/*************************************************************************************
*
* SPI mode - read BU_URX, UART receive buffer, it may reach up to 262 bytes
* Return Value: received length
* Parameter pBuff: Data storage pointer (the buffer should be long enough)
*
* ************************************************************************************/
word spiRead_URX(byte *pBuff) {
	byte data[4];
	byte temp;
	word DataLen;
	SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE1));
	checkDelay();
	digitalWrite(etPin, LOW);
	SPI.transfer(CMD_RD_BUFF | BU_URXB); // Read Buffer (Register Set)
	delayMicroseconds(51); //Read Long Buffer Instruction Delay (T3)
	data[0] = SPI.transfer(0x00);
	if (data[0] != (CMD_RD_BUFF | BU_URXB)) {
		delayMicroseconds(10);
		digitalWrite(etPin, HIGH);
		SPI.endTransaction();
		return 0;
	}
	
	*pBuff = SPI.transfer(0x00); //Read two-byte frame information containing data length
	temp = *pBuff;
	pBuff++;

	*pBuff = SPI.transfer(0x00);
	if (!initDone) {
		if (*pBuff == 0x70) {
			return 0;
		}
	}
	DataLen = (0x01 & (*pBuff));  //High byte length, only bit0 1 Wei
	pBuff++;
	DataLen <<= 8;
	DataLen += temp;

	for (byte i = 0; i < DataLen; i++) {
		*pBuff = SPI.transfer(0x00);
		pBuff++;
	}

	delayMicroseconds(2); //Data Transmission (T2)
	digitalWrite(etPin, HIGH);
	setDelay(COMMAND_DELAY);
	SPI.endTransaction();
	return DataLen + 2; //Returns the total length, including 2-byte frame information
}

void spiResetET7190() {
	SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE1));
	digitalWrite(etPin, LOW);
	SPI.transfer(CMD_RESET); //Device reset
	delayMicroseconds(spiT1Val); 
	digitalWrite(etPin, HIGH);
	SPI.endTransaction();
	delay(25); // After reset instruction minimum delay is 20MS.
}

byte spiReadET7190ID(byte * pID, byte * pVer) {
	byte data[4];
	spiResetET7190();
	SPI.beginTransaction(SPISettings(SPICLOCK, MSBFIRST, SPI_MODE1));
	digitalWrite(etPin, LOW);
	SPI.transfer(CMD_RDID);
	delayMicroseconds(spiT1Val * 2); 
	data[0] = SPI.transfer(0x00);
	if (data[0] != CMD_RDID) {
		return 0;
	}
	data[1] = SPI.transfer(0x00);
	*pID = data[1];
	data[2] = SPI.transfer(0x00);
	*pVer = data[2];
	delayMicroseconds(2); 
	digitalWrite(etPin, HIGH);
	setDelay(COMMAND_DELAY);
	SPI.endTransaction();
	return 1;
}

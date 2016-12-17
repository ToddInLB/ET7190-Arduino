bool uartResetET7190() {
	uartTempBuff[0] = CMD_RESET;
	uartTempBuff[1] = 0;
	uartTempBuff[2] = 0;
	uartTempBuff[3] = 0;
	if (ET7190Serial.write(uartTempBuff, 4) != 4) {
		//Problem sending data
		delay(51);
		return false;
	}
	delay(51);
	return true;
}
void uartWriteRegister(byte Register, byte Val) {
	uartTempBuff[0] = CMD_WR_REG;
	uartTempBuff[1] = Register;
	uartTempBuff[2] = Val;
	uartTempBuff[3] = 0;
	if (ET7190Serial.write(uartTempBuff, 4) != 4) {
		//Problem sending data
	}
}
void uartSetRegBit(byte Register, byte Val) {
	uartTempBuff[0] = CMD_SET_REG_BIT;
	uartTempBuff[1] = Register;
	uartTempBuff[2] = Val;
	uartTempBuff[3] = 0;
	if (ET7190Serial.write(uartTempBuff, 4) != 4) {
		//Problem sending data
	}
}
void uartClrRegBit(byte Register, byte Val) {
	uartTempBuff[0] = CMD_CLR_REG_BIT;
	uartTempBuff[1] = Register;
	uartTempBuff[2] = Val;
	uartTempBuff[3] = 0;
	if (ET7190Serial.write(uartTempBuff, 4) != 4) {
		//Problem sending data
	}
}
void uartWriteRegisterBuff(byte RegBuff, byte DataLen, byte * pData) {
	uartTempBuff[0] = CMD_WR_BUFF;
	uartTempBuff[1] = RegBuff;
	uartTempBuff[2] = DataLen;
	uartTempBuff[3] = 0;
	if (ET7190Serial.write(uartTempBuff, 4) == 4) {
		delayMicroseconds(31);
		ET7190Serial.write(pData, DataLen);
	} else {
// problem
	}
}
void uartWriteLongBuff(byte RegBuff, byte DataLen, byte * pData) {
	uartTempBuff[0] = CMD_WR_LB;
	uartTempBuff[1] = RegBuff;
	uartTempBuff[2] = (0xFF & DataLen);
	uartTempBuff[3] = (DataLen >> 8);
	if (ET7190Serial.write(uartTempBuff, 4) == 4) {
		delayMicroseconds(11);
		ET7190Serial.write(pData, DataLen);
	} else {
		//uartWriteLongBuff Problem sending 4 byte instuction
	}
}
void uartWriteByteOfLongBuff(byte RegBuff, byte offset, byte Val) {
	uartTempBuff[0] = CMD_WR_BUFF_BYTE;
	uartTempBuff[1] = RegBuff;
	uartTempBuff[2] = offset;
	uartTempBuff[3] = Val;
	if (ET7190Serial.write(uartTempBuff, 4) != 4) {
		//Problem sending data
	}
}
void uartFillByteOfLongBuff(byte RegBuff, byte Val) {
	uartTempBuff[0] = CMD_FILL_BUFF;
	uartTempBuff[1] = RegBuff;
	uartTempBuff[2] = Val;
	uartTempBuff[3] = 0;
	if (ET7190Serial.write(uartTempBuff, 4) != 4) {
		//Problem sending data
	}
}
byte uartReadReg(byte Register) {
	uint32_t begtime;
	uint32_t waittime = 4000;

	uartTempBuff[0] = CMD_RD_REG;
	uartTempBuff[1] = Register;
	uartTempBuff[2] = 0;
	uartTempBuff[3] = 0;
	if (ET7190Serial.write(uartTempBuff, 4) != 4) {
		//Problem sending data
	}
	delayMicroseconds(11);
	begtime = millis();
	while ((ET7190Serial.available() == 0) && (waittime >= (millis() - begtime))) {}
	if (ET7190Serial.available()>0) {
		uartReadBuff[0] = ET7190Serial.read();
		return uartReadBuff[0];
	}
	return 0;
}
int16_t uartReadRegisterBuff(byte RegBuff, int16_t DataLen, byte * pBuff) {
	byte delayVal = 50;
	int16_t curIndex = 0;
	uint32_t begtime;
	uint32_t waittime = 4000;

	uartTempBuff[0] = CMD_RD_BUFF;
	uartTempBuff[1] = RegBuff;
	uartTempBuff[2] = 0;
	uartTempBuff[3] = 0;
	if (ET7190Serial.write(uartTempBuff, 4) != 4) {
		//Problem sending data
	}
	if (DataLen < 17) { delayVal = 11; }
	delayMicroseconds(delayVal);
	begtime = millis();
	while ((ET7190Serial.available() == 0) && (waittime >= (millis() - begtime))) {}
	if (ET7190Serial.available()>0) {
		begtime = millis();
		while ((curIndex <= DataLen) & (readIntervalTimeout >= (millis() - begtime))) {
			while (ET7190Serial.available()) {
				*pBuff = ET7190Serial.read();
				pBuff++;
				curIndex++;
				//        DataLen--;
				begtime = millis();
			}
		}
		return curIndex - 1;
		//    return curIndex;
	}
	return 0;
}
int16_t uartRead_URX(byte *pBuff) {
	return 0;
}
bool uartReadID(byte * pID, byte * pVer) {
	uartTempBuff[0] = CMD_RDID;
	uartTempBuff[1] = 0;
	uartTempBuff[2] = 0;
	uartTempBuff[3] = 0;
	if (ET7190Serial.write(uartTempBuff, 4) != 4) {
		//"uartReadID ET7190Serial.write did NOT send 4 bytes"
	}
	//  delayMicroseconds(3);
	uint32_t begtime;
	uint32_t waittime = 4000;
	begtime = millis();
	while ((ET7190Serial.available() != 2) && (waittime >= (millis() - begtime))) {}
	if (ET7190Serial.available()) {
		uartReadBuff[0] = ET7190Serial.read();
		*pID = uartReadBuff[0];
		*pVer = uartReadBuff[1];
		return true;
	}
	return false;
}

#ifndef ET7190_H
#define ET7190_H





//#define NOP __asm__ __volatile__ ("nop\n\t")
/*********************************************************************************************************
*
*   J1850 Frame Information£¬4 bytes.........J1850_FRAME_INFO
*
*********************************************************************************************************/
typedef union Tag_J1850_FRAME_INFO {
	struct HEADER {
		unsigned char FVOP : 1;    // 0=VPW or 1=PWM Falg
		unsigned char r1 : 1;    //res
		unsigned char VPWNB : 1;    //VPW_NB bits 0=short plus   1=long puls
		unsigned char HEADERTYPE : 1;
		unsigned char  r2 : 3;    //res
		unsigned char DIR : 1;    //Direction

		unsigned char   DATA_Len : 4;  //Data Length
		unsigned char IFR_Len : 4;    //IFR Length

		unsigned char   RESB1;                     //res

		unsigned char   r3 : 4;    //res
		unsigned char   Protocol : 4;    //Read always=1  
	}header;

	//unsigned int wValue[2];
	//unsigned long dwValue;
	//unsigned char bbyte[4];
	uint16_t wValue[2];
	uint32_t dwValue;
	unsigned char bbyte[4];

}J1850_FRAME_INFO;

/*
typedef union tag_no_member {
J1850_FRAME_INFO J_FrameInfo;
byte   data_bytes[4];
} no_member; //, *lpno_member;
*/
/*********************************************************************************************************
*
*   J1850 RX/TX  Buffer £¬Length 16 bytes..............J1850_DATAPACKAGE
*
*********************************************************************************************************/

typedef union Tag_J1850_DATAPACKAGE {
	struct {
		J1850_FRAME_INFO J_FrameInfo;       //4 bytes
		unsigned char    Data[12];
	}u1;
	struct {
		J1850_FRAME_INFO J_FrameInfo;
		union {
			struct {
				unsigned char Z0 : 1;
				unsigned char Z1 : 1;
				unsigned char Y : 1;
				unsigned char K : 1;
				unsigned char H : 1;
				unsigned char P0 : 1;
				unsigned char P1 : 1;
				unsigned char P2 : 1;

			}con;
			struct {
				unsigned char YZZ : 3;
				unsigned char K2 : 1;
				unsigned char H2 : 1;
				unsigned char P : 3;
			}yzz;
			unsigned char  Val;
		}HeaderID;
		unsigned char Target;
		unsigned char Source;
	}u2;

	uint16_t    _word[8];
	uint32_t   _dword[4];
	unsigned char   _byte[16];

}J1850_DATAPACKAGE, *LPJ1850_DATAPACKAGE;
const byte SM_SLOW_INIT_ISO = 0;
const byte SM_FAST_INIT_ISO = 1;
/*********************************************************************************************************
*
*  EUART Transmit/receive Buffer  264 bytes
*
*********************************************************************************************************/

//typedef union Tag_EUART_DATAPACKAGE {
//
//	struct//Normal Frame
//	{
//		//2 Bytes,Frame Information
//		unsigned short int FLen : 9;    //Frame Data Length, Maximum: 260 bytes
//		unsigned short int noop : 2;    //res
//		unsigned short int CSERR : 1;    //CS Flag
//		unsigned short int INIT : 3;    //Initialization Format
//		unsigned short int SACK : 1;    //special for LIN protocol
//
//																		//262????????
//		unsigned char      Data[262];     //Data Buffer .  Maximum : header 4 + 255 +CS =260 bytes
//	}normFrame;
//
//	struct//Initialization Information
//	{
//		unsigned short int  FrameInfo;
//		unsigned char       InitErrFlag;
//		unsigned char       NADDR;
//		unsigned short int  BRG;
//		unsigned char       IniData[8];
//
//	}initInf;
//
//	unsigned short int    _word[132];
//	unsigned char         _byte[264];
//
//}EUART_DATAPACKAGE, *LPEUART_DATAPACKAGE;
typedef union Tag_EUART_DATAPACKAGE {

	struct//Normal Frame
	{
		//2 Bytes,Frame Information
		uint16_t FLen : 9;    //Frame Data Length, Maximum: 260 bytes
		uint16_t noop : 2;    //res
		uint16_t CSERR : 1;    //CS Flag
		uint16_t INIT : 3;    //Initialization Format
		uint16_t SACK : 1;    //special for LIN protocol

													//262????????
		unsigned char      Data[262];     //Data Buffer .  Maximum : header 4 + 255 +CS =260 bytes
	}normFrame;

	struct//Initialization Information
	{
		uint16_t  FrameInfo;
		unsigned char       InitErrFlag;
		unsigned char       NADDR;
		uint16_t  BRG;
		unsigned char       IniData[8];

	}initInf;

	uint16_t    _word[132];
	unsigned char         _byte[264];

}EUART_DATAPACKAGE, *LPEUART_DATAPACKAGE;

/*************************************************************************************************************
*
*    ET7190 User Command
*
**************************************************************************************************************/
#define CMD_RD_REG       0x00   // Read register    
#define CMD_RD_BUFF      0x40   // Read Buffer (Register Set)

#define CMD_WR_REG       0x80   // Write Register     Addr from 0 to 0x7F
#define CMD_SET_REG_BIT  0x81   // Register Bit set £º Addr:0-63
#define CMD_CLR_REG_BIT  0x82   // Register Bit Clear £º Addr:0-63

#define CMD_WR_BUFF_BYTE 0x83   // Write one Byte in Long Buffer
#define CMD_FILL_BUFF    0x84   // Fill the entire buffer with a fixed value

#define CMD_WR_BUFF      0xA1   // Continuously write data in the entire buffer(the buffer with length equal to or smaller than 16 bytes) 
#define CMD_WR_LB        0xA8   // Continuously write data in the entire long buffer  (the buffer length is bigger than 16 bytes)

#define CMD_RESET        0xEA   // Reset device
#define CMD_RDID         0xFF   // Read device ID and version 


/*************************************************************************************************************
*
*    ET7190 Register Address Map
*
**************************************************************************************************************/

#define X_INTF                0x00  //Interrupt Flag Register
#define X_INTIE               0x01  //Interrupt Enable Register
#define X_SLEEP               0x02  //Sleep Control Register

//J1850 register
#define XJ_JERRF              0x04  //J1850 Error Register
#define XJ_JERRIE             0x05  //J1850 Error Interrupt Flag Enable Register
#define XJ_IFR3B0LEN          0x06  //IFR Type 3 Buffer 0 Data Length
#define XJ_IFR3B1LEN          0x07  //IFR Type 3 Buffer 1 Data Length
#define XJ_JCON0              0x08  //J1850 Control Register 0
#define XJ_JCON1              0x09  //J1850 Control Register 1
#define XJ_JSTAT              0x0A  //J1850 State Register

//IO Port
#define X_ODC                 0x0B  //IO Open-drain
#define X_UPR                 0x0C  //IO Input Pull-up
#define X_PDIR                0x0D  //IO Direction Control
#define X_PORT                0x0E  //IO Port Level

//CAN register 
#define XC_CCTRL0             0x10  //CAN Control Register 0, set CAN operation mode
#define XC_CCTRL1             0x11  //CAN CAN Control Register 1
#define XC_CCTRL2             0x12  //CAN Control Register 2, control NSTB and SWEN pin
#define XC_PINSEL             0x13  //CAN Select CAN1TX/RX or CAN2TX/RX Pin Set 
#define XC_CDNCNT             0x14  //This register contains DeviceNet? filter control bit
#define XC_CINTF0             0x15  //CAN Error Interrupt Flag Register 0
#define XC_CINTF1             0x16  //CAN Error Interrupt Flag Register 1
#define XC_CINTE              0x17  //CAN Error Interrupt Enable Register
#define XC_TERRCNT            0x18  //CAN Transmit Error Counter
#define XC_RERRCNT            0x19  //CAN Receive Error Counter
#define XC_CCFG0              0x1A  //CAN BaudRate Configuration Register 0
#define XC_CCFG1              0x1B  //CAN BaudRate Configuration Register 1
#define XC_CCFG2              0x1C  //CAN BaudRate Configuration Register 2
#define XC_CFEN0              0x1D  //CAN Receive Filter Enable Register 0
#define XC_CFEN1              0x1E  //CAN Receive Filter Enable Register 1
#define XC_CFMSKSEL0          0x1F  //CAN  Filter 3-0     Mask Select Register
#define XC_CFMSKSEL1          0x20  //CAN  Filter 7-4     Mask Select Register
#define XC_CFMSKSEL2          0x21  //CAN Filter 11-8     Mask Select Register
#define XC_CFMSKSEL3          0x22  //CAN Filter 15-12    Mask Select Register
#define XC_CTXPRI0            0x23  //CAN Transmitter 3-0 Priority
#define XC_CTXPRI1            0x24  //CAN Transmitter 7-4 Priority
#define XC_CTXREQ             0x25  //CAN Message Transmit Request Register
#define XC_CTXABT             0x26  //CAN Message Abort Transmit Request Register
#define XC_CRTREN             0x27  //CAN Auto Remote Transmit Enable Register
#define XC_CTXAUTOEN          0x28  //After CAN writes in transmit buffer, automatically start the transmit register 
#define XC_CPROCON            0x29  //CAN Multi-frame Protocol Control
#define XC_CINTF2             0x2A  //CAN Multi-frame Protocol Transmit Error Flag Register
#define XC_CCFDL              0x2B  //CAN multi-frame transmission, data length of CFTXB buffer

//UART register
#define XU_UFRMCON            0x2C  //EUART Frame Mode Control Register
#define XU_PINSEL             0x2D  //EUART Drive Pin Select
#define XU_UFRMV              0x2E  //EUART Frame Check Control Register
#define XU_UFLENCON           0x2F  //EUART Frame Length Control Register
#define XU_UFLAND             0x30  //"And Value" in EUART Frame Length Setting
#define XU_UFTXSTAT           0x31  //EUART State Register in EUART Frame Transmission
#define XU_INITLT             0x32  //EUART Low Level Time of ISO14230 Fast Initialization
#define XU_INITHT             0x33  //EUART High Level Time of ISO14230 Fast Initialization
#define XU_INITADDR           0x34  //EUART 5BUAD Slow Initialization and Trigger Address
#define XU_UMODE              0x35  //EUART EUART Control Register
#define XU_USTAT0             0x36  //EUART State Register 0
#define XU_USTAT1             0x37  //EUART State Register 1
#define XU_TRXREG             0x38  //EUART Single Byte Transmit/Receive Register
#define XU_BITADJ             0x39  //EUART Time Fine-tuning of Fast Initialization and Slow Initialization
#define XU_UERRF              0x3A  //EUART Error Register
#define XU_UERRIE             0x3B  //EUART Error Interrupt Enable Register
#define XU_UPRICON            0x3C  //EUART J1708 Priority Control

/****************************************************************************************************************
*
*    ET7190 Register address from 0x40 to 0x7F register write only
*
*****************************************************************************************************************/
#define XU_ABAUD              0x40  // EUART EUART Enable BaudRate Measurement
#define XU_UTXBRK             0x41  // EUART Transmit Sync Break,
#define XU_UTXP1T             0x42  // EUART Interbyte Space Time in EUART Frame Transmission
#define XU_UFREET             0x44  // EUART Bus Release Time.
#define XU_URXP1T             0x45  // EUART Allowed Maximum Interbyte Space in EUART Frame Reception 
#define XU_UTXVT0             0x46  // EUART Maximum Time of Receive Check Byte in EUART Frame Transmission  
#define XU_URXVT1             0x48  // EUART Time Delay of Transmit Check Byte in EUART Frame Reception

//J1850 register YZZCON
#define XJ_YZZCON0            0x50  // J1850 Corresponding to  YZZCON Buffer Byte 0 
#define XJ_YZZCON1            0x51  // J1850 Corresponding to  YZZCON Buffer Byte 1 
#define XJ_YZZCON2            0x52  // J1850 Corresponding to  YZZCON Buffer Byte 2
#define XJ_YZZCON3            0x53  // J1850 Corresponding to  YZZCON Buffer Byte 3 
#define XJ_YZZCON4            0x54  // J1850 Corresponding to  YZZCON Buffer Byte 4 
#define XJ_YZZCON5            0x55  // J1850 Corresponding to  YZZCON Buffer Byte 5 
#define XJ_YZZCON6            0x56  // J1850 Corresponding to  YZZCON Buffer Byte 6 
#define XJ_YZZCON7            0x57  // J1850 Corresponding to  YZZCON Buffer Byte 7 

#define XC_TimeN_BS           0x60  //CAN ISO15765 Time_N_BS =xx * 5ms
#define XC_TimeN_ST           0x61  //CAN ISO15765 Time_N_ST =xx * 100us
#define XC_TimeN_BR           0x62  //CAN ISO15765 Time_N_BR =xx * 100us
#define XC_TimeN_CR           0x63  //CAN ISO15765 Time_N_CR =xx * 5ms

#define XC_TP20_T1            0x64  //CAN TP2.0 T1= xx * 5ms  Time-out used by this Cont rol mode for  received telegrams
#define XC_TP20_T3            0x65  //CAN TP2.0 T3= xx * 100us minimum t ime when sending between consecutive telegrams
#define XC_TP20_TE            0x66  //CAN TP2.0 TE = xx * 5ms Time-out for connection structure wai ting for response
#define XC_TP20_T3A           0x67  //CAN TP2.0 T3 = xx * 1000us minimum t ime when sending between consecutive telegrams
#define XC_TP20_TW            0x6F  //CAN TP2.0 Tw = xx * 5ms If Receiver is not ready, insert a delay of T_wait before sending the next Datatelegram.

#define XC_J1939_T2           0x68  //CAN J1939 T2= xx * 5ms Maximum waiting DT time when receiving multiple frames
#define XC_J1939_T3           0x69  //CAN J1939 T3= xx * 5ms Maximum waiting receiver CTS time when transmitting multiple frames
#define XC_J1939_TS           0x6A  //CAN J1939 TS= xx * 1ms Time slot of continuous transmission of DT
#define XC_J1939_TR           0x6B  //CAN J1939 TR= xx * 1m  Delay of requiring the transmission of respond frame CTS/EOM when receiving multiple frames

#define X_ADC                 0x70  //Start AD conversion, do voltage measurement 

/*************************************************************************************************************
*
*    ET7190 Register Set  (Buffer)
*
**************************************************************************************************************/
//J1850 buffer
#define BJ_YZZCON             0x00 //J1850 3 Byte Header IFR Control Register Set                                                8 Bytes
#define BJ_IDIFRCON           0x01 //J1850 IFR Control Register Set in Single Header or One Byte Header of Consilidated Mode,  
//write and use long buffer write command:CMD_WR_LB                                           256 bytes      
#define BJ_HDFILTER           0x02 //Receive Filter Control Register Set, write and use long buffer command CMD_WR_LB            256 bytes
#define BJ_IDBUFF             0x03 //J1850 Responded ID Buffer in J1850 IFR type 1/2 Mode                        8 bytes
#define BJ_IFR3BUFF0          0x04 //J1850 Responded Data Buffer 0 in J1850 IFR Type 3 Mode                      12 bytes--10 vaild
#define BJ_IFR3BUFF1          0x05 //J1850 Responded Data Buffer 1 in J1850 IFR Type 3 Mode                      12 bytes--10 vaild    
#define BJ_JTXB               0x06 //J1850 Transmit Buffer                                                       16 bytes    
#define B_DRVUBRG             0x07 //UART  User Interface BaudRate Scaler                                         2 bytes
#define B_ADCB                0x08 //AD    ADC value,                                                             2 bytes--10 bits   

// CAN buffer
#define BC_CRXMASK0           0x09 //CAN Receive Filter Mask Standard Identifier Register 0   4 bytes
#define BC_CRXMASK1           0x0A //CAN Receive Filter Mask Standard Identifier Register 1   4 bytes
#define BC_CRXMASK2           0x0B //CAN Receive Filter Mask Standard Identifier Register 2   4 bytes
#define BC_CRXF0              0x0C //CAN Receive Filter Identifier Register 0       4 bytes
#define BC_CRXF1              0x0D //CAN Receive Filter Identifier Register 1       4 bytes
#define BC_CRXF2              0x0E //CAN Receive Filter Identifier Register 2       4 bytes
#define BC_CRXF3              0x0F //CAN Receive Filter Identifier Register 3       4 bytes
#define BC_CRXF4              0x10 //CAN Receive Filter Identifier Register 4       4 bytes
#define BC_CRXF5              0x11 //CAN Receive Filter Identifier Register 5       4 bytes
#define BC_CRXF6              0x12 //CAN Receive Filter Identifier Register 6       4 bytes
#define BC_CRXF7              0x13 //CAN Receive Filter Identifier Register 7       4 bytes
#define BC_CRXF8              0x14 //CAN Receive Filter Identifier Register 8       4 bytes
#define BC_CRXF9              0x15 //CAN Receive Filter Identifier Register 9       4 bytes
#define BC_CRXF10             0x16 //CAN Receive Filter Identifier Register 10      4 bytes
#define BC_CRXF11             0x17 //CAN Receive Filter Identifier Register 11      4 bytes
#define BC_CRXF12             0x18 //CAN Receive Filter Identifier Register 12      4 bytes
#define BC_CRXF13             0x19 //CAN Receive Filter Identifier Register 13      4 bytes
#define BC_CRXF14             0x1A //CAN Receive Filter Identifier Register 14      4 bytes
#define BC_CRXF15             0x1B //CAN Receive Filter Identifier Register 15      4 bytes

#define BC_CFPNT0             0x1C //CAN Filter 0-7 Buffer Pointer Register         4 bytes  
#define BC_CFPNT1             0x1D //CAN Filter 8-15 Buffer Pointer Register        4 bytes  

#define BC_CTXB0              0x1E //CAN CAN Transmit Buffer 0                     16 bytes 
#define BC_CTXB1              0x1F //CAN CAN Transmit Buffer 1                     16 bytes 
#define BC_CTXB2              0x20 //CAN CAN Transmit Buffer 2                     16 bytes 
#define BC_CTXB3              0x21 //CAN CAN Transmit Buffer 3                     16 bytes 
#define BC_CTXB4              0x22 //CAN CAN Transmit Buffer 4                     16 bytes 
#define BC_CTXB5              0x23 //CAN CAN Transmit Buffer 5                     16 bytes 
#define BC_CTXB6              0x24 //CAN CAN Transmit Buffer 6                     16 bytes 
#define BC_CTXB7              0x25 //CAN CAN Transmit Buffer 7                     16 bytes

#define BC_C15765TXFC         0x26 //CAN ISO15765 Multi-Frame Transmit Control Register Set           4 bytes
#define BC_C15765RXFC0        0x27 //CAN ISO15765 Receive Auto Respond FC Control Register Set 0      4 bytes
#define BC_C15765RXFC1        0x28 //CAN ISO15765 Receive Auto Respond FC Control Register Set 1      4 bytes
#define BC_CTP20CTRL          0x29 //CAN TP2.0 Multi-frame Transmit/Auto Respond Control              4 bytes
#define BC_CJ1939CTRL         0x34 //CAN J1939 Multi-frame Transmit/Receive RTS/CTS Control           4 bytes

//UART buffer
#define BU_UBRG               0x2A  //EUART Bus BaudRate Scaler(16-bit integer)                       2 bytes
#define BU_U5BAUDCON          0x2B  //EUART 5 Baud Bus Initialization Control                         6 bytes
#define BU_UPULSCON           0x2C  //EUART User-defined Initialization Pulse Control                 8 bytes
#define BU_UPULSBIT           0x2D  //EUART 16 User-defind Pulse Bit Setting                         16 bytes
#define BU_SIMUCON            0x2E  //EUART ECU Simulation Control Special Use                       12 bytes
#define BU_LIDCON             0x2F  //EUART LIN1.x LIN2.x  ID Control                                64 bytes  / Write Use Command: CMD_WR_LB
#define BU_LACKB0             0x30  //EUART LIN Slaver Auto Respond, Data Transmit Buffer 0           8 bytes
#define BU_LACKB1             0x31  //EUART LIN Slaver Auto Respond, Data Transmit Buffer 1           8 bytes
#define BU_LACKB2             0x32  //EUART LIN Slaver Auto Respond, Data Transmit Buffer 2           8 bytes
#define BU_LACKB3             0x33  //EUART LIN Slaver Auto Respond, Data Transmit Buffer 3           8 bytes

#define BX_ERRB               0x3C  //Error Flag Register Map Register Set                            6 bytes£¬Bufer[5] always as 0 when Read

#define BJ_JRXB               0x3D  // J1850 FIFO Receive Buffer, FIFO depth is 10 frames            16 bytes*10 FIFO
#define BC_CRXB               0x3E  // CAN   (Read Only) CAN Receive Buffer,                         16 bytes*16 FIFO
#define BC_CFTXB              0x3E  // CAN   (Write Only) Multi-frame Transmit Buffer                256 bytes          
#define BU_URXB               0x3F  // UART  (Read Only) FIFO Message Receive Buffer                 262 bytes*2 FIFO 
#define BU_UTXB               0x3F  // UART  (Write Only) Message Transmit Buffer                    262 Bytes   / Write Use Command: CMD_WR_LB



#endif //end of ET7190_H

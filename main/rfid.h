/*
  Library for controlling the Nano M6E from ThingMagic
  This is a stripped down implementation of the Mercury API from ThingMagic

  By: Nathan Seidle @ SparkFun Electronics
  Date: October 3rd, 2016
  https://github.com/sparkfun/Simultaneous_RFID_Tag_Reader

  License: Open Source MIT License
  If you use this code please consider buying an awesome board from SparkFun. It's a ton of
  work (and a ton of fun!) to put these libraries together and we want to keep making neat stuff!
  https://opensource.org/licenses/MIT

  The above copyright notice and this permission notice shall be included in all copies or
  substantial portions of the Software.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_task_wdt.h"

#define MAX_MSG_SIZE 255

#define TMR_SR_OPCODE_VERSION 0x03
#define TMR_SR_OPCODE_SET_BAUD_RATE 0x06
#define TMR_SR_OPCODE_READ_TAG_ID_SINGLE 0x21
#define TMR_SR_OPCODE_READ_TAG_ID_MULTIPLE 0x22
#define TMR_SR_OPCODE_WRITE_TAG_ID 0x23
#define TMR_SR_OPCODE_WRITE_TAG_DATA 0x24
#define TMR_SR_OPCODE_KILL_TAG 0x26
#define TMR_SR_OPCODE_READ_TAG_DATA 0x28
#define TMR_SR_OPCODE_CLEAR_TAG_ID_BUFFER 0x2A
#define TMR_SR_OPCODE_MULTI_PROTOCOL_TAG_OP 0x2F
#define TMR_SR_OPCODE_GET_READ_TX_POWER 0x62
#define TMR_SR_OPCODE_GET_WRITE_TX_POWER 0x64
#define TMR_SR_OPCODE_GET_POWER_MODE 0x68
#define TMR_SR_OPCODE_GET_READER_OPTIONAL_PARAMS 0x6A
#define TMR_SR_OPCODE_GET_PROTOCOL_PARAM 0x6B
#define TMR_SR_OPCODE_SET_ANTENNA_PORT 0x91
#define TMR_SR_OPCODE_SET_TAG_PROTOCOL 0x93
#define TMR_SR_OPCODE_SET_READ_TX_POWER 0x92
#define TMR_SR_OPCODE_SET_WRITE_TX_POWER 0x94
#define TMR_SR_OPCODE_SET_REGION 0x97
#define TMR_SR_OPCODE_SET_READER_OPTIONAL_PARAMS 0x9A
#define TMR_SR_OPCODE_SET_PROTOCOL_PARAM 0x9B

#define COMMAND_TIME_OUT  2000 //Number of ms before stop waiting for response from module

//Define all the ways functions can return
#define ALL_GOOD                        0
#define ERROR_COMMAND_RESPONSE_TIMEOUT  1
#define ERROR_CORRUPT_RESPONSE          2
#define ERROR_WRONG_OPCODE_RESPONSE     3
#define ERROR_UNKNOWN_OPCODE            4
#define RESPONSE_IS_TEMPERATURE         5
#define RESPONSE_IS_KEEPALIVE           6
#define RESPONSE_IS_TEMPTHROTTLE        7
#define RESPONSE_IS_TAGFOUND            8
#define RESPONSE_IS_NOTAGFOUND          9
#define RESPONSE_IS_UNKNOWN             10
#define RESPONSE_SUCCESS  			    11
#define RESPONSE_FAIL          			12

//Define the allowed regions - these set the internal freq of the module
#define REGION_INDIA        0x04
#define REGION_JAPAN        0x05
#define REGION_CHINA        0x06
#define REGION_EUROPE       0x08
#define REGION_KOREA        0x09
#define REGION_AUSTRALIA    0x0B
#define REGION_NEWZEALAND   0x0C
#define REGION_NORTHAMERICA 0x0D
#define REGION_OPEN         0xFF

#define DEFAULT_UART_PORT 			UART_NUM_2
#define DEFAULT_UART_PORT_DEBUG 	UART_NUM_0

//TODO cserelve lett

#define DEFAULT_UART_PORT_TXD	GPIO_NUM_13 //(UART_PIN_NO_CHANGE)
#define DEFAULT_UART_PORT_RXD	GPIO_NUM_12 //(UART_PIN_NO_CHANGE)
#define DEFAULT_UART_PORT_CTS	(UART_PIN_NO_CHANGE)//GPIO_NUM_25
#define DEFAULT_UART_PORT_RTS	(UART_PIN_NO_CHANGE)//GPIO_NUM_26// ezeket lehet valtoztatni, nem lenyegesek
#define RFID_EN_GPIO 			GPIO_NUM_14	// ra van most kotve az rfid engedelyezo laba

/*
#define DEFAULT_UART_PORT_TXD	GPIO_NUM_15 //(UART_PIN_NO_CHANGE)
#define DEFAULT_UART_PORT_RXD	GPIO_NUM_2 //(UART_PIN_NO_CHANGE)
#define DEFAULT_UART_PORT_CTS	(UART_PIN_NO_CHANGE)//GPIO_NUM_25
#define DEFAULT_UART_PORT_RTS	(UART_PIN_NO_CHANGE)//GPIO_NUM_26// ezeket lehet valtoztatni, nem lenyegesek
#define RFID_EN_GPIO 			GPIO_NUM_22	// ra van most kotve az rfid engedelyezo laba
//UART PORT
*/

#define TAG_PROTOCOL 0x05

#define BUF_SIZE 				(2048)
#define POWER_IN_PERCENT    	50		//read power, in percent 0-100 / if u write more then 100 or less than 0, it will be set at 100%
#define ONE_CYCLE_TIME 			1000
#define NUM_OF_KNOWN_EPC 		50
#define IS_DEBUG_ENABLED 		0
#define IS_DEBUG_ENABLED_LVL2 	1
#define DEFAULT_BADURATE_1 		115200
#define DEFAULT_BADURATE_2 		9600  //Ezen a bauderate-n probalja meg atallitani az eszkoz sebesseget, ha az rossz baudrate-en lenne
#define KILL_PASSWORD_LENGTH 	4
#define ACCES_PASSWORD_LENGTH 	4
#define EPC_LENGTH				12

#define _IF_IS_DEBUG_ENABLED 		if(IS_DEBUG_ENABLED == 1)
#define _IF_IS_DEBUG_ENABLED_LVL2 	if(IS_DEBUG_ENABLED_LVL2 == 1)
#define REBOOT_SYSTEM_IF_RFID_SETUP_FAILS 1 //if this is 1, the system will reboot if the rfid setup fails
#define DEFAULT_DELAY_IN_FORCE_FUNCTIONS 10 // in force functions, this delay will be between two try


typedef struct Stream
{
	uart_port_t uartPort;
}Stream;


typedef struct EPCData //Az EPC-t tarolo struktura
{
	uint8_t bytes[12];
}EPCData;


typedef struct userData //Az user data-t tartalmazo struktura
{
	uint8_t bytes[64];
}userData;


extern int16_t readPower; //

/* ideiglenes tomb, konnyebb kezelni belol az EPC-ket, ha tudom a sorszamukat */
extern uint8_t knownEPC[NUM_OF_KNOWN_EPC + 1][EPC_LENGTH];

uint16_t 	EPCReadingNum[NUM_OF_KNOWN_EPC + 1]; //ez tarolja, hogy melyik elemet egy olvasas soran hanyszor olvastuk ki
bool 		EPCReaded[NUM_OF_KNOWN_EPC + 1];//tarolja hogy az adott indexu EPC be lett-e mar eddig olvasva
int 		EPCRSSINum[NUM_OF_KNOWN_EPC + 1];//tarolja hogy az adott indexu EPC be lett-e mar eddig olvasva

uint8_t 	EPCArray[60];// minden continous olvasasnal ez tarolja az EPC-k sorszamat a beolvasasuk sorrendjeben
uint8_t 	EPCArrayOrdered[60];// ez a tomb tarolja az EPC-k sorszamat beolvasas szerinti sorrendben, de mindnek csak az elso beolvasasi helyet tartja meg
uint8_t 	EPC_temp[EPC_LENGTH];	// ez tarolja mindig az aktualis beolvasott EPC indexet



extern Stream serialConnectionNano;
extern Stream serialConnectionDebug;




int 				available	(Stream* stream_in);
void 				flush 		(Stream* stream_in);
size_t 				rfid_write 		(Stream* stream_in,uint8_t in_write);
uint8_t 			rfid_read 		(Stream* stream_in);
/*
Stream 	_nanoSerial[0]; //The generic connection to user's chosen serial hardware
Stream 	_debugSerial[0]; //The stream to send debug messages to if enabled
uint8_t _head = 0; //Tracks the length of the incoming message as we poll the software serial
bool 		_printDebug = 0; //Flag to print the serial commands we are sending to the Serial port for debug
uint8_t msg[MAX_MSG_SIZE];
*/
Stream* _nanoSerial; //The generic connection to user's chosen serial hardware
Stream* _debugSerial; //The stream to send debug messages to if enabled
uint8_t _head; //Tracks the length of the incoming message as we poll the software serial
bool 	_printDebug; //Flag to print the serial commands we are sending to the Serial port for debug
uint8_t msg[MAX_MSG_SIZE];



	void printErrorCode(uint8_t errorCode);
	/*		class RFID functions 		*/
	bool begin(Stream *serialPort); //If user doesn't specify then Serial will be used
	void enableDebugging(Stream *debugPort); //Turn on command sending and response printing. If user doesn't specify then Serial will be used
	void disableDebugging(void);

	void setBaud(long baudRate);
	void getVersion(void);
	void setReadPower(int16_t powerSetting);
	void getReadPower(void);
	void setWritePower(int16_t powerSetting);
	void getWritePower(void);
	void setRegion(uint8_t region);
	void setAntennaPort(void);
	void setAntennaSearchList(void);
	void setTagProtocol(uint8_t protocol);//0x05 default

	void startReading(void); //Disable filtering and start reading continuously
	void stopReading(void); //Stops continuous read. Give 1000 to 2000ms for the module to stop reading.

	void enableReadFilter(void);
	void disableReadFilter(void);

	void setReaderConfiguration(uint8_t option1, uint8_t option2);
	void getOptionalParameters(uint8_t option1, uint8_t option2);
	void setProtocolParameters(void);
	void getProtocolParameters(uint8_t option1, uint8_t option2);

	uint8_t parseResponse	(void);

	uint8_t 	getTagEPCBytes(void); //Pull number of EPC data bytes from record response.
	uint8_t 	getTagDataBytes(void); //Pull number of tag data bytes from record response. Often zero.
	uint16_t 	getTagTimestamp(void); //Pull timestamp value from full record response
	uint32_t 	getTagFreq	(void); //Pull Freq value from full record response
	int8_t 		getTagRSSI	(void); //Pull RSSI value from full record response

	bool 		check					(void);

	uint8_t readTagEPC		(uint8_t *epc, uint8_t epcLength, uint16_t timeOut);
	uint8_t writeTagEPC		(char *newID, uint8_t newIDLength, uint16_t timeOut);

	uint8_t readData			(uint8_t bank, uint32_t address, uint8_t *dataRead, uint8_t dataLengthRead, uint16_t timeOut);
	uint8_t writeData			(uint8_t bank, uint32_t address, uint8_t *dataToRecord, uint8_t dataLengthToRecord, uint16_t timeOut);

	uint8_t readUserData	(uint8_t *userData, uint8_t userDataLength, uint16_t timeOut);
	uint8_t writeUserData	(uint8_t *userData, uint8_t userDataLength, uint16_t timeOut);

	uint8_t readKillPW		(uint8_t *password, uint8_t passwordLength, uint16_t timeOut);
	uint8_t writeKillPW		(uint8_t *password, uint8_t passwordLength, uint16_t timeOut);

	uint8_t readAccessPW	(uint8_t *password, uint8_t passwordLength, uint16_t timeOut);
	uint8_t writeAccessPW	(uint8_t *password, uint8_t passwordLength, uint16_t timeOut);

	uint8_t readTID				(uint8_t *tid, uint8_t tidLength, uint16_t timeOut);
	uint8_t readUID				(uint8_t *tid, uint8_t tidLength, uint16_t timeOut);

	uint8_t killTag				(uint8_t *password, uint8_t passwordLength, uint16_t timeOut);

	void sendMessage			(uint8_t opcode, uint8_t *data, uint8_t size, uint16_t timeOut, bool waitForResponse);
	void sendCommand			(uint16_t timeOut, bool waitForResponse);

	void printMessageArray(void);

	uint16_t calculateCRC	(uint8_t *u8Buf, uint8_t len);
	/*		End of class RFID functions 		*/


	uint32_t millis();


	bool readSingleEPCForce(EPCData* out, uint32_t measureTimeMillis);

	bool setupNano(long baudRate, uart_config_t* uart_config);

	bool readUsrDataFree(uint8_t* dataOut, uint8_t lenOut);

	bool readUsrDataForce(uint8_t* dataOut, uint8_t lenOut,uint32_t measureTimeMillis);

	void writeUsrDataFree(uint8_t* dataWrite, int len);

	void writeUsrDataForce(uint8_t* dataWrite, int len,uint32_t measureTimeMillis);
	//TODO irni egy olyan fuggvenyt, ami kiolvassa a 2. Banktol a 3. vegeig, es igy meglesz az EPC es az User Data is, es lehet EPC alapjan szurni az USER data-t

	bool readPWcontinouslyFree(uint8_t* killPw, uint8_t* accesPw);

	void readPWcontinouslyForce(uint8_t* killPw, uint8_t* accesPw,  uint32_t measureTimeMillis);

	void writeKillPWForce(uint8_t* killPW, uint32_t measureTimeMillis);

	void writeAccesPWForce(uint8_t* accesPW, uint32_t measureTimeMillis);

	bool setUpNanoAndConnections();

	void insertEPCDataToOrderedArray(EPCData in, EPCData* orderedArray,uint8_t* orderedArraySize);

	void readEPCsForce(uint32_t measureTimeMillis, uint8_t* orderedArraySize,EPCData* orderedArray);

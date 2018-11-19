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

  To learn more about how ThingMagic controls the module please look at the following SDK files:
    serial_reader_l3.c - Contains the bulk of the low-level routines
    serial_reader_imp.h - Contains the OpCodes
    tmr__status_8h.html - Contains the Status Word error codes

  Available Functions:
    setBaudRate
    setRegion
    setReadPower
    startReading (continuous read)
    stopReading
    readTagEPC
    writeTagEPC
    readTagData
    writeTagData
    killTag
    (nyw) lockTag
*/



#include "rfid.h"

#define TICKS_TO_WAIT_FLUSH 		100
#define TICKS_TO_WAIT_IN_UART_WRITE 100
#define TICKS_TO_WAIT_FOR_RECIEVE 	100

Stream serialConnectionNano 	= { DEFAULT_UART_PORT };
Stream serialConnectionDebug 	= { DEFAULT_UART_PORT_DEBUG };

int16_t readPower = ( POWER_IN_PERCENT * 22) + 500;

uint8_t knownEPC[NUM_OF_KNOWN_EPC + 1][EPC_LENGTH] =
{
		{ 0x00, 0x0, 0x0, 0x00, 0x00, 0x0, 0x0, 0x00, 0x0, 0x00, 0x00, 0x00 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x61, 0x0, 0x80, 0xb7, 0x95 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x19, 0x0, 0x80, 0xb7, 0x38 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x18, 0x0, 0x80, 0xb7, 0x3f },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x42, 0x0, 0x80, 0xb7, 0x6f },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x28, 0x0, 0x80, 0xb6, 0x8e },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x16, 0x0, 0x80, 0xb7, 0x3e },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x51, 0x0, 0x80, 0xb6, 0xb6 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x31, 0x0, 0x80, 0xb6, 0x88 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x53, 0x0, 0x80, 0xb6, 0xb7 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x63, 0x0, 0x80, 0xb6, 0xc8 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x63, 0x0, 0x80, 0xb7, 0x96 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x34, 0x0, 0x80, 0xb7, 0x5f },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x33, 0x0, 0x80, 0xb7, 0x57 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x40, 0x0, 0x80, 0xb7, 0x6e },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x90, 0x0, 0x0, 0xbf, 0x1d },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x87, 0x0, 0x0, 0xbf, 0x0 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x0, 0x0, 0x0, 0xbf, 0x2f },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x55, 0x0, 0x90, 0xc1, 0x93 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x42, 0x0, 0x90, 0xc1, 0x79 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x53, 0x0, 0x90, 0xc1, 0x92 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x0, 0x0, 0x0, 0xbf, 0x27 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x74, 0x0, 0x90, 0xc1, 0xb9 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x40, 0x0, 0x90, 0xc1, 0x6c },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x56, 0x0, 0x0, 0xbf, 0x9e },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x53, 0x0, 0x0, 0xbf, 0x95 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x65, 0x0, 0x0, 0xbe, 0xe5 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x52, 0x0, 0x0, 0xbf, 0x90 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x66, 0x0, 0x0, 0xbf, 0xaf },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x10, 0x0, 0x90, 0xc1, 0x39 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x99, 0x0, 0x90, 0xc1, 0x23 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x33, 0x0, 0x90, 0xc1, 0x64 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x29, 0x0, 0x90, 0xc1, 0x62 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x17, 0x0, 0x90, 0xc1, 0x44 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x53, 0x0, 0x80, 0xb7, 0x85 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x39, 0x0, 0x80, 0xb7, 0x66 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x41, 0x0, 0x80, 0xb7, 0x67 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x52, 0x0, 0x80, 0xb7, 0x80 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x17, 0x0, 0x80, 0xb7, 0x37 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x73, 0x0, 0x80, 0xb7, 0xa7 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x55, 0x0, 0x0, 0xbf, 0x96 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x54, 0x0, 0x80, 0xb6, 0xbf },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x64, 0x0, 0x80, 0xb6, 0xd0 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x85, 0x0, 0x80, 0xb6, 0xf7 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x52, 0x0, 0x80, 0xb6, 0xbe },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x0, 0x0, 0x80, 0xb7, 0x2e },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x75, 0x0, 0x80, 0xb6, 0xe6 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x94, 0x0, 0x80, 0xb7, 0x0 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x92, 0x0, 0x80, 0xb7, 0x0 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x91, 0x0, 0x80, 0xb7, 0x0 },
		{ 0xe2, 0x0, 0x0, 0x17, 0x23, 0x0, 0x0, 0x95, 0x0, 0x80, 0xb7, 0x0 } };


void printErrorCode(uint8_t errorCode) {
	/* Kiirja a parameterkent kapott error kodot */
	switch (errorCode) {
	case ALL_GOOD:
		ets_printf("ALL_GOOD \n");
		break;
	case ERROR_COMMAND_RESPONSE_TIMEOUT:
		ets_printf("ERROR_COMMAND_RESPONSE_TIMEOUT \n");
		break;
	case ERROR_CORRUPT_RESPONSE:
		ets_printf("ERROR_CORRUPT_RESPONSE \n");
		break;
	case ERROR_WRONG_OPCODE_RESPONSE:
		ets_printf("ERROR_WRONG_OPCODE_RESPONSE \n");
		break;
	case ERROR_UNKNOWN_OPCODE:
		ets_printf("ERROR_UNKNOWN_OPCODE \n");
		break;
	case RESPONSE_IS_TEMPERATURE:
		ets_printf("RESPONSE_IS_TEMPERATURE \n");
		break;
	case RESPONSE_IS_KEEPALIVE:
		ets_printf("RESPONSE_IS_KEEPALIVE \n");
		break;
	case RESPONSE_IS_TEMPTHROTTLE:
		ets_printf("RESPONSE_IS_TEMPTHROTTLE \n");
		break;
	case RESPONSE_IS_TAGFOUND:
		ets_printf("RESPONSE_IS_TAGFOUND \n");
		break;
	case RESPONSE_IS_NOTAGFOUND:
		ets_printf("RESPONSE_IS_NOTAGFOUND \n");
		break;
	case RESPONSE_IS_UNKNOWN:
		ets_printf("RESPONSE_IS_UNKNOWN \n");
		break;
	case RESPONSE_SUCCESS:
		ets_printf("RESPONSE_SUCCESS \n");
		break;
	case RESPONSE_FAIL:
		ets_printf("RESPONSE_FAIL \n");
	}

}
int available(Stream *stream_in)
{
  int result;
  //ets_printf("Available \n");
  //ets_printf("Belepett az available-be \n");
  //vTaskDelay(10/portTICK_PERIOD_MS);
  ESP_ERROR_CHECK( uart_get_buffered_data_len(stream_in->uartPort,(size_t*) &result) );
  return result;
}
void flush(Stream *stream_in)
{
	//ets_printf("Flush \n");
  ESP_ERROR_CHECK( uart_wait_tx_done(stream_in->uartPort,TICKS_TO_WAIT_FLUSH) );
}

size_t 	rfid_write(Stream *stream_in,uint8_t in)
{
	//ets_printf("Write \n");
	//ets_printf("Kezdodik az iras uart_write_bytes() \n");
	//if(stream_in->uartPort == UART_NUM_1);
		//ets_printf("A beallitott port az UART_NUM_1 \n");
	//else if(stream_in->uartPort == UART_NUM_0);
		//ets_printf("A beallitott port az UART_NUM_0 \n");
	//else;
		//ets_printf("A beallitott port szar \n");

	//ets_printf("A kiirando adat : %u \n",in);

	//char* test_str = "This is a test string.\n";
	return (size_t)uart_write_bytes(stream_in->uartPort, (const char*)(&in), 1);
	/* hasonlo az elozohoz, de az utolso parameterkent megadott ideig egy bajt irasa utan lent tartja a vonalat "break signal"*/
	//return uart_write_bytes_with_break(uartPort, (const char*)(&in), 1, TICKS_TO_WAIT_IN_UART_WRITE);
}

uint8_t rfid_read(Stream *stream_in)
{
	//ets_printf("Read \n");
  uint8_t result;
  uart_read_bytes(stream_in->uartPort,&result,1,TICKS_TO_WAIT_FOR_RECIEVE);
  return result;
}

uint32_t millis()
{
   return ( xTaskGetTickCount() * portTICK_PERIOD_MS);
  //get_rtc_time_us
}





//Initialize the Serial port
bool begin(Stream *serialPort)
{
	if(serialPort == NULL)
		return 0;
	else
	{
		_nanoSerial = serialPort; //Grab which port the user wants us to use
		return 1;
	}

  //_nanoSerial->begin(); //Stream has no .begin() so the user has to do a whateverSerial.begin(xxxx); from setup()
}

//Enable or disable the printing of sent/response HEX values.
//Use this in conjunction with 'Transport Logging' from the Universal Reader Assistant to see what they're doing that we're not
void enableDebugging(Stream *debugPort)
{
  _debugSerial = debugPort; //Grab which port the user wants us to use for debugging

  _printDebug = 1; //Should we print the commands we send? Good for debugging
}
void disableDebugging(void)
{
  _printDebug = 0; //Turn off extra print statements
}

//Set baud rate
//Takes in a baud rate
//Returns response in the msg array
void setBaud(long baudRate)
{
  //Copy this setting into a temp data array
  uint8_t size = sizeof(baudRate);
  uint8_t data[size];
  for (uint8_t x = 0 ; x < size ; x++)
    data[x] = (uint8_t)(baudRate >> (8 * (size - 1 - x)));

  sendMessage(TMR_SR_OPCODE_SET_BAUD_RATE, data, size, COMMAND_TIME_OUT, 0);
}

//Begin scanning for tags
//There are many many options and features to the nano, this sets options
//for continuous read of GEN2 type tags
void startReading()
{
  disableReadFilter(); //Don't filter for a specific tag, read all tags

  //This blob was found by using the 'Transport Logs' option from the Universal Reader Assistant
  //And connecting the Nano eval kit from Thing Magic to the URA
  //A lot of it has been deciphered but it's easier and faster just to pass a blob than to
  //assemble every option and sub-opcode.
  uint8_t configBlob[] = {0x00, 0x00, 0x01, 0x22, 0x00, 0x00, 0x05, 0x07, 0x22, 0x10, 0x00, 0x1B, 0x03, 0xE8, 0x01, 0xFF};

  /*
    //Timeout should be zero for 1 continuous reading
    SETU16(newMsg, i, 0);
    SETU8(newMsg, i, (uint8_t)0x1); // TM Option 1, for continuous reading
    SETU8(newMsg, i, (uint8_t)TMR_SR_OPCODE_READ_TAG_ID_MULTIPLE); // sub command opcode
    SETU16(newMsg, i, (uint16_t)0x0000); // search flags, only 0x0001 is supported
    SETU8(newMsg, i, (uint8_t)TMR_TAG_PROTOCOL_GEN2); // protocol ID
  */

  sendMessage(TMR_SR_OPCODE_MULTI_PROTOCOL_TAG_OP, configBlob, sizeof(configBlob),COMMAND_TIME_OUT,ERROR_COMMAND_RESPONSE_TIMEOUT);

}

//Stop a continuous read
void stopReading()
{
  //00 00 = Timeout, currently ignored
  //02 = Option - stop continuous reading
  uint8_t configBlob[] = {0x00, 0x00, 0x02};

  sendMessage(TMR_SR_OPCODE_MULTI_PROTOCOL_TAG_OP, configBlob, sizeof(configBlob),COMMAND_TIME_OUT, 0); //Do not wait for response
}

//Given a region, set the correct freq
//0x04 = IN
//0x05 = JP
//0x06 = PRC
//0x08 = EU3
//0x09 = KR2
//0x0B = AU
//0x0C = NZ
//0x0D = NAS2 (North America)
//0xFF = OPEN



void setRegion(uint8_t region)
{
  sendMessage(TMR_SR_OPCODE_SET_REGION, &region, sizeof(region),COMMAND_TIME_OUT,ERROR_COMMAND_RESPONSE_TIMEOUT);
}

//Sets the TX and RX antenna ports to 01
//Because the Nano module has only one antenna port, it is not user configurable
void setAntennaPort(void)
{
  uint8_t configBlob[] = {0x01, 0x01}; //TX port = 1, RX port = 1
  sendMessage(TMR_SR_OPCODE_SET_ANTENNA_PORT, configBlob, sizeof(configBlob),COMMAND_TIME_OUT,ERROR_COMMAND_RESPONSE_TIMEOUT);
}

//This was found in the logs. It seems to be very close to setAntennaPort
//Search serial_reader_l3.c for cmdSetAntennaSearchList for more info
void setAntennaSearchList(void)
{
  uint8_t configBlob[] = {0x02, 0x01, 0x01}; //logical antenna list option, TX port = 1, RX port = 1
  sendMessage(TMR_SR_OPCODE_SET_ANTENNA_PORT, configBlob, sizeof(configBlob),COMMAND_TIME_OUT,ERROR_COMMAND_RESPONSE_TIMEOUT);
}

//Sets the protocol of the module
//Currently only GEN2 has been tested and supported but others are listed here for reference
//and possible future support
//TMR_TAG_PROTOCOL_NONE              = 0x00
//TMR_TAG_PROTOCOL_ISO180006B        = 0x03
//TMR_TAG_PROTOCOL_GEN2              = 0x05
//TMR_TAG_PROTOCOL_ISO180006B_UCODE  = 0x06
//TMR_TAG_PROTOCOL_IPX64             = 0x07
//TMR_TAG_PROTOCOL_IPX256            = 0x08
//TMR_TAG_PROTOCOL_ATA               = 0x1D
void setTagProtocol(uint8_t protocol)
{
  uint8_t data[2];
  data[0] = 0; //Opcode expects 16-bits
  data[1] = protocol;

  sendMessage(TMR_SR_OPCODE_SET_TAG_PROTOCOL, data, sizeof(data),COMMAND_TIME_OUT,ERROR_COMMAND_RESPONSE_TIMEOUT);
}

void enableReadFilter(void)
{
  setReaderConfiguration(0x0C, 0x01); //Enable read filter
}

//Disabling the read filter allows continuous reading of tags
void disableReadFilter(void)
{
  setReaderConfiguration(0x0C, 0x00); //Diable read filter
}

//Sends optional parameters to the module
//See TMR_SR_Configuration in serial_reader_imp.h for a breakdown of options
void setReaderConfiguration(uint8_t option1, uint8_t option2)
{
  uint8_t data[3];

  //These are parameters gleaned from inspecting the 'Transport Logs' of the Universal Reader Assistant
  //And from serial_reader_l3.c
  data[0] = 1; //Key value form of command
  data[1] = option1;
  data[2] = option2;

  sendMessage(TMR_SR_OPCODE_SET_READER_OPTIONAL_PARAMS, data, sizeof(data),COMMAND_TIME_OUT,ERROR_COMMAND_RESPONSE_TIMEOUT);
}

//Gets optional parameters from the module
//We know only the blob and are not able to yet identify what each parameter does
void getOptionalParameters(uint8_t option1, uint8_t option2)
{
  //These are parameters gleaned from inspecting the 'Transport Logs' of the Universal Reader Assistant
  //During setup the software pings different options
  uint8_t data[2];
  data[0] = option1;
  data[1] = option2;
  sendMessage(TMR_SR_OPCODE_GET_READER_OPTIONAL_PARAMS, data, sizeof(data),COMMAND_TIME_OUT,ERROR_COMMAND_RESPONSE_TIMEOUT);
}

//Get the version number from the module
void getVersion(void)
{
  sendMessage(TMR_SR_OPCODE_VERSION,0, 0,COMMAND_TIME_OUT,ERROR_COMMAND_RESPONSE_TIMEOUT);
}

//Set the read TX power
//Maximum power is 2700 = 27.00 dBm
//1005 = 10.05dBm
void setReadPower(int16_t powerSetting)
{
  if (powerSetting > 2700) powerSetting = 2700; //Limit to 27dBm

  //Copy this setting into a temp data array
  uint8_t size = sizeof(powerSetting);
  uint8_t data[size];
  for (uint8_t x = 0 ; x < size ; x++)
    data[x] = (uint8_t)(powerSetting >> (8 * (size - 1 - x)));

  sendMessage(TMR_SR_OPCODE_SET_READ_TX_POWER, data, size,COMMAND_TIME_OUT,ERROR_COMMAND_RESPONSE_TIMEOUT);

}

//Get the read TX power
void getReadPower()
{
  uint8_t data[] = {0x00}; //Just return power
  //uint8_t data[] = {0x01}; //Return power with limits

  sendMessage(TMR_SR_OPCODE_GET_READ_TX_POWER, data, sizeof(data),COMMAND_TIME_OUT,ERROR_COMMAND_RESPONSE_TIMEOUT);
}

//Set the write power
//Maximum power is 2700 = 27.00 dBm
//1005 = 10.05dBm
void setWritePower(int16_t powerSetting)
{
  uint8_t size = sizeof(powerSetting);
  uint8_t data[size];
  for (uint8_t x = 0 ; x < size ; x++)
    data[x] = (uint8_t)(powerSetting >> (8 * (size - 1 - x)));

  sendMessage(TMR_SR_OPCODE_SET_WRITE_TX_POWER, data, size,COMMAND_TIME_OUT,ERROR_COMMAND_RESPONSE_TIMEOUT);
}

//Get the write TX power
void getWritePower()
{
  uint8_t data[] = {0x00}; //Just return power
  //uint8_t data[] = {0x01}; //Return power with limits

  sendMessage(TMR_SR_OPCODE_GET_WRITE_TX_POWER, data, sizeof(data),COMMAND_TIME_OUT,ERROR_COMMAND_RESPONSE_TIMEOUT);
}

//Read a single EPC
//Caller must provide an array for EPC to be stored in
uint8_t readTagEPC(uint8_t *epc, uint8_t epcLength, uint16_t timeOut)
{
  uint8_t bank = 0x01; //User data bank
  uint8_t address = 0x02; //Starts at 2

  return (readData(bank, address, epc, epcLength, timeOut));
}

//This writes a new EPC to the first tag it detects
//Use with caution. This function doesn't control which tag hears the command.
uint8_t writeTagEPC(char *newID, uint8_t newIDLength, uint16_t timeOut)
{
  uint8_t bank = 0x01; //EPC memory
  uint8_t address = 0x02; //EPC starts at spot 4
  printf("%u %u, %s %u,%u ",bank, address, newID, newIDLength, timeOut);
  return 0;
}

//This reads the user data area of the tag. 0 to 64 bytes are normally available.
//Use with caution. The module can't control which tag hears the command.
//TODO Add support for accessPassword
uint8_t readUserData(uint8_t *userData, uint8_t userDataLength, uint16_t timeOut)
{
  uint8_t bank = 0x03; //User data bank
  uint8_t address = 0x00; //Starts at 0

  return (readData(bank, address, userData, userDataLength, timeOut));
}

//This writes data to the tag. 0, 4, 16 or 64 bytes may be available.
//Writes to the first spot 0x00 and fills up as much of the bytes as user provides
//Use with caution. Function doesn't control which tag hears the command.
uint8_t writeUserData(uint8_t *userData, uint8_t userDataLength, uint16_t timeOut)
{
  uint8_t bank = 0x03; //User memory
  uint8_t address = 0x00;

  return (writeData(bank, address, userData, userDataLength, timeOut));
}

//Write the kill password. Should be 4 bytes long
uint8_t writeKillPW(uint8_t *password, uint8_t passwordLength, uint16_t timeOut)
{
  uint8_t bank = 0x00; //Passwords bank
  uint8_t address = 0x00; //Kill password address

  return (writeData(bank, address, password, passwordLength, timeOut));
}

//Read the kill password. Should be 4 bytes long
uint8_t readKillPW(uint8_t *password, uint8_t passwordLength, uint16_t timeOut)
{
  uint8_t bank = 0x00; //Passwords bank
  uint8_t address = 0x00; //Kill password address

  return (readData(bank, address, password, passwordLength, timeOut));
}

//Write the access password. Should be 4 bytes long
uint8_t writeAccessPW(uint8_t *password, uint8_t passwordLength, uint16_t timeOut)
{
  uint8_t bank = 0x00; //Passwords bank
  uint8_t address = 0x02; //Access password address

  return (writeData(bank, address, password, passwordLength, timeOut));
}

//Read the access password. Should be 4 bytes long
uint8_t readAccessPW(uint8_t *password, uint8_t passwordLength, uint16_t timeOut)
{
  uint8_t bank = 0x00; //Passwords bank
  uint8_t address = 0x02; //Access password address

  return (readData(bank, address, password, passwordLength, timeOut));
}

//Read the unique TID of the tag. Should be 20 bytes long
//This is a depricated function left in place in case users still use the readTID command
//This function is actually reading the UID. To read the TID, including the Chip Vendor
//change the address to 0x00.
uint8_t readTID(uint8_t *tid, uint8_t tidLength, uint16_t timeOut)
{
  uint8_t bank = 0x02; //Bank for TID
  uint8_t address = 0x02;

  return (readData(bank, address, tid, tidLength, timeOut));
}

//Read the unique ID of the tag. Can vary from 0 to 20 or more bytes
uint8_t readUID(uint8_t *tid, uint8_t tidLength, uint16_t timeOut)
{
  uint8_t bank = 0x02; //Bank for TID
  uint8_t address = 0x02; //UID of the TID starts at 4

  return (readData(bank, address, tid, tidLength, timeOut));
}

//Writes a data array to a given bank and address
//Allows for writing of passwords and user data
//TODO Add support for accessPassword
//TODO Add support for writing to specific tag
uint8_t writeData(uint8_t bank, uint32_t address, uint8_t *dataToRecord, uint8_t dataLengthToRecord, uint16_t timeOut)
{
  //Example: FF  0A  24  03  E8  00  00  00  00  00  03  00  EE  58  9D
  //FF 0A 24 = Header, LEN, Opcode
  //03 E8 = Timeout in ms
  //00 = Option initialize
  //00 00 00 00 = Address
  //03 = Bank
  //00 EE = Data
  //58 9D = CRC

  uint8_t data[8 + dataLengthToRecord];

  //Pre-load array options
  data[0] = timeOut >> 8 & 0xFF; //Timeout msB in ms
  data[1] = timeOut & 0xFF; //Timeout lsB in ms
  data[2] = 0x00; //Option initialize

  //Splice address into array
  for (uint8_t x = 0 ; x < sizeof(address) ; x++)
    data[3 + x] = address >> (8 * (3 - x)) & 0xFF;

  //Bank 0 = Passwords
  //Bank 1 = EPC Memory Bank
  //Bank 2 = TID
  //Bank 3 = User Memory
  data[7] = bank;

  //Splice data into array
  for (uint8_t x = 0 ; x < dataLengthToRecord ; x++)
    data[8 + x] = dataToRecord[x];

  sendMessage(TMR_SR_OPCODE_WRITE_TAG_DATA, data, sizeof(data), timeOut,ERROR_COMMAND_RESPONSE_TIMEOUT);

  if (msg[0] == ALL_GOOD) //We received a good response
  {
    uint16_t status = (msg[3] << 8) | msg[4];

    if (status == 0x0000)
      return (RESPONSE_SUCCESS);
  }

  //Else - msg[0] was timeout or other
  return (RESPONSE_FAIL);
}

//Reads a given bank and address to a data array
//Allows for writing of passwords and user data
//TODO Add support for accessPassword
//TODO Add support for writing to specific tag
uint8_t readData(uint8_t bank, uint32_t address, uint8_t *dataRead, uint8_t dataLengthRead, uint16_t timeOut)
{
  //Bank 0
  //response: [00] [08] [28] [00] [00] [EE] [FF] [11] [22] [12] [34] [56] [78]
  //[EE] [FF] [11] [22] = Kill pw
  //[12] [34] [56] [78] = Access pw

  //Bank 1
  //response: [00] [08] [28] [00] [00] [28] [F0] [14] [00] [AA] [BB] [CC] [DD]
  //[28] [F0] = CRC
  //[14] [00] = PC
  //[AA] [BB] [CC] [DD] = EPC

  //Bank 2
  //response: [00] [18] [28] [00] [00] [E2] [00] [34] [12] [01] [6E] [FE] [00] [03] [7D] [9A] [A3] [28] [05] [01] [6B] [00] [05] [5F] [FB] [FF] [FF] [DC] [00]
  //[E2] = CIsID
  //[00] [34] [12] = Vendor ID = 003, Model ID == 412
  //[01] [6E] [FE] [00] [03] [7D] [9A] [A3] [28] [05] [01] [69] [10] [05] [5F] [FB] [FF] [FF] [DC] [00] = Unique ID (TID)

  //Bank 3
  //response: [00] [40] [28] [00] [00] [41] [43] [42] [44] [45] [46] [00] [00] [00] [00] [00] [00] ...
  //User data

  uint8_t data[8];

  //Insert timeout
  data[0] = timeOut >> 8 & 0xFF; //Timeout msB in ms
  data[1] = timeOut & 0xFF; //Timeout lsB in ms

  data[2] = bank; //Bank

  //Splice address into array
  for (uint8_t x = 0 ; x < sizeof(address) ; x++)
    data[3 + x] = address >> (8 * (3 - x)) & 0xFF;

  data[7] = dataLengthRead / 2; //Number of 16-bit chunks to read.
  //0x00 will read the entire bank but may be more than we expect (both Kill and Access PW will be returned when reading bank 1 from address 0)

  //When reading the user data area we need to read the entire bank
  if(bank == 0x03) data[7] = 0x00;

  sendMessage(TMR_SR_OPCODE_READ_TAG_DATA, data, sizeof(data), timeOut,ERROR_COMMAND_RESPONSE_TIMEOUT);

  if (msg[0] == ALL_GOOD) //We received a good response
  {
    uint16_t status = (msg[3] << 8) | msg[4];

    if (status == 0x0000)
    {
      uint8_t responseLength = msg[1];

      if (responseLength < dataLengthRead) //User wants us to read more than we have available
        dataLengthRead = responseLength;

	  //There is a case here where responseLegnth is more than dataLengthRead, in which case we ignore (don't load) the additional bytes
      //Load limited response data into caller's array
	  for (uint8_t x = 0 ; x < dataLengthRead ; x++)
        dataRead[x] = msg[5 + x];

      return (RESPONSE_SUCCESS);
    }
  }

  //Else - msg[0] was timeout or other
  dataLengthRead = 0; //Inform caller that we weren't able to read anything

  return (RESPONSE_FAIL);
}

//Send the appropriate command to permanently kill a tag. If the password does not
//match the tag's pw it won't work. Default pw is 0x00000000
//Use with caution. This function doesn't control which tag hears the command.
//TODO Can we add ability to write to specific EPC?
uint8_t killTag(uint8_t *password, uint8_t passwordLength, uint16_t timeOut)
{
  uint8_t data[4 + passwordLength];

  data[0] = timeOut >> 8 & 0xFF; //Timeout msB in ms
  data[1] = timeOut & 0xFF; //Timeout lsB in ms
  data[2] = 0x00; //Option initialize

  //Splice password into array
  for (uint8_t x = 0 ; x < passwordLength ; x++)
    data[3 + x] = password[x];

  data[3 + passwordLength] = 0x00; //RFU

  sendMessage(TMR_SR_OPCODE_KILL_TAG, data, sizeof(data), timeOut,ERROR_COMMAND_RESPONSE_TIMEOUT);

  if (msg[0] == ALL_GOOD) //We received a good response
  {
    uint16_t status = (msg[3] << 8) | msg[4];

    if (status == 0x0000)
      return (RESPONSE_SUCCESS);
  }

  //Else - msg[0] was timeout or other
  return (RESPONSE_FAIL);
}

//Checks incoming buffer for the start characters
//Returns 1 if a new message is complete and ready to be cracked
bool check()
{
  while ( available(_nanoSerial) )
  {
    uint8_t incomingData = rfid_read(_nanoSerial);

    //Wait for header byte
    if (_head == 0 && incomingData != 0xFF)
    {
      //Do nothing. Ignore this byte because we need a start byte
    }
    else
    {
      //Load this value into the array
      msg[_head++] = incomingData;

      _head %= MAX_MSG_SIZE; //Wrap variable

      if ((_head > 0) && (_head == msg[1] + 7))
      {
        //We've got a complete sentence!

        //Erase the remainder of the array
        for (uint8_t x = _head ; x < MAX_MSG_SIZE ; x++)
          msg[x] = 0;

        _head = 0; //Reset

		//Used for debugging: Does the user want us to print the command to serial port?
		if (_printDebug == 1)
		{
		  printf("response: ");
		  printMessageArray();
		}

        return (1);
      }

    }
  }

  return (0);
}

//See parseResponse for breakdown of fields
//Pulls the number of EPC bytes out of the response
//Often this is 12 bytes
uint8_t getTagEPCBytes(void)
{
  uint16_t epcBits = 0; //Number of bits of EPC (including PC, EPC, and EPC CRC)

  uint8_t tagDataBytes = getTagDataBytes(); //We need this offset

  for (uint8_t x = 0 ; x < 2 ; x++)
    epcBits |= (uint16_t)msg[27 + tagDataBytes + x] << (8 * (1 - x));
  uint8_t epcBytes = epcBits / 8;
  epcBytes -= 4; //Ignore the first two bytes and last two bytes

  return (epcBytes);
}

//See parseResponse for breakdown of fields
//Pulls the number of data bytes out of the response
//Often this is zero
uint8_t getTagDataBytes(void)
{
  //Number of bits of embedded tag data
  uint8_t tagDataLength = 0;
  for (uint8_t x = 0 ; x < 2 ; x++)
    tagDataLength |= (uint16_t)msg[24 + x] << (8 * (1 - x));
  uint8_t tagDataBytes = tagDataLength / 8;
  if (tagDataLength % 8 > 0) tagDataBytes++; //Ceiling trick

  return (tagDataBytes);
}

//See parseResponse for breakdown of fields
//Pulls the timestamp since last Keep-Alive message from a full response record stored in msg
uint16_t getTagTimestamp(void)
{
  //Timestamp since last Keep-Alive message
  uint32_t timeStamp = 0;
  for (uint8_t x = 0 ; x < 4 ; x++)
    timeStamp |= (uint32_t)msg[17 + x] << (8 * (3 - x));

  return (timeStamp);
}

//See parseResponse for breakdown of fields
//Pulls the frequency value from a full response record stored in msg
uint32_t getTagFreq(void)
{
  //Frequency of the tag detected is loaded over three bytes
  uint32_t freq = 0;
  for (uint8_t x = 0 ; x < 3 ; x++)
    freq |= (uint32_t)msg[14 + x] << (8 * (2 - x));

  return (freq);
}

//See parseResponse for breakdown of fields
//Pulls the RSSI value from a full response record stored in msg
int8_t getTagRSSI(void)
{
  return (msg[12] - 256);
}

//This will parse whatever response is currently in msg into its constituents
//Mostly used for parsing out the tag IDs and RSSI from a multi tag continuous read
uint8_t parseResponse(void)
{
  //See http://www.thingmagic.com/images/Downloads/Docs/AutoConfigTool_1.2-UserGuide_v02RevA.pdf
  //for a breakdown of the response packet

  //Example response:
  //FF  28  22  00  00  10  00  1B  01  FF  01  01  C4  11  0E  16
  //40  00  00  01  27  00  00  05  00  00  0F  00  80  30  00  00
  //00  00  00  00  00  00  00  00  00  15  45  E9  4A  56  1D
  //  [0] FF = Header
  //  [1] 28 = Message length
  //  [2] 22 = OpCode
  //  [3, 4] 00 00 = Status
  //  [5 to 11] 10 00 1B 01 FF 01 01 = RFU 7 bytes
  //  [12] C4 = RSSI
  //  [13] 11 = Antenna ID (4MSB = TX, 4LSB = RX)
  //  [14, 15, 16] 0E 16 40 = Frequency in kHz
  //  [17, 18, 19, 20] 00 00 01 27 = Timestamp in ms since last keep alive msg
  //  [21, 22] 00 00 = phase of signal tag was read at (0 to 180)
  //  [23] 05 = Protocol ID
  //  [24, 25] 00 00 = Number of bits of embedded tag data [M bytes]
  //  [26 to M] (none) = Any embedded data
  //  [26 + M] 0F = RFU reserved future use
  //  [27, 28 + M] 00 80 = EPC Length [N bytes]  (bits in EPC including PC and CRC bits). 128 bits = 16 bytes
  //  [29, 30 + M] 30 00 = Tag EPC Protocol Control (PC) bits
  //  [31 to 42 + M + N] 00 00 00 00 00 00 00 00 00 00 15 45 = EPC ID
  //  [43, 44 + M + N] 45 E9 = EPC CRC
  //  [45, 46 + M + N] 56 1D = Message CRC

  uint8_t msgLength = msg[1] + 7; //Add 7 (the header, length, opcode, status, and CRC) to the LEN field to get total bytes
  uint8_t opCode = msg[2];

  //Check the CRC on this response
  uint16_t messageCRC = calculateCRC(&msg[1], msgLength - 3 ); //Ignore header (start spot 1), remove 3 bytes (header + 2 CRC)
  if ((msg[msgLength - 2] != (messageCRC >> 8)) || (msg[msgLength - 1] != (messageCRC & 0xFF)))
  {
    return (ERROR_CORRUPT_RESPONSE);
  }

  if (opCode == TMR_SR_OPCODE_READ_TAG_ID_MULTIPLE) //opCode = 0x22
  {
    //Based on the record length identify if this is a tag record, a temperature sensor record, or a keep-alive?
    if (msg[1] == 0x00) //Keep alive
    {
      //We have a Read cycle reset/keep-alive message
      //Sent once per second
      uint16_t statusMsg = 0;
      for (uint8_t x = 0 ; x < 2 ; x++)
        statusMsg |= (uint32_t)msg[3 + x] << (8 * (1 - x));

      if (statusMsg == 0x0400)
      {
        return (RESPONSE_IS_KEEPALIVE);
      }
      else if (statusMsg == 0x0504)
      {
        return (RESPONSE_IS_TEMPTHROTTLE);
      }
      else
      {
    	  return ERROR_UNKNOWN_OPCODE;
      }

    }
    else if (msg[1] == 0x08) //Unknown
    {
      return (RESPONSE_IS_UNKNOWN);
    }
    else if (msg[1] == 0x0a) //temperature
    {
        return (RESPONSE_IS_TEMPERATURE);
    }
    else //Full tag record
    {
      //This is a full tag response
      //User can now pull out RSSI, frequency of tag, timestamp, EPC, Protocol control bits, EPC CRC, CRC
      return (RESPONSE_IS_TAGFOUND);
    }
  }
  else
  {
    if (_printDebug == 1)
	{
		printf("Unknown opcode in response: 0x");
		//println(opCode, HEX);//TODO
	}
    return (ERROR_UNKNOWN_OPCODE);
  }

}

//Given an opcode, a piece of data, and the size of that data, package up a sentence and send it
void sendMessage(uint8_t opcode, uint8_t *data, uint8_t size, uint16_t timeOut, bool waitForResponse)
{
  msg[1] = size; //Load the length of this operation into msg array
  msg[2] = opcode;

  //Copy the data into msg array
  for (uint8_t x = 0 ; x < size ; x++)
    msg[3 + x] = data[x];
  //ets_printf("sendMessage/ SendCommand \n");
  sendCommand(timeOut, waitForResponse); //Send and wait for response
}

//Given an array, calc CRC, assign header, send it out
//Modifies the caller's msg array
void sendCommand(uint16_t timeOut, bool waitForResponse)
{
  msg[0] = 0xFF; //Universal header
  uint8_t messageLength = msg[1];

  uint8_t opcode = msg[2]; //Used to see if response from module has the same opcode

  //Attach CRC
  uint16_t crc = calculateCRC(&msg[1], messageLength + 2); //Calc CRC starting from spot 1, not 0. Add 2 for LEN and OPCODE bytes.
  msg[messageLength + 3] = crc >> 8;
  msg[messageLength + 4] = crc & 0xFF;

  //Used for debugging: Does the user want us to print the command to serial port?
  if (_printDebug == 1)
  {
    ets_printf("sendCommand: ");//TODO
    printMessageArray();
  }

  //Remove anything in the incoming buffer
  //TODO this is a bad idea if we are constantly readings tags
  //printf("1 \n");
  //while ( available(_nanoSerial)) read(_nanoSerial);
  while(available(_nanoSerial)) uart_flush_input(_nanoSerial->uartPort);
  //while(available(&serialConnectionNano)) read(&serialConnectionNano);
  //printf("2 \n");
  //Send the command to the module
  for (uint8_t x = 0 ; x < messageLength + 5 ; x++)
  {
	//ets_printf("Az msg tartalma : %u \n",msg[x]);
    rfid_write(_nanoSerial,msg[x]);
  }
  if (_printDebug == 1)ets_printf("3 \n");
  //There are some commands (setBaud) that we can't or don't want the response
  if (waitForResponse == 0)
  {
	//printf("4 \n");
	flush(_nanoSerial); //Wait for serial sending to complete
	return;
  }

  //For debugging, probably remove
  //for (uint8_t x = 0 ; x < 100 ; x++) msg[x] = 0;

  //Wait for response with timeout
  uint32_t startTime = millis();
  while ( available(_nanoSerial) == 0)
  {
	//printf("5 \n");
    if ( millis() - startTime > timeOut)
    {

      if (_printDebug == 1) printf("Time out 1: No response from module, actTime = %u startTIme = %u, timeOut = %u \n",millis(),startTime,timeOut);
      msg[0] = ERROR_COMMAND_RESPONSE_TIMEOUT;
      return;
    }
    //vTaskDelay(1/portTICK_PERIOD_MS);
    //ESP_ERROR_CHECK(esp_task_wdt_feed());
    esp_task_wdt_reset();
  }
  // Layout of response in data array:
  // [0] [1] [2] [3]      [4]      [5] [6]  ... [LEN+4] [LEN+5] [LEN+6]
  // FF  LEN OP  STATUSHI STATUSLO xx  xx   ... xx      CRCHI   CRCLO
  messageLength = MAX_MSG_SIZE - 1; //Make the max length for now, adjust it when the actual len comes in
  uint8_t spot = 0;
  if (_printDebug == 1)ets_printf("Tullepett az elso available-n \n");
  while (spot < messageLength)
  {
	  //printf("6 \n");
    if (millis() - startTime > timeOut)
    {
      if (_printDebug == 1) printf("Time out 2: Incomplete response");

      msg[0] = ERROR_COMMAND_RESPONSE_TIMEOUT;
      return;
    }

    if (available(_nanoSerial))
    {
    	//printf("7 \n");
      msg[spot] = rfid_read(_nanoSerial);

      if (spot == 1) //Grab the length of this response (spot 1)
        messageLength = msg[1] + 7; //Actual length of response is ? + 7 for extra stuff (header, Length, opcode, 2 status bytes, ..., 2 bytes CRC = 7)

      spot++;

      //There's a case were we miss the end of one message and spill into another message.
      //We don't want spot pointing at an illegal spot in the array
      spot %= MAX_MSG_SIZE; //Wrap condition
    }
    esp_task_wdt_reset();
  }
  if (_printDebug == 1)ets_printf("Tullepett a masodik available-n \n");
  //printf("11 \n");
  //Used for debugging: Does the user want us to print the command to serial port?
  if (_printDebug == 1)
  {
    printf("response: ");
    printMessageArray();
  }

  //Check CRC
  crc = calculateCRC(&msg[1], messageLength - 3); //Remove header, remove 2 crc bytes
  if ((msg[messageLength - 2] != (crc >> 8)) || (msg[messageLength - 1] != (crc & 0xFF)))
  {
    msg[0] = ERROR_CORRUPT_RESPONSE;
    if (_printDebug == 1) printf("Corrupt response");
    return;
  }
  //printf("8 \n");

  //If crc is ok, check that opcode matches (did we get a response to the command we sent or a different one?)
  if (msg[2] != opcode)
  {
	if (_printDebug == 1)  ets_printf("Az msg2 : %u , az opcode : %u \n",msg[2],opcode);
    msg[0] = ERROR_WRONG_OPCODE_RESPONSE;
    if (_printDebug == 1) ets_printf("Wrong opcode response");
    return;
  }
  if (_printDebug == 1) ets_printf("9 \n");

  //If everything is ok, load all ok into msg array
  msg[0] = ALL_GOOD;

}

//Print the current message array - good for debugging, looking at how the module responded
//TODO Don't hardcode the serial stream
void printMessageArray(void)
{
  if(_printDebug == 1) //If user hasn't enabled debug we don't know what port to debug to
  {
	uint8_t amtToPrint = msg[1] + 5;
	if (amtToPrint > MAX_MSG_SIZE) amtToPrint = MAX_MSG_SIZE; //Limit this size

	for (uint8_t x = 0 ; x < amtToPrint ; x++)
	{
	  printf(" [");
	  if (msg[x] < 0x10) 	printf("0 ");
	  else	  				printf("%u", msg[x]);
	  printf("]");//TODO
	}
	printf(" \n");
  }
}

//Comes from serial_reader_l3.c
//ThingMagic-mutated CRC used for messages.
//Notably, not a CCITT CRC-16, though it looks close.
static uint16_t crctable[] =
{
  0x0000, 0x1021, 0x2042, 0x3063,
  0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b,
  0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
};

//Calculates the magical CRC value
uint16_t calculateCRC(uint8_t *u8Buf, uint8_t len)
{
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0 ; i < len ; i++)
  {
    crc = ((crc << 4) | (u8Buf[i] >> 4)) ^ crctable[crc >> 12];
    crc = ((crc << 4) | (u8Buf[i] & 0x0F)) ^ crctable[crc >> 12];
  }

  return crc;
}

bool readSingleEPCForce(EPCData* out, uint32_t measureTimeMillis)
{
	/* kiolvasunk eg EPC-t, vagy lejar a parameterkent megadott ido.
	 * ha sikerul, 1-et ad vissza, es a parameterkent megadott pointerre kiirja az EPC-t
	 * ha nem, 0-val ter vissza */

	EPCData myEPC;
	uint32_t startTime = millis(); //az aktualis ido lementese, ehhez hasonlitva maradunk vagy lepunk ki
	uint8_t response = RESPONSE_FAIL;
	while( ((response = readTagEPC(myEPC.bytes, EPC_LENGTH, DEFAULT_DELAY_IN_FORCE_FUNCTIONS) ) != RESPONSE_SUCCESS ) && (millis()- startTime < measureTimeMillis) );
	if (response == RESPONSE_SUCCESS)
	{

		for (uint8_t x = 0; x < EPC_LENGTH; x++)
		{
			out->bytes[x] = myEPC.bytes[x];
		}
		_IF_IS_DEBUG_ENABLED_LVL2 ets_printf("Sikerult kiolvasni \n");
		return 1;
	}
	else
	{
		_IF_IS_DEBUG_ENABLED_LVL2 ets_printf("Nem sikerult kiolvasni \n");
		return 0;
	}
}

bool setupNano(long baudRate, uart_config_t* uart_config)
{
	/* A gyari setup fuggveny, parameterkent megadott baudrate-et allit be
	 *  a thingmagic Nano-n, ellenorzi hogy mukodik-e a kommunikacio */
	_IF_IS_DEBUG_ENABLED ets_printf("Belepett a setupNano-ba \n");

	/* Bekapcsolja a debugot, az eredeti megoldas atirasa, nem praktikus, de meghagytam igy.
	 * Ha megsem felel meg, a _printDebug valtozo deklaraciojat torolni kell, es a 0 ertekadast is a main-ben,
	 * majd  mindenhol le kell cserelni a kodban az IS_DEBUG_ENABLE makro-ra, es az enableDebug
	 * fuggvenyt torolni kell az rfid.c �s rfid.h fajlokbol */
	_IF_IS_DEBUG_ENABLED
		enableDebugging(&serialConnectionDebug);

	_IF_IS_DEBUG_ENABLED ets_printf("Tuljutott az enabledebugging-on \n");

	//About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
	while (available(&serialConnectionNano))
		uart_flush_input(_nanoSerial->uartPort);

	_IF_IS_DEBUG_ENABLED ets_printf("Tuljutott az available-n, jon a getversion \n");
	getVersion();
	_IF_IS_DEBUG_ENABLED ets_printf("kilepett a getversion-bol \n");
	if (msg[0] == ALL_GOOD)
	{
		printf("All good \n");
		//The M6E has these settings no matter what
		setTagProtocol(TAG_PROTOCOL); //Set protocol to GEN2

		setAntennaPort(); //Set TX/RX antenna ports to 1

		return (true);
	}
	else
		if (msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
		{
		_IF_IS_DEBUG_ENABLED_LVL2 ets_printf("Correct baudrate \n");
		//This happens if the baud rate is correct but the module is doing a continuous read
		/* Tuntess el minden RFID TAG-et a kozelebol */
		stopReading();

		_IF_IS_DEBUG_ENABLED_LVL2 ets_printf("Module continuously reading. Asking it to stop...");

		vTaskDelay(500 / portTICK_PERIOD_MS);
		esp_task_wdt_reset();
		}

		else
		{
			//The module did not respond so assume it's just been powered on and communicating at 115200bps
			_IF_IS_DEBUG_ENABLED_LVL2 ets_printf("Incorrect baudrate \n");
			_IF_IS_DEBUG_ENABLED ets_printf("A visszakapott uzenet: ");
			printErrorCode(msg[0]);

			uart_set_baudrate(DEFAULT_UART_PORT, DEFAULT_BADURATE_2);
			uart_param_config(DEFAULT_UART_PORT, uart_config);

			setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

			uart_set_baudrate(DEFAULT_UART_PORT, baudRate);
			uart_param_config(DEFAULT_UART_PORT, uart_config);

		}

	//Test the connection
	getVersion();
	if (msg[0] != ALL_GOOD)
		return (0); //Something is not right
	setTagProtocol(TAG_PROTOCOL); //Set protocol to GEN2
	setAntennaPort(); //Set TX/RX antenna ports to 1

	return (true); //We are ready to rock
}

bool readUsrDataFree(uint8_t* dataOut, uint8_t lenOut)
{
	/* Ha a kiolvasott user data nem 0, akkor 1-el ter vissza, egyebkent 0-val */
	uint8_t responseType;
	uint8_t myData[64];
	uint8_t myDataLength = sizeof(myData); //Tell readUserData to read up to 64 bytes
	bool wasntZero = 0;

	responseType = readUserData(myData, myDataLength, COMMAND_TIME_OUT); //readUserData will modify myDataLength to the actual # of bytes read
	if (responseType == RESPONSE_SUCCESS)
	{
		//Print User Data
		ets_printf("Size [ %u", myDataLength);
		ets_printf("] User data[");
		for (uint8_t x = 0; x < myDataLength; x++)
		{

			if (x < lenOut)
				dataOut[x] = myData[x];
			ets_printf("%c ", (char) myData[x]);
			if (myData[x] > 0)
				wasntZero = 1;
		}
		ets_printf("] \n");
	}
	else
	{
		if (_printDebug == 1)
			ets_printf("Error reading tag data, error code:");

		if (_printDebug == 1)
			printErrorCode(responseType);
		wasntZero = 1;
	}

	return wasntZero;
}

bool readUsrDataForce(uint8_t* dataOut, uint8_t lenOut,uint32_t measureTimeMillis)
{
	/* Ha a kiolvasott user data nem 0, akkor 1-el ter vissza, egyebkent 0-val. addig olvas tageket, amig nem talal
	 * egyet, aminek kiolvassa az user data-jat, vagy amik ki nem lep az idobol, amit kapott*/
	uint8_t myData[64];
	uint8_t myDataLength = sizeof(myData); //Tell readUserData to read up to 64 bytes
	bool wasntZero = 0;
	uint32_t startTime = millis(); //az aktualis ido lementese, ehhez hasonlitva maradunk vagy lepunk ki
	uint8_t response = RESPONSE_FAIL;

	while ((millis() - startTime < measureTimeMillis) && ((response = readUserData(myData, myDataLength, COMMAND_TIME_OUT)) != RESPONSE_SUCCESS))
	{
		esp_task_wdt_reset();
		vTaskDelay(DEFAULT_DELAY_IN_FORCE_FUNCTIONS / portTICK_PERIOD_MS);
	}; //readUserData will modify myDataLength to the actual # of bytes read

	//Print User Data
	ets_printf("Size [ %u", myDataLength);
	ets_printf("] User data[");
	for (uint8_t x = 0; x < myDataLength; x++)
	{
		if (x < lenOut)
			dataOut[x] = myData[x];
		ets_printf("%c ", (char) myData[x]);
		if (myData[x] > 0)
			wasntZero = 1;
	}
	ets_printf("] \n");

	return wasntZero;
}

void writeUsrDataFree(uint8_t* dataWrite, int len)
{
	len = (len / 2) * 2;
	uint8_t responseType = writeUserData(dataWrite, len, COMMAND_TIME_OUT);

	if (responseType == RESPONSE_SUCCESS)
		ets_printf("New Data Written! \n");
	else
	{
		ets_printf("Failed write \n");
	}

}

void writeUsrDataForce(uint8_t* dataWrite, int len,uint32_t measureTimeMillis)
{
	/* kiir "len" szamu bajtot a user data-ba,
	 * es addig nem all le, amig nem sikerul
	 */
	len = (len > 64) ? 64 : (len / 2) * 2; //csak 64 bajtnal nem tobb, paros szamu bajtot kuldhetunk ki
	uint8_t responseType = RESPONSE_FAIL;
	uint32_t startTime = millis();
	while( (( responseType = writeUserData(dataWrite, len, COMMAND_TIME_OUT) ) != RESPONSE_SUCCESS) && (millis() - startTime < measureTimeMillis) )
	{
		vTaskDelay(10 / portTICK_PERIOD_MS);
		esp_task_wdt_reset();
	}
	if( responseType == RESPONSE_SUCCESS)
	{
		ets_printf("Sikerult a User Data iras: [");
	}
	else
	{
		ets_printf("Nem sikerult a User Data iras: [");
	}

}
//TODO irni egy olyan fuggvenyt, ami kiolvassa a 2. Banktol a 3. vegeig, es igy meglesz az EPC es az User Data is, es lehet EPC alapjan szurni az USER data-t

bool readPWcontinouslyFree(uint8_t* killPw, uint8_t* accesPw)
{
	/* a bejovo 4 bajt-os tombokbe es a soros portokra
	 * is kiirja az elso szembe jovo tag jelszavait, ha szembe jon */
	uint8_t response;
	uint8_t myPW[4];
	uint8_t pwLength = sizeof(myPW);

	//Read Kill password
	response = readKillPW(myPW, pwLength, COMMAND_TIME_OUT);
	if (response == RESPONSE_SUCCESS)
	{
		ets_printf("Kill PW readed! \n");
		_IF_IS_DEBUG_ENABLED ets_printf("KillPW: [");
		for (uint8_t x = 0; x < pwLength; x++) {
			if (myPW[x] < 0x10) {
				_IF_IS_DEBUG_ENABLED ets_printf("0 ");
				killPw[x] = (uint8_t)"0";
			} else {
				_IF_IS_DEBUG_ENABLED ets_printf("%u ", myPW[x]);
				killPw[x] = myPW[x];
			}
		}
		_IF_IS_DEBUG_ENABLED ets_printf("] \n");
	}
	else
	{
		ets_printf("Failed read \n");
	}

	//Read Access PW
	pwLength = sizeof(myPW); //Reset this variable. May have been changed above.
	response = readAccessPW(myPW, pwLength, COMMAND_TIME_OUT);
	if (response == RESPONSE_SUCCESS)
	{
		ets_printf("Access PW readed! \n");
		_IF_IS_DEBUG_ENABLED ets_printf("AccessPW: [");
		for (uint8_t x = 0; x < pwLength; x++) {
			if (myPW[x] < 0x10) {
				_IF_IS_DEBUG_ENABLED ets_printf("0 ");
				accesPw[x] = (uint8_t) "0";
			} else {
				_IF_IS_DEBUG_ENABLED ets_printf("%u ", myPW[x]);
				accesPw[x] = myPW[x];
			}
		}
		_IF_IS_DEBUG_ENABLED ets_printf("] \n");
		return 1;
	}

	else
	{
		ets_printf("Failed read \n");
		return 0;
	}
}

void readPWcontinouslyForce(uint8_t* killPw, uint8_t* accesPw,  uint32_t measureTimeMillis)
{
	/* a bejovo 4 bajt-os tombokbe es a soros portokra
	 * is kiirja az elso szembe jovo tag jelszavait, amig ez nem tortenik meg, addig foglalja a vonalat */
	uint8_t response = RESPONSE_FAIL;
	uint8_t myPW[4];
	uint8_t pwLength = sizeof(myPW);

	//Read Kill password
	uint32_t startTime = millis();
	while ( ((response = readKillPW(myPW, pwLength, COMMAND_TIME_OUT)) != RESPONSE_SUCCESS) && (millis() - startTime < measureTimeMillis) )
	{
		vTaskDelay(10 / portTICK_PERIOD_MS);
		esp_task_wdt_reset();
	};
	if( response == RESPONSE_SUCCESS)
	{
		ets_printf("Kill PW readed! \n");
		_IF_IS_DEBUG_ENABLED ets_printf("KillPW: [");
		for (uint8_t x = 0; x < pwLength; x++)
		{
			if (myPW[x] < 0x10)
			{
				_IF_IS_DEBUG_ENABLED ets_printf("0 ");
				killPw[x] = (uint8_t)"0";
			}
			else
			{
				_IF_IS_DEBUG_ENABLED ets_printf("%u ", myPW[x]);
				killPw[x] = myPW[x];
			}
		}
		_IF_IS_DEBUG_ENABLED ets_printf("] \n");
	}
	else
	{
		ets_printf("Time is up ! \n");
	}


	//Read Access PW
	startTime = millis();
	pwLength = sizeof(myPW); //Reset this variable. May have been changed above.
	response = readAccessPW(myPW, pwLength, COMMAND_TIME_OUT);
	while( ( (response = readAccessPW(myPW, pwLength, COMMAND_TIME_OUT) ) != RESPONSE_SUCCESS) && (millis() - startTime < measureTimeMillis) )
	{
		vTaskDelay(10 / portTICK_PERIOD_MS);
		esp_task_wdt_reset();
	}
	if( response == RESPONSE_SUCCESS )
	{
		ets_printf("Access PW read! \n");
		_IF_IS_DEBUG_ENABLED ets_printf("AccessPW: [");
		for (uint8_t x = 0; x < pwLength; x++)
		{
			if (myPW[x] < 0x10)
			{
				_IF_IS_DEBUG_ENABLED ets_printf("0 ");
				accesPw[x] = (uint8_t)"0";
			}
			else
			{
				_IF_IS_DEBUG_ENABLED ets_printf("%u ", myPW[x]);
				accesPw[x] = myPW[x];
			}
		}
		_IF_IS_DEBUG_ENABLED ets_printf("] \n");
	}
	else
	{
		ets_printf("Time is up! \n");
	}

}

void writeKillPWForce(uint8_t* killPW, uint32_t measureTimeMillis)
{
	/* addig blokkolja a vonalat, amig nem talal egy tag-et, ekkor ennek atirja a KillPW-jet es kilep, vagy ha lejar az ido amit megadtunk a keresesre*/
	uint32_t startTime = millis(); //az aktualis ido lementese, ehhez hasonlitva maradunk vagy lepunk ki
	uint8_t response = RESPONSE_FAIL;
	while ((millis() - startTime < measureTimeMillis) && ((response = writeKillPW(killPW, KILL_PASSWORD_LENGTH,	COMMAND_TIME_OUT)) != RESPONSE_SUCCESS))
	{
		vTaskDelay(DEFAULT_DELAY_IN_FORCE_FUNCTIONS / portTICK_PERIOD_MS);
		esp_task_wdt_reset();
	};

	_IF_IS_DEBUG_ENABLED_LVL2
	{
		if (response == RESPONSE_SUCCESS)
			ets_printf("New Kill PW Written! \n");
		else
			ets_printf("Time is Up! \n");
	}
}

void writeAccesPWForce(uint8_t* accesPW, uint32_t measureTimeMillis)
{
	/* addig blokkolja a vonalat, amig nem talal egy tag-et, ekkor ennek atirja a AccesPW-jet es kilep, vagy ha lejar a megadott ido " measureTimeMillies*/
	uint32_t startTime = millis(); //az aktualis ido lementese, ehhez hasonlitva maradunk vagy lepunk ki
	uint8_t response = RESPONSE_FAIL;
	while ((millis() - startTime < measureTimeMillis) && ((response = writeAccessPW(accesPW, ACCES_PASSWORD_LENGTH, COMMAND_TIME_OUT)) != RESPONSE_SUCCESS))
	{
		vTaskDelay(DEFAULT_DELAY_IN_FORCE_FUNCTIONS / portTICK_PERIOD_MS);
		esp_task_wdt_reset();
	};

	_IF_IS_DEBUG_ENABLED_LVL2
	{
		if (response == RESPONSE_SUCCESS)
			ets_printf("New Access PW Written! \n");
		else
			ets_printf("Time is Up! \n");
	}

}

bool setUpNanoAndConnections()
{
	/* Itt allitom be az RFID olvasohoz az UART vonalat, az RFID-t itt kapcsolom be,
	 * itt kapnak erteket a hozza kapcsolodo valtozok,
	 * itt tortenik:	- az UART sebesseg allitas,
	 * 					- a regiaoallitas,
	 * 					- es az olvasasi sebesseg allitas */
	_head = 0;
	_printDebug = IS_DEBUG_ENABLED;
	uart_config_t uart_config = {
			.baud_rate = DEFAULT_BADURATE_1,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
	/* RFID engedelyezo labanak beallitasa */
	gpio_pad_select_gpio(RFID_EN_GPIO);
	gpio_set_direction(RFID_EN_GPIO, GPIO_MODE_OUTPUT);
	_IF_IS_DEBUG_ENABLED ets_printf("RFID EN konfiguralas \n");

	/* RFID kikapcsolas */
	gpio_set_level(RFID_EN_GPIO, 0);
	vTaskDelay(200 / portTICK_PERIOD_MS);
	_IF_IS_DEBUG_ENABLED ets_printf("RFID EN deaktivalas \n");

	/* RFID engedelyezes*/
	//gpio_reset_pin(RFID_EN_GPIO);
	gpio_set_level(RFID_EN_GPIO, 1);
	vTaskDelay(400 / portTICK_PERIOD_MS);
	_IF_IS_DEBUG_ENABLED ets_printf("RFID EN bekapcsolva \n");

	uart_param_config(DEFAULT_UART_PORT, &uart_config);
	uart_set_pin(DEFAULT_UART_PORT, DEFAULT_UART_PORT_TXD, DEFAULT_UART_PORT_RXD, DEFAULT_UART_PORT_RTS, DEFAULT_UART_PORT_CTS);
	uart_driver_install(DEFAULT_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);

	_nanoSerial = &serialConnectionNano;
	_nanoSerial->uartPort = DEFAULT_UART_PORT;

	vTaskDelay(500 / portTICK_PERIOD_MS);

	/* Thingmagic konfiguralasa, baudrate beallitasa es ellenorzese */
	if (setupNano(DEFAULT_BADURATE_1, &uart_config) == false)
	{
		printf("Module failed to respond. Please check wiring.");
		/* Itt ugyis ujraindul */
		if (REBOOT_SYSTEM_IF_RFID_SETUP_FAILS == 1)
			esp_restart();
		return 1;
	}
	else
	{
		setRegion(REGION_EUROPE); //Set to North America
		_IF_IS_DEBUG_ENABLED ets_printf("Megvolt a regioallitas \n");

		/* itt lehet allitani az antenna altal felvett teljesitmenyt, 5-27dBm-ig, 0-100as skalan */
		setReadPower(readPower);
		/* Max Read TX Power is 27.00 dBm, ezt egy 0-100-as skalan lehet megadni */

		_IF_IS_DEBUG_ENABLED ets_printf("Megvolt a readPower allitas	 \n");
		return 0;
	}

}

void insertEPCDataToOrderedArray(EPCData in, EPCData* orderedArray,uint8_t* orderedArraySize) {
	/* a kapott orderedArray tomb utolso eleme utan beilleszti a kapott in strukturat ha az
	 * meg nem szerepel a tombben, es noveli a tomb meretet mutato orderedArraySize valtozot */
	for (uint8_t i = 0; i < *orderedArraySize; i++)
	{
		for (int j = 0; j < EPC_LENGTH; j++)
		{
			if (in.bytes[j] != orderedArray[i].bytes[j])
				break;
			if (j == 11)
				return;
		}
	}
	for (int i = 0; i < EPC_LENGTH; i++)
	{
		orderedArray[*orderedArraySize].bytes[i] = in.bytes[i];
	}
	(*orderedArraySize)++;
}

void readEPCsForce(uint32_t measureTimeMillis, uint8_t* orderedArraySize,EPCData* orderedArray)
{
	/* meghivaskor az orderArraySize tarolja a tomb meretet, ezen
	 * felul nem szabad indexelni */
	startReading();
	EPCData actualEPC;
	uint8_t maxNum = *orderedArraySize; //tarolja hogy mekkkora a tomb, ezen felul nem szabad indexelni
	*orderedArraySize = 0;

	uint32_t startTime = millis(); //az aktualis ido lementese, ehhez hasonlitva maradunk vagy lepunk ki
	while ((millis() - startTime < measureTimeMillis) && (*orderedArraySize < maxNum))
	{
		if (check() == true) //Check to see if any new data has come in from module
		{
			uint8_t responseType = parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

			if (responseType == RESPONSE_IS_KEEPALIVE)
			{
				_IF_IS_DEBUG_ENABLED
					ets_printf("Scanning \n");
			}
			else
				if (responseType == RESPONSE_IS_TAGFOUND)
				{
					//If we have a full record we can pull out the fun bits
					int rssi = getTagRSSI(); //Get the RSSI for this tag read

					long freq = getTagFreq(); //Get the frequency this tag was detected at

					long timeStamp = getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

					uint8_t tagEPCBytes = getTagEPCBytes(); //Get the number of bytes of EPC from response

					_IF_IS_DEBUG_ENABLED ets_printf("rssi[ %d ] ", rssi);
					_IF_IS_DEBUG_ENABLED ets_printf("freq[ %ld ] ", freq);
					_IF_IS_DEBUG_ENABLED ets_printf("time[ %ld ] ", timeStamp);

					//Print EPC bytes, this is a subsection of bytes from the response/msg array
					_IF_IS_DEBUG_ENABLED ets_printf("epc[");
					if (tagEPCBytes > EPC_LENGTH) tagEPCBytes = EPC_LENGTH; //csak hogy ne cimezzuk tul az EPCData-t
					for (uint8_t x = 0; x < tagEPCBytes; x++)
					{
						if (msg[31 + x] < 0x10)
						{
							_IF_IS_DEBUG_ENABLED ets_printf("0 "); //Pretty print
							actualEPC.bytes[x] = 0;

						}
						else
						{
							_IF_IS_DEBUG_ENABLED ets_printf("%x ", msg[31 + x]);
							actualEPC.bytes[x] = msg[31 + x];
						}
					}
					_IF_IS_DEBUG_ENABLED ets_printf("%u : count \n");
					insertEPCDataToOrderedArray(actualEPC, orderedArray,orderedArraySize);

					_IF_IS_DEBUG_ENABLED ets_printf("] \n");

				}
				else
					if (responseType == ERROR_CORRUPT_RESPONSE)
					{
						_IF_IS_DEBUG_ENABLED
						ets_printf("Bad CRC");
					}
					else
					{
						//Unknown response
						_IF_IS_DEBUG_ENABLED ets_printf("Unknown error");
					}
		}
		else
		{
			_IF_IS_DEBUG_ENABLED ets_printf("0");
		}

	}
	stopReading();
}

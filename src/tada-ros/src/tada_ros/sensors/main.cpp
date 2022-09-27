// C standard
#include <stdlib.h>
#include <stdint.h>
#include "string.h"
#include <math.h>
#include "stdio.h"



// IMU
#include "MPU6050.h"
#define IMU_ADDRESS                (MPU6050_ADDRESS_AD0_LOW<<1)  // address pin low (GND), Accelerometer, Gyroscope, Thermometer
#define BAROMETER_ADDRESS          (0x77<<1)                     // address of BMP-180 Pressure Sensor
#define MAG_ADDRESS                (MPU6050_RA_MAG_ADDRESS<<1)   // address of magnetometer onboard MPU-9150



// Trajectory
#include "trajectory.h"
#include "matrix.h"

// RTC
// #define RTC_CLOCK_SOURCE_LSE            // LSE used as RTC source clock
#define RTC_CLOCK_SOURCE_LSI           // LSI used as RTC source clock

// main.c
#define grav												9.8027				// [m/s^2] local constant
#define RED													1   					// LED enumeration
#define BLUE 												2   					// LED enumeration
// #define colorSignalMax              grav
// #define colorSignalMin              0.0
// #define colorSignalRange            (colorSignalMax-colorSignalMin)
#define WRITEBLOCKSIZE 8192 //16 sectors
#define PACKETLENGTH 23 //size, year, month, day, hour, minute, second, subsecond(4), Ax(2), Ay(2), Az(2), Gx(2), Gy(2), Gz(2)

//**********************//
// INITIALIZE VARIABLES //
//**********************//
//----- general variables -----//
char ss[256]; // character array for USART_puts()
char dataString[256]; // character array for IMUdata storage
char serialString[256];
char str[256];
uint8_t i = 0;
uint8_t j = 0;
uint8_t regData;
uint8_t received_data[20]; // buffer to receive data
int8_t received_byte = 0.0;
int16_t data1 = 0;
float colorVal = 0;
uint8_t colorInterval[][3] = {
	{0, 0, 255},    // BLUE
	{0, 255, 0},    // GEEEN
	{255, 255, 0},  // YELLOW
	{255, 0, 0},    // RED
	{255, 255, 255} // WHITE
};
uint8_t numColorIntervals = sizeof(colorInterval)/3;
uint16_t motorCount = 0;
uint8_t state = 1;
UINT bw = 0;

// uint8_t RGB_Value[3] = {0};
float Ar, Ag, Ab, Br, Bg, Bb;
// float Ag;
// float Ab;
// float Br;
// float Bg;
// float Bb;
float range;
float point1;
float point2;

// float colorPoints[5] = {0.0, 0.33*colorSignalRange, 0.67*colorSignalRange, 1.0*colorSignalRange, 1.3*colorSignalRange};
float colorPoints[5] = {0.0, 0.33, 0.67, 1.0, 1.3};

//***** buffer logging related variables *****//
int ii = 0;
char sensor_log_string[WRITEBLOCKSIZE]={0};
volatile int sensor_pos = 0;		  // PGA
volatile int sensor_next_save_start = 0;	// PGA - for Redoing the Save so as to Never Get Behind (write "as fast as possible")
volatile int sensor_next_save_end = 0;
char sensor_chars[PACKETLENGTH] = {0};	// PGA
int endsv;


//***** configuration file related variables *****//
char config_string[8192] = {0}; //pre-allocate??
char logFileHeader[8192] = {0}; //pre-allocate??
UINT readByte = 0;
unsigned int* Set_LogRate;
unsigned int* Set_GyroRange;
unsigned int* Set_AccelRange;
unsigned int* Log_RTCdate;
unsigned int* Log_RTCtime;
unsigned int* Log_RTCsubsec;
unsigned int* Log_MPU6050Accel;
unsigned int* Log_MPU6050Gyro;


//***** ADC related variables *****//
uint16_t ADC_CH11 = 0;
uint16_t ADC_CH13 = 0;

//*****VCP related variables*****//
uint8_t rxBuf[256]; // buffer to store sequential bytes incoming from USB
uint32_t iRx;          // receive buffer index
#define MAX_STRLEN 1 // this is the maximum string length of our string in characters
volatile char received_string[MAX_STRLEN+1]; // this will hold the received string
char control_string[MAX_STRLEN+1];
char rd[256];

// motor control over VCP
char cmd = 0;
char num = 0;
uint8_t motor = 0;
uint8_t direction = 1;
int8_t duty = 0;
char dutycycle[3] = {0};
char strMotorDirection[3] = {0};
char strMotor[1] = {0};

//----- Accelerometer related variables -----//
float A[3] = {0}; // holds XYZ-axis linear acceleration data reading
float accelScaleFactor = (1.0/2048.0*grav);  // Accel[g] = ( ACCEL_xOUT/2048[LSB/mg] ) //for full scale range setting = +/- 16g
float T_mpu = 0.0; // holds temperature data reading from MPU-9150

//----- Gyroscope related variables -----//
float G[3] = {0.0}; // holds XYZ-axis angular rate data reading
float gyroScaleFactor_Rads = (3.14159265/180)*(1/16.4);  //Angular Velocity[deg/s] = (GYRO_xOUT/16.4[LSB/(deg/s)])  (3.14159265/180)*

//----- SD Card related variables -----//
FATFS FatFs; //Fatfs object
FIL fil, config; // File object
FRESULT  fr;
FILINFO fno;
uint8_t logCnt = 0;
uint8_t logToSD = 1;
const uint8_t SZ_TBL = 20;
DWORD clmt[20];                    /* Cluster link map table buffer */
char file_name[32] = {0};

//----- RTC related variables -----//
char timeData = 0;
char Year[4] = {0};
char Month[2] = {0};
char Date[2] = {0};
char hour[2] = {0};
char minute[2] = {0};
char second[2] = {0};
unsigned int* yr = 0;
unsigned int* mo = 0;
unsigned int* day = 0;
unsigned int* hr = 0;
unsigned int* min = 0;
unsigned int* sec = 0;
uint32_t fracSec = 0;
uint32_t subSec = 0;
float subSecFrac;
uint8_t diff = 0;
float floatFracSec = 0;

RTC_InitTypeDef RTC_InitStructure;
RTC_TimeTypeDef RTC_TimeStructure;
RTC_DateTypeDef RTC_DateStructure;
uint32_t uwAsynchPrediv = 0;
uint32_t uwSynchPrediv = 0;
uint32_t uwSecondfraction = 0;

//----- integration related variables -----//
long count = 0;
double Q[4];
double V[3];
double P[3];
struct kalman K;
struct Multi_classifier_linearSVM classifiers[5];
char class = 0;
double score[3];
double predictor[3];
double features[25];
double walkingSpeed[1];
// stance or swing, 0 for stance, 1 for swing

//----- State and Stiffness related variables ----//
float estimatedWalkingSpeed; // defaults to 1.0 because it's a nice modest value for walking speed.
const double absoluteMinStiffness = 10; // N/mm min from compression testing data
const double absoluteMaxStiffness = 32; // N/mm max from compression testing data
double stiffnessRange;
double desiredStiffness;
double estimatedStiffness;
double minStiffness;
double maxStiffness;
double desiredFulcrumPosition;
double vMag;
double meanWalkingSpeed;
double speeds[3];
int loopCount = 0;
const int stiffPosition = 75;
const int softPosition = 3400;

//----- Configuration File Variables -----//
double subjectMass = 75;
double lowSpeed = 0.9;
double highSpeed = 1.5;
int numStrides = 3;
int mapDir = 1; // -1: faster=softer, 0: neutral, 1: faster=stiffer
int vsfState = 1; 	// 1 is on, 0 is off
int stiffnessSetting = 1;
char configText[256];

//----- classification related variables -----//


//***********************//
//       OPTIONS         //
//***********************//
int printFlag_baroCalc = 0;       // set to print intermediate values
int printFlag_MPUregSettings = 0; // set to print MPU6050 configuration register values
int printFlag_outputGyro = 0;     // set to print Gyroscope data scaled to real units
int printFlag_outputAccel = 0;    // set to print Accelerometer data scaled to real units
int runFlag_timerLoop = 1;        // set to run timer loop
int useUSART = 1;


//***********************//
// FUNCTION DECLARATIONS //
//***********************//
void dDelay(__IO uint32_t nCount);


//***********************//
// FUNCTION DEFINITIONS  //
//***********************//

// ***** USART communication ***** //
void Config_USART(void)
{ // configure USART2; TX on PA2.  Set baud rate: <115200>

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* enable peripheral clock for USART2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);


	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* GPIOA Configuration:  USART2 TX on PA2 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //GPIO_Mode_AF
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART2 pins to AF2 */
	// TX = PA2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	/* Enable USART2 receive interrupt*/
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // enable the USART2 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART2 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART2 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART2 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE); // enable USART2

}

void USART_puts(USART_TypeDef* USARTx, char *s)
{ // stuff for USART output; iterate through a string and send each character
	if (useUSART){
		while(*s){
			// wait until data register is empty
			while( !(USARTx->SR & 0x00000040) );
			USART_SendData(USARTx, *s);
			s++;
		}
	}
}

// ***** END USART communication ***** //

// ***** VCP communication ***** //
void VCP_send_str(uint8_t* buf)
{ // function for USB-serial communication
	uint32_t i=0;
	while(*(buf + i))
	{
		i++;
	}
	VCP_DataTx(buf, i);
}
// ***** END VCP communication ***** //

// ***** BEGIN I2C FUNCTIONS ***** //
void Config_I2C(I2C_TypeDef* I2Cx)
{ // configure I2Cx peripheral

	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	// enable clock for SCL and SDA pins on Port B
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// enable I2C peripheral clock for I2Cx...
	if(I2Cx == I2C1){
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	// setup SCL and SDA pins
	// SCL : PB6
	// SDA : PB7
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;						// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;			// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;					// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;						// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);									// init GPIOB

	// connect I2C1 pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA
	}

	if(I2Cx == I2C2){
	// enable APB1 peripheral clock for I2C2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	// setup SCL and SDA pins
	// SCL : PB10
	// SDA : PB11
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // we are going to use PB10 and PB11
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;					  	// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;				// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;						// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;							// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);										// init GPIOB

	// connect I2C2 pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2); // SDA
	}


	// configure I2C peripheral
	I2C_InitStruct.I2C_ClockSpeed = 400000; 																// 400kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;																	// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00; 																	// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;															 	// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; 	// set address length to 7 bit addresses
	I2C_Init(I2Cx, &I2C_InitStruct);								   											// init I2C

	// enable I2C peripheral
	I2C_Cmd(I2Cx, ENABLE);
}

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{ // Begin I2C transaction

/* This function issues a start condition and
 * transmits the slave address + R/W bit
 *
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */

	// wait until I2Cx is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// Send I2C START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2Cx EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);


	/* wait for I2Cx EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if(direction == I2C_Direction_Transmitter){

		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	}
	else if(direction == I2C_Direction_Receiver){

		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	}
}

void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{ // Write byte

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1
 *		data --> the data byte to be transmitted
 */
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t I2C_read_ack(I2C_TypeDef* I2Cx)
{ // Read byte; ACK

/* This function reads one byte from the slave device
 * and acknowledges the byte (requests another byte)
 */
	uint8_t data;
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{ // Read byte; NACK

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data
 */
	uint8_t data;
	// disabe acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	data = I2C_ReceiveData(I2Cx);
	return data;
}

void I2C_stop(I2C_TypeDef* I2Cx)
{ // Send I2Cx STOP Condition
/* This funtion issues a stop condition and therefore
 * releases the bus
 */
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

uint8_t I2C_readRegister(I2C_TypeDef* I2Cx, uint8_t devAddress, uint8_t regAddress)
{ // Read contents of a register
	uint8_t data;

	I2C_start(I2Cx, devAddress, I2C_Direction_Transmitter); // start a transmission in Master Transmitter mode
	I2C_write(I2Cx, regAddress); // write one byte to the slave
	I2C_stop(I2Cx); // stop the transmission

	I2C_start(I2Cx, devAddress, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	data = I2C_read_nack(I2Cx);
	I2C_stop(I2Cx); // stop the transmission

	return data;
}

void I2C_setBit(I2C_TypeDef* I2Cx, uint8_t devAddress, uint8_t regAddress, uint8_t bitPosition)
{ // Read-Modify-Write to set register bits

	uint8_t regData;

	// *** A delay is necessary after each Read, Modify, Write step
	// *** sprint + USART_puts causes a delay which can replace explicit delay function

	// Read
	regData = I2C_readRegister(I2Cx, devAddress, regAddress);
		dDelay(0xFFFF);
// 		sprintf(ss, "I2C Read Register \r\n");
// 		USART_puts(USART2, ss);

	// Modify
	regData |= bitPosition; // bitPosition is really a mask, eg. (1 << x)
		dDelay(0xFFFF);
// 		sprintf(ss, "I2C Modify Register \r\n");
// 		USART_puts(USART2, ss);

	// Write
	I2C_start(I2Cx, devAddress, I2C_Direction_Transmitter); // start a transmission in Master transmitter modex
	I2C_write(I2Cx, regAddress); // write one byte to the slave
	I2C_write(I2Cx, (uint8_t)(regData));
	I2C_stop(I2Cx); // stop the transmission
		dDelay(0xFFFF);
// 		sprintf(ss, "I2C Write Register \r\n");
// 		USART_puts(USART2, ss);
}

void I2C_clearBit(I2C_TypeDef* I2Cx, uint8_t devAddress, uint8_t regAddress, uint8_t bitPosition)
{ // Read-Modify-Write to clear register bits

	uint8_t regData;

	// Read
	regData = I2C_readRegister(I2Cx, devAddress, regAddress);

	// Modify
	regData &= ~bitPosition; // bitPosition is really a mask, eg. (1 << x)

	// Write
	I2C_start(I2Cx, devAddress, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2Cx, regAddress); // write one byte to the slave
	I2C_write(I2Cx, (uint8_t)(regData));
	I2C_stop(I2Cx); // stop the transmission
}

// ***** END I2C FUNCTIONS ***** //

// ***** USER FUNCTIONS ***** //
void ledON(uint8_t color)
{ // <color>: RED or BLUE
	if (color == RED)
	{
		GPIOB->BSRRH = (1<<14);  //RED, bit clear = on
	}
	else if (color == BLUE)
	{
		GPIOB->BSRRH = (1<<15);  //BLUE, bit clear = on
	}
}

void ledOFF(uint8_t color)
{ // <color>: RED or BLUE
	if (color == RED)
	{
		GPIOB->BSRRL = (1<<14);  //RED, bit clear = off
	}
	else if (color == BLUE)
	{
		GPIOB->BSRRL = (1<<15);  //BLUE, bit clear = off
	}
}

void flashLED(uint8_t color)
{ // <color>: RED or BLUE
	if (color == RED)
	{
		ledON(RED);
		dDelay(0x8FFFFF);
		ledOFF(RED);
		dDelay(0x8FFFFF);
	}
	else if (color == BLUE)
	{
		ledON(BLUE);
		dDelay(0x8FFFFF);
		ledOFF(BLUE);
		dDelay(0x8FFFFF);
	}
}

static void bufferdata( char data[], int n)
{ // BUFFER THE DATA programmatically into a single file.
	//Copy the data to a larger buffer. This keeps the number of 'saves' lower,
	//which results in less overruns in reading the sensors.
	for(ii = 0; ii<n; ii++)
	{
		sensor_log_string[sensor_pos] = data[ii] ;
		sensor_pos++ ;

		// If the buffer is full, reset to the beginning.
		if(sensor_pos == WRITEBLOCKSIZE)
		{
			sensor_pos = 0;
		}

		// Whether or not it recycle to the beginning of the buffer, if it crosses a sector edge (512 bytes), log up to that point.
		// IF it DID recycle top the beginning, this will become 0, and there is a case for that in the main file writing program.
		if(  (sensor_pos % 512) == 0 )
		{
			sensor_next_save_end = sensor_pos;	// this can have values of: 0, 512, 1024, 1536, 2048, 2560, 3072, 3584, 4096, 4608, 5120, 5632, 6144, 6656, 7168, 7680. (won't ever get to 8192, due to the preceding if statement).
		}
	}
}

void saven_sensorData( char * data, int n)
{ // write data from bufferdata to SD card
	f_write(&fil, data, n, &bw);
	f_sync(&fil);
}

void saveSD_flush_Buffer(void)
{ // step through bufferdata to save data to SD card
	// Check for an Available Block of Data
		endsv = sensor_next_save_end ;
		if ( endsv < sensor_next_save_start ) // If LESS THAN the start index, then it has rolled over. Save up to the end of the buffer.
		{
			//LEDon();
			saven_sensorData((char*) &sensor_log_string[sensor_next_save_start], WRITEBLOCKSIZE - sensor_next_save_start); // Save to the end of the buffer...

			sensor_next_save_start = 0; // then set the next write block to start at 0
		}

		// Now check again. If it had rolled over before, then now there will be data to write from the beginning. If not, there may still be data to write if end>start
		endsv = sensor_next_save_end;
		if ( endsv > sensor_next_save_start )
		{
			//LEDon();
			saven_sensorData((char*) &sensor_log_string[sensor_next_save_start] , endsv - sensor_next_save_start );

			sensor_next_save_start = endsv;

			if (sensor_next_save_start == WRITEBLOCKSIZE)
			{ sensor_next_save_start = 0 ;}
		}
}

void Read_All_Sensors(I2C_TypeDef* I2Cx)
{ // read every sensor and fill data structure


		/////// Read Accelerometer, Gyroscope, Thermometer ///////
		I2C_start(I2Cx, IMU_ADDRESS, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		I2C_write(I2Cx, MPU6050_RA_ACCEL_XOUT_H); // write one byte(address to start reading from) to the slave
		I2C_stop(I2Cx); // stop the transmission

	  // Read in data from registers 3Bh to 47h, in HIGH/LOW byte pairs
		I2C_start(I2Cx, IMU_ADDRESS, I2C_Direction_Receiver); // start a transmission in Master receiver mode

		received_data[0] = I2C_read_ack(I2Cx); // read ACCEL_XOUT [HIGH] ( 0x3B )
		received_data[1] = I2C_read_ack(I2Cx); // read ACCEL_XOUT [LOW]
		received_data[2] = I2C_read_ack(I2Cx); // read ACCEL_YOUT [HIGH]
		received_data[3] = I2C_read_ack(I2Cx); // read ACCEL_YOUT [LOW]
		received_data[4] = I2C_read_ack(I2Cx); // read ACCEL_ZOUT [HIGH]
		received_data[5] = I2C_read_ack(I2Cx); // read ACCEL_ZOUT [LOW]

		received_data[6] = I2C_read_ack(I2Cx); // read TEMP_OUT [HIGH]
		received_data[7] = I2C_read_ack(I2Cx); // read TEMP_OUT [LOW]

		received_data[8] = I2C_read_ack(I2Cx);   // read GYRO_XOUT [HIGH]
		received_data[9] = I2C_read_ack(I2Cx);   // read GYRO_XOUT [LOW]
		received_data[10] = I2C_read_ack(I2Cx);  // read GYRO_YOUT [HIGH]
		received_data[11] = I2C_read_ack(I2Cx);  // read GYRO_YOUT [LOW]
		received_data[12] = I2C_read_ack(I2Cx);  // read GYRO_ZOUT [HIGH]
		received_data[13] = I2C_read_nack(I2Cx); // read GYRO_ZOUT [LOW] // don't request another byte

		I2C_stop(I2Cx); // stop the transmission


		/////// Assemble the IMU data ///////
		A[0] = (float)(int16_t)((received_data[0] << 8) | received_data[1]); // Accelerometer X-Axis data
		A[1] = (float)(int16_t)((received_data[2] << 8) | received_data[3]); // Accelerometer Y-Axis data
		A[2] = (float)(int16_t)((received_data[4] << 8) | received_data[5]); // Accelerometer Z-Axis data

		T_mpu = (float)(int16_t)((received_data[6] << 8) | received_data[7]); // Thermometer data

		G[0] = (float)(int16_t)((received_data[8] << 8) | received_data[9]);   // Gyroscope X-Axis data
		G[1] = (float)(int16_t)((received_data[10] << 8) | received_data[11]); // Gyroscope Y-Axis data
		G[2] = (float)(int16_t)((received_data[12] << 8) | received_data[13]); // Gyroscope Z-Axis data


		for(i=0; i<=2; i++)
		{
			A[i] *= (float)(accelScaleFactor);
			G[i] *= (float)(gyroScaleFactor_Rads);
		}

 		T_mpu = (float)(T_mpu*340 + 35);


		// Update RTC structures for Date and Time
		RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
		RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
    subSec = RTC_GetSubSecond();

		// assemble self describing data byte packet
		//size, year, month, day, hour, minute, second, subsecond(4), Ax(2), Ay(2), Az(2), Gx(2), Gy(2), Gz(2)
	  sensor_chars[0] = PACKETLENGTH;
		sensor_chars[1] = RTC_DateStructure.RTC_Year;
		sensor_chars[2] = RTC_DateStructure.RTC_Month;
		sensor_chars[3] = RTC_DateStructure.RTC_Date;
		sensor_chars[4] = RTC_TimeStructure.RTC_Hours;
		sensor_chars[5] = RTC_TimeStructure.RTC_Minutes;
		sensor_chars[6] = RTC_TimeStructure.RTC_Seconds;
		sensor_chars[7] = (subSec >> 24) & 0xFF;
		sensor_chars[8] = (subSec >> 16) & 0xFF;
		sensor_chars[9] = (subSec >> 8) & 0xFF;
		sensor_chars[10] = subSec & 0xFF;
		sensor_chars[11] = ((uint16_t)A[0] >> 8) & 0xFF;
		sensor_chars[12] = (uint16_t)A[0] & 0xFF;
		sensor_chars[13] = ((uint16_t)A[1] >> 8) & 0xFF;
		sensor_chars[14] = (uint16_t)A[1] & 0xFF;
		sensor_chars[15] = ((uint16_t)A[2] >> 8) & 0xFF;
		sensor_chars[16] = (uint16_t)A[2] & 0xFF;
		sensor_chars[17] = ((uint16_t)G[0] >> 8) & 0xFF;
		sensor_chars[18] = (uint16_t)G[0] & 0xFF;
		sensor_chars[19] = ((uint16_t)G[0] >> 8) & 0xFF;
		sensor_chars[20] = (uint16_t)G[0] & 0xFF;
		sensor_chars[21] = ((uint16_t)G[0] >> 8) & 0xFF;
		sensor_chars[22] = (uint16_t)G[0] & 0xFF;

		// send sensor data packet to buffer
		bufferdata( &sensor_chars[0], sensor_chars[0]) ;

		return;
			
}

void Read_All_Sensors_toString(I2C_TypeDef* I2Cx)
{ // read every sensor, fill formatted string, fill buffer


		/////// Read Accelerometer, Gyroscope, Thermometer ///////
		I2C_start(I2Cx, IMU_ADDRESS, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		I2C_write(I2Cx, MPU6050_RA_ACCEL_XOUT_H); // write one byte(address to start reading from) to the slave
		I2C_stop(I2Cx); // stop the transmission

	  // Read in data from registers 3Bh to 47h, in HIGH/LOW byte pairs
		I2C_start(I2Cx, IMU_ADDRESS, I2C_Direction_Receiver); // start a transmission in Master receiver mode

		received_data[0] = I2C_read_ack(I2Cx); // read ACCEL_XOUT [HIGH] ( 0x3B )
		received_data[1] = I2C_read_ack(I2Cx); // read ACCEL_XOUT [LOW]
		received_data[2] = I2C_read_ack(I2Cx); // read ACCEL_YOUT [HIGH]
		received_data[3] = I2C_read_ack(I2Cx); // read ACCEL_YOUT [LOW]
		received_data[4] = I2C_read_ack(I2Cx); // read ACCEL_ZOUT [HIGH]
		received_data[5] = I2C_read_ack(I2Cx); // read ACCEL_ZOUT [LOW]

		received_data[6] = I2C_read_ack(I2Cx); // read TEMP_OUT [HIGH]
		received_data[7] = I2C_read_ack(I2Cx); // read TEMP_OUT [LOW]

		received_data[8] = I2C_read_ack(I2Cx);   // read GYRO_XOUT [HIGH]
		received_data[9] = I2C_read_ack(I2Cx);   // read GYRO_XOUT [LOW]
		received_data[10] = I2C_read_ack(I2Cx);  // read GYRO_YOUT [HIGH]
		received_data[11] = I2C_read_ack(I2Cx);  // read GYRO_YOUT [LOW]
		received_data[12] = I2C_read_ack(I2Cx);  // read GYRO_ZOUT [HIGH]
		received_data[13] = I2C_read_nack(I2Cx); // read GYRO_ZOUT [LOW] // don't request another byte

		I2C_stop(I2Cx); // stop the transmission


		/////// Assemble the IMU data ///////
		A[0] = (float)(int16_t)((received_data[0] << 8) | received_data[1]); // Accelerometer X-Axis data
		A[1] = (float)(int16_t)((received_data[2] << 8) | received_data[3]); // Accelerometer Y-Axis data
		A[2] = (float)(int16_t)((received_data[4] << 8) | received_data[5]); // Accelerometer Z-Axis data

		T_mpu = (float)(int16_t)((received_data[6] << 8) | received_data[7]); // Thermometer data

		G[0] = (float)(int16_t)((received_data[8] << 8) | received_data[9]);   // Gyroscope X-Axis data
		G[1] = (float)(int16_t)((received_data[10] << 8) | received_data[11]); // Gyroscope Y-Axis data
		G[2] = (float)(int16_t)((received_data[12] << 8) | received_data[13]); // Gyroscope Z-Axis data


		for(i=0; i<=2; i++)
		{
			A[i] *= (float)(accelScaleFactor);
			G[i] *= (float)(gyroScaleFactor_Rads);
		}

 		T_mpu = (float)(T_mpu*340 + 35);


		// Update RTC structures for Date and Time
		RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
		RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
		subSec = 16383 - RTC_GetSubSecond();

		// create sensor data string
		/*sprintf(dataString, "%02u/%02u/%02u, %02u:%02u:%02u:%u, %f, %f, %f, %f, %f, %f \r\n", RTC_DateStructure.RTC_Year,
																								RTC_DateStructure.RTC_Month,
																								RTC_DateStructure.RTC_Date,
																								RTC_TimeStructure.RTC_Hours,
																								RTC_TimeStructure.RTC_Minutes,
																								RTC_TimeStructure.RTC_Seconds,
																								subSec,
																								A[0], A[1], A[2], G[0], G[1], G[2]);
		// send sensor data string to buffer
		bufferdata( (char*) dataString, strlen(dataString));*/

		return;
}

void ADC_config(void)
{ // set up ADC input on PC1, PC3
	GPIO_InitTypeDef      GPIO_AdcInit;
  ADC_InitTypeDef  ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	// Put everything back to power-on defaults //
  ADC_DeInit();

	// Enable ADC1 clock & GPIOC Clock //
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// Configure the PC1 pin (ADC123_IN11) //
	GPIO_AdcInit.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
  GPIO_AdcInit.GPIO_Mode = GPIO_Mode_AN;
  GPIO_AdcInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_AdcInit);

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;  // Disable the scan conversion so we do one at a time
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;   // Don't do contimuous conversions - do them on demand
  ADC_InitStructure.ADC_ExternalTrigConv = 0; // Start conversion by software, not an external trigger
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // Conversions are 12 bit - put them in the lower 12 bits of the result
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

  // PCLK2 is the APB2 clock
	// APB2 prescalar = 2
	// PCLK2 = SYSCLK/2 = 84MHz
  // ADCCLK = PCLK2/2/8 = 168/8 = 10.5MHz
  ADC_CommonInitStructure.ADC_Mode = ADC_Prescaler_Div8;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Enable ADC1 //
  ADC_Cmd(ADC1, ENABLE);
}

void readADC1_Channel_11()
{ // read analog input on PC1
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_480Cycles);
  // Start the conversion
  ADC_SoftwareStartConv(ADC1);
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  // Get the conversion value
  ADC_CH11 = ADC_GetConversionValue(ADC1);
}

void readADC1_Channel_13()
{ // read analog input on PC3
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_112Cycles);
  // Start the conversion
  ADC_SoftwareStartConv(ADC1);
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  // Get the conversion value
  ADC_CH13 = ADC_GetConversionValue(ADC1);
}

void setDutyCycle_MotorA(uint16_t dutycycle)
{ // set dutycycle for PWM on TIM4
    TIM4->CCR4 = dutycycle*200/100; // (dutycycle/100)*ARR = value to set CCR4
}

void setRGB_R(uint16_t colorval)
{ // set dutycycle for PWM on TIM4
		colorval = 255-colorval;  // invert value because logic LOW = LED fully ON
    TIM5->CCR1 = colorval*200/255; // (colorval/255)*ARR = value to set CCR1
}

void setRGB_G(uint16_t colorval)
{ // set dutycycle for PWM on TIM4
		colorval = 255-colorval;  // invert value because logic LOW = LED fully ON
    TIM3->CCR1 = colorval*200/255; // (colorval/255)*ARR = value to set CCR1
}

void setRGB_B(uint16_t colorval)
{ // set dutycycle for PWM on TIM4
		colorval = 255-colorval;  // invert value because logic LOW = LED fully ON
    TIM5->CCR2 = colorval*200/255; // (colorval/255)*ARR = value to set CCR1
}

void setDutyCycle_MotorB(uint16_t dutycycle)
{ // set dutycycle for PWM on TIM1
    TIM1->CCR1 = dutycycle*200/100; // (dutycycle/100)*ARR = value to set CCR1
}

void motorCTL(uint8_t mot, uint8_t dir, uint8_t duty)
{ // apply control string commands to motors
	if(mot == 1) // Motor A
		{
			if(dir == 1) // FWD
			{
				GPIOB->BSRRH = (1<<7);   // PB7; BSRR H = LOW
				dDelay(0xFFF);
				GPIOB->BSRRL = (1<<8);  	// PB8; BSRR L = HIGH
				sprintf(strMotorDirection,"FWD");
			}
			else // REV
			{
				GPIOB->BSRRL = (1<<7);   // PB7; BSRR L = HIGH
				dDelay(0xFFF);
				GPIOB->BSRRH = (1<<8);  	// PB8; BSRR H = LOW
				sprintf(strMotorDirection,"REV");
			}
			setDutyCycle_MotorA(duty);
			sprintf(strMotor, "A");
		}
	else // Motor B
	  {
			if(dir == 1) // FWD
			{
				GPIOC->BSRRH = (1<<2);   // PC2; BSRR H = LOW
				dDelay(0xFFF);
				GPIOD->BSRRL = (1<<12);  // PD12; BSRR L = HIGH
				sprintf(strMotorDirection,"FWD");

			}
			else // REV
			{
				GPIOC->BSRRL = (1<<2);   // PC2; BSRR L = HIGH
				dDelay(0xFFF);
				GPIOD->BSRRH = (1<<12);  // PD12; BSRR H = LOW
				sprintf(strMotorDirection,"REV");
	}
			setDutyCycle_MotorB(duty);
			sprintf(strMotor, "B");
		}
}

void parseControlString(void)
{ // parse and interperet control string sent from PC or control device


	/*                 BASIC COMMANDS
	***************************************************
	* v = variable mode on
	* s = variable mode off
	* 1 = variable->fast=soft 	static->softest position
	* 2 = variable->neutral 	static->mid soft position
	* 3 = variable->fast=stiff 	static->mid position
	* 4 = variable->nothing		static->mid stiff position
	* 5 = variable->noting		static->stiffest position
	*
	*
	****************************************************
	* ONLY SEND ONE CHARACTER AT A TIME
	***************************************************/

	for (i=0; i<MAX_STRLEN+1; i++)
	{
		cmd = control_string[i];

		switch(cmd)
		{
		// set foot to variable mode
		case 'V':
		case 'v':
			vsfState = 1;
			break;

		// set foot to static mode
		case 'S':
		case 's':
			vsfState = 0;
			break;

		// put foot in stiffness setting 1 for static mode (softest)
		// set foot to speed mapping (faster=softer)
		case '1':
		case 'G':
		case 'g':
			stiffnessSetting = 1;
			mapDir = -1;
			break;

		// put foot in stiffness setting 2 for static mode (neutral)
		// set foot to speed mapping (neutral static)
		case '2':
		case 'H':
		case 'h':
			mapDir = 0;
			stiffnessSetting = 2;
			break;

		// put foot in stiffness setting 3 for static mode (stiffest)
		// set foot to speed mapping (faster=stiffer)
		case '3':
		case 'J':
		case 'j':
			mapDir = 1;
			stiffnessSetting = 3;
			break;

		// increase the maximum stiffness
		case 'Q':
		case 'q':
			maxStiffness += 2;
			break;

		// decrease the maximum stiffness
		case 'A':
		case 'a':
			maxStiffness -=2;
			break;

		// put foot in stiffness setting 8 (25% of range)
		case '8':
		case 'K':
		case 'k':
			stiffnessSetting = 8;
			break;

		// put foot in stiffness setting 9 (75% of range)
		case '9':
		case 'L':
		case 'l':
			stiffnessSetting = 9;
			break;

		}
	}
}

void dDelay(__IO uint32_t nCount)
{ // useful delay function
  while(nCount--)
  {
  }
}

void mapIMU_RGB(float signal, float min, float max)
{ // map GGB LED's to IMU channels


 	signal = fabs(signal);
	range = max-min;

	// setup interpolate interval from BLUE to GREEN: 0-0-255 to 0-255-0
	if (signal <= colorPoints[1]*range)
	{
		point1 = colorPoints[0]*range;
		point2 = colorPoints[1]*range;
		i = 0;
	}
	// setup interpolate interval from GREEN to YELLOW: 0-255-0 to 255-255-0
	else if (signal > colorPoints[1]*range && signal <= colorPoints[2]*range)
	{
		point1 = colorPoints[1]*range;
		point2 = colorPoints[2]*range;
		i = 1;
	}
	// setup interpolate interval from YELLOW to RED: 255-255-0 to 255-0-0
	else if (signal > colorPoints[2]*range && signal <= colorPoints[3]*range)
	{
		point1 = colorPoints[2]*range;
		point2 = colorPoints[3]*range;
		i = 2;
	}
	// setup interpolate interval from RED to WHITE: 255-0-0 to 255-255-255
	else if (signal > colorPoints[3]*range && signal <= colorPoints[4]*range)
	{
		point1 = colorPoints[3]*range;
		point2 = colorPoints[4]*range;
		i = 3;
	}
	// set to WHITE: 255-255-255
	else
	{
		point1 = signal;
		point2 = signal;
// 		colorVal = 1;
		i = 3;
	}

	colorVal = ((signal-point1)/(point2-point1));

	Ar = (float)colorInterval[i][0];   // First Color in interval
	Ag = (float)colorInterval[i][1];
	Ab = (float)colorInterval[i][2];

	Br = (float)colorInterval[i+1][0]; // Second Color in interval
	Bg = (float)colorInterval[i+1][1];
	Bb = (float)colorInterval[i+1][2];

	setRGB_R((uint8_t)(Ar + colorVal*(Br - Ar)));
	setRGB_G((uint8_t)(Ag + colorVal*(Bg - Ag)));
	setRGB_B((uint8_t)(Ab + colorVal*(Bb - Ab)));
}

void parseRTCstring(void)
{ // parse input string from rxBuf (VCP input) to synchronize RTC to subsecond accuracy


	// send Real Time as string via VCP in this format:
	// Year<xxxx>Month<xx>Date<xx>Hour<xx>Minute<xx>Second<xx>fractionSecond<xxxxxx>
	// ex: Y2015M01D05h12m13s00u500000

	// resolution of Fraction of Second: 1/(PREDIV_S +1)
	char str[27];

	memcpy(str, rxBuf, 27);

	sscanf(str, "Y%4uM%2uD%2uh%2um%2us%2uu%6u", &yr, &mo, &day, &hr, &min, &sec, &fracSec);

	// fill structures with data
	RTC_DateStructure.RTC_Year = (unsigned)(yr-2000);
	RTC_DateStructure.RTC_Month = (unsigned)mo;
	RTC_DateStructure.RTC_Date = (unsigned)day;
	RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);


	RTC_TimeStructure.RTC_Hours = (unsigned)hr;
	RTC_TimeStructure.RTC_Minutes = (unsigned)min;
	RTC_TimeStructure.RTC_Seconds = (unsigned)sec;
	RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);

	floatFracSec = ((float)fracSec)/1000000;  // fracSec comes in as microseconds

	RTC_SynchroShiftConfig(RTC_ShiftAdd1S_Set, (uint32_t)((1-floatFracSec)*(uwSynchPrediv + 1)));
}

void processConfigFile(void)
{ // parse config_string from config.txt file to store settings, generate and return header string

	/* config.txt file should be formatted as below:
	 ***********************************************
	 * //System_Configuration:
	 * SetLogRate uint8_t
	 * //Sensor_Configuration:
	 * Set_AccelRange uint8_t
	 * Set_GyroRange uint8_t
	 * //Data_Format:
	 * Log_RTCdate uint8_t
	 * Log_RTCtime uint8_t
	 * Log_MPU6050Accel uint8_t
	 * Log_MPU6050Gyro uint8_t
	 *
	 *** Changes to config.txt file should be reflected above for documentation, and below
	 *** as appropriate to sscanf(...), sprintf(...), and appended to the header string
	 */

	// parse config.txt
	sscanf(config_string, "%*s %*s %u %*s %*s %u %*s %u %*s %*s %u %*s %u %*s %u %*s %u", &Set_LogRate, &Set_AccelRange, &Set_GyroRange, &Log_RTCdate, &Log_RTCtime, &Log_MPU6050Accel, &Log_MPU6050Gyro);

		// initialize header string
	sprintf(logFileHeader, "\r\nSETTINGS: Log Rate: %u, Accel Range: %u, Gyro Range: %u \r\nDATA FORMAT: ", Set_LogRate, Set_AccelRange, Set_GyroRange);

	// append header string
	if(Log_RTCdate)
	{ strcat (logFileHeader,"RTC_Date "); }

	if(Log_RTCtime)
	{ strcat (logFileHeader,"RTC_Time "); }

	if(Log_MPU6050Accel)
	{ strcat (logFileHeader,"Ax Ay Az "); }

	if(Log_MPU6050Gyro)
	{ strcat (logFileHeader,"Gx Gy Gz "); }
}

int create_file(const char * name, const char * extension)
{
	int file_number=0;
	//char * file_name;

	//&file_name[0]=strcpy(&file_name[0], name);
	strcpy(&file_name[0], name);

	sprintf(file_name, "%s%03d%s", name, file_number, extension);

	sprintf(ss, "file_name: %s\r\n", file_name);
		USART_puts(USART2, ss);

	//Check to see if the file already exists in the root directory.
  fr = f_stat(file_name, 0);
	sprintf(ss, "f_stat Result: %d\r\n", fr);
		USART_puts(USART2, ss);

	while(!(f_stat(file_name, 0)))
    {
        file_number++;	//If the file already exists, increment the file number and check again.
        if(file_number == 999)
        {
            return 0;
        }
		//file_name=strcpy(file_name, name);
		strcpy(&file_name[0], name);
		sprintf(file_name+strlen(file_name), "%03d", file_number);
		//file_name = strcat(file_name, extension);
		strcat(&file_name[0], extension);
    }

	if(f_open(&fil, file_name, FA_CREATE_NEW | FA_READ | FA_WRITE) == FR_OK)
	{
		f_sync(&fil);
		return 1;
	}
	else { return 0; }
}

// **** Functions to drive fulcrum **** //
void DriveToFulcrumPosition(int targetFulcrumPosition)  // control motor to drive the carriage
{
		float posErrorGainP = 4; // Proportional gain on Position Error (%DutyCycle per Count)
	    float posErrorTolerance = 20; // deadband.
		uint8_t motorDirection = 1;
		uint16_t motorCommandInt = 0;
		float posError = 0;
		float motorCommandIdeal = 0;

		int posCurrent = ADC_CH11;

	    posError = (float) (targetFulcrumPosition-posCurrent);
		motorCommandIdeal = posError * posErrorGainP ;

		if (posError < -1*posErrorTolerance)
		{
			motorCommandInt = (uint16_t) fabs(motorCommandIdeal) ;
			motorDirection = 1;
		}
		else if (posError > posErrorTolerance)
		{
			motorCommandInt = (uint16_t) fabs(motorCommandIdeal) ;
			motorDirection = 0;
		}
		else
		{
			motorCommandInt = 0;
			motorDirection = 1;
		}
		motorCTL(1, motorDirection, motorCommandInt) ;
}

void DriveCarriageWithSinusoid(void)
{ // control motor to drive the carriage
		float posErrorGainP = 4; // Proportional gain on Position Error (%DutyCycle per Count)
	    float posErrorTolerance = 20; // deadband.
		uint8_t motorDirection = 1;
		uint16_t motorCommandInt = 0;
		float posError = 0;
		float motorCommandIdeal = 0;

		int posCurrent = ADC_CH11;

		int tSec =  RTC_TimeStructure.RTC_Seconds;
	  // Make a slowly oscillating reference. Note that because tSec increments in integer seconds, this will change the setting once per second.
// 		int posTarget = (int) (((fabs( (float)(tSec%60) - 30.0))/30.0 * 740) ) + 10; // Triangle Wave from 1300 to 2500 (obsolete)
		int posTarget = (int) (1600.0+1400.0*sin(2.0*3.1415926535*3.0/12.0 * (float)tSec));  // Sinusoid between 200 and 3000

	    posError = (float) (posTarget-posCurrent);
		motorCommandIdeal = posError * posErrorGainP ;

		if (posError < -1*posErrorTolerance)
		{
			motorCommandInt = (uint16_t) fabs(motorCommandIdeal) ;
			motorDirection = 1;
		}
		else if (posError > posErrorTolerance)
		{
			motorCommandInt = (uint16_t) fabs(motorCommandIdeal) ;
			motorDirection = 0;
		}
		else
		{
			motorCommandInt = 0;
			motorDirection = 1;
		}
		motorCTL(1, motorDirection, motorCommandInt) ;
}

double norm(double* Vec)
{
	double normvec;

	normvec = sqrt(Vec[0]*Vec[0] + Vec[1]*Vec[1] + Vec[2]*Vec[2]);

	return normvec;
}

float mean(int m, double a[]) {
    double sum=0;
    for(i=0; i<m; i++)
        sum+=a[i];
    return((double)sum/m);
}

double linearMap(double val_in, double in_min, double in_max, double out_min, double out_max) {
	return (val_in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double MapSpeedtoStiffness(double walkingSpeed)
{
	// linear interpolation around a mid-range stiffness good for preferred walking speed
	// accelerating and decelerating (walking speed less than
	if (walkingSpeed < lowSpeed) {
		if (mapDir == -1) { // -1: faster=softer
			desiredStiffness = maxStiffness;
		}
		else if (mapDir == 0) { // 0: neutral
			desiredStiffness = minStiffness+stiffnessRange*0.5;;
		}
		else if (mapDir == 1) { // 1: faster=stiffer
			desiredStiffness = minStiffness;
		}
	// walking
	} else {
		if (mapDir == -1) { // -1: faster=softer
			desiredStiffness = linearMap(walkingSpeed,lowSpeed,highSpeed,maxStiffness,minStiffness);
		}
		else if (mapDir == 0) { // 0: neutral
			desiredStiffness = minStiffness+stiffnessRange*0.5;;
		}
		else if (mapDir == 1) { // 1: faster=stiffer
			desiredStiffness = linearMap(walkingSpeed,lowSpeed,highSpeed,minStiffness,maxStiffness);
		}
	}

	// Check to make sure the calculated desired stiffness is within the limits
	if (desiredStiffness < minStiffness) {
			desiredStiffness = minStiffness;
		}
		else if (desiredStiffness > maxStiffness) {
			desiredStiffness = maxStiffness;
		}
	return desiredStiffness;
}

int MapStiffnesstoFulcrumPosition(double desiredStiffness, double minStiffness, double maxStiffness)
{
	const int stiffLimit = 100;
	const int softLimit = 3350;

	// should be non-linear interpolation because of beam stiffness equations...
	// approximate as linear for now
	desiredFulcrumPosition = linearMap(desiredStiffness,absoluteMinStiffness,absoluteMaxStiffness,softPosition,stiffPosition);

	if (desiredFulcrumPosition < stiffLimit) {
			desiredFulcrumPosition = stiffLimit;
		}
		else if (desiredFulcrumPosition > softLimit) {
			desiredFulcrumPosition = softLimit;
		}

	return desiredFulcrumPosition;
}

double WalkingSpeedRollingMean(double* walkingSpeed)
{	//Calculates an 'n' stride moving average of walking speed to smooth stiffness changes
	const int numStrides = 3;

	if (*walkingSpeed != speeds[0]) {
		for (i=numStrides; i>0; i--){
			speeds[i] = speeds[i-1];
		}
		speeds[0]=walkingSpeed[0];
	}
	if (speeds[numStrides-1]!=0){
		meanWalkingSpeed = mean(numStrides,speeds);
	} else {
		meanWalkingSpeed = *walkingSpeed;
	}

	return meanWalkingSpeed;
}

void readConfig() {

	char buffer[100]; // File copy buffer
	FRESULT fc; 	  // FatFs return code
	float val[7];
	int i=0;

	fc = f_open(&config, "config.txt", FA_READ);
	if (fc != FR_OK) {
		sprintf(ss, "\r\nError or EOF\r\n");
			USART_puts(USART2, ss);
	}

	while (f_gets(buffer, sizeof(buffer), &config)) {
		sscanf(buffer,"%f",&val[i]);
		i++;
	}
	f_close(&config);

	subjectMass = (double)val[0];
	lowSpeed = (double)val[1];
	highSpeed = (double)val[2];
	numStrides = (int)val[3];
	mapDir = (int)val[4];
	vsfState = (int)val[5];
	stiffnessSetting = (int)val[6];
}

void write2config() {
	FRESULT fc;
	fc = f_open(&config, "config.txt", FA_WRITE | FA_READ);
	if (fc != FR_OK) {
		sprintf(ss, "\r\nError or EOF at write open\r\n");
			USART_puts(USART2, ss);
	}

	f_lseek(&config,0);
	// Write values from config file to header of data file
	sprintf(configText, "%i %i %i\n", vsfState,mapDir,stiffnessSetting);
		f_puts(configText,&config);
	f_close(&config);
}

void setRange() {
	//minStiffness = 1.1*(-0.0001886*subjectMass*subjectMass+0.1366*subjectMass+1.5484); // original keel
	minStiffness = 1.1*(-0.00013072*subjectMass*subjectMass+0.1012*subjectMass+6.2915); // new 0.25in G10/FR4 keel with foam
	maxStiffness = absoluteMaxStiffness;
	stiffnessRange = maxStiffness-minStiffness;
	if (minStiffness < absoluteMinStiffness) {
			minStiffness = absoluteMinStiffness;
		}
	if (maxStiffness > absoluteMaxStiffness) {
			maxStiffness = absoluteMaxStiffness;
		}
}

double setStaticStiffness(int stiffnessSetting) {
	if (stiffnessSetting == 1) {
		desiredStiffness = minStiffness;
	}else if (stiffnessSetting==8) {
		desiredStiffness = minStiffness+stiffnessRange*0.2;
	} else if (stiffnessSetting==2) {
		desiredStiffness = minStiffness+stiffnessRange*0.5;
	} else if (stiffnessSetting==9) {
		desiredStiffness = minStiffness+stiffnessRange*0.8;
	} else if (stiffnessSetting==3) {
		desiredStiffness = maxStiffness;
	}
	return desiredStiffness;
}


// ***** END OF USER FUNCTIONS *****//

/***** CONFIGURATIONS *****/
 void Config_TIM7_Timer(void)
{ // configure TIM7; Set frequency <Hz> ; match DT to frequency
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; // enable TIM7 clock

    NVIC->ISER[1] |= 1 << 23; // enable the TIM7 IRQ - Actually position 54, but bit spilled over from ISER[0] into ISER[1]

	  TIM7->PSC = 1679;  /// TIMCLK = 84MHz, TIMCLK/PSC = 50000 Hz output
    TIM7->DIER |= TIM_DIER_UIE; // enable update interrupt
		//TIM7->ARR = 49999;  // 1Hz
		//TIM7->ARR = 4999;  // 10Hz
	  //TIM7->ARR = 2499;   // 20Hz
		//TIM7->ARR = 499;  // 100Hz
	  TIM7->ARR = 249;  // 200Hz
	  //TIM7->ARR = 99;  // 500Hz
		//TIM7->ARR = 49;  // 1000Hz
		//TIM7->ARR = 4;  // 10000Hz
    TIM7->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; // autoreload on, counter enabled
    TIM7->EGR = 1; // trigger update event to reload timer registers

		// to set Timer frequency, if PSC = 1679; set ARR = (50000/[Desired Frequency]) - 1

		/// PSC and ARR are 16-bit ~> 65535 value limit

		//TIM7->ARR = 99999;  // 10Hz
		//TIM7->PSC = 83;  /// TIMCLK = 84MHz, TIMCLK/PSC = 1MHz output counts at 1 MHz, 1us/tick

}

void Config_MPU6050(I2C_TypeDef* I2Cx)
{ // configure MPU-9150 IMU: Accelerometer, Gyroscope, Magnetometer, Thermometer

		//
		// Registers all (except Reg 107 and 117) default to 0x00 on reset but critical bit settings are still cleared
		//

	  //----- System Settings -----//
	  // Register 107 - MPU6050_RA_PWR_MGMT_1 - Power Management 1
	  I2C_setBit(I2Cx, IMU_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT); // Reset Registers
			dDelay(0xFFF);
		I2C_setBit(I2Cx, IMU_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_GX);          // Select gyroscope X-axis for clock source
	  	dDelay(0xFFF);
		I2C_clearBit(I2Cx, IMU_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT);     // Clear to enable temperature sensor
			dDelay(0xFFF);
		I2C_clearBit(I2Cx, IMU_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT);     // Clear to enable temperature sensor
			dDelay(0xFFF);

	  // Register 106 - MPU6050_RA_USER_CTRL - User Control
		I2C_clearBit(I2Cx, IMU_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT);     // Clear to allow I2C comm. with auxillary bus (magnetometer, temperature sensor): Precondition to enabling Bypass Mode
			dDelay(0xFFF);
	  I2C_clearBit(I2Cx, IMU_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_IF_DIS_BIT);     // Always clear this bit.  Just Do it!!!
			dDelay(0xFFF);

		// Register 55 - MPU6050_RA_INT_PIN_CFG - INT Pin / Bypass Enable Configuration
	  I2C_setBit(I2Cx, IMU_ADDRESS, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT); // Enable Bypass Mode
			dDelay(0xFFF);


		//----- Accelerometer and Gyroscope -----//
	  // Register 25 - MPU6050_RA_SMPLRT_DIV - Sample Rate Divider
		I2C_setBit(I2Cx, IMU_ADDRESS, MPU6050_RA_SMPLRT_DIV, 39); // Sample Rate = Gyroscope Output Rate/(1+SMPLRT_DIV) ; GOR = 1khz when DLPF is active
		dDelay(0xFFF);
		char a = I2C_readRegister(I2Cx, IMU_ADDRESS, MPU6050_RA_SMPLRT_DIV);
		sprintf(ss, "%x\n", a);
		USART_puts(USART2, ss);

		// Register 27 - MPU6050_RA_GYRO_CONFIG - Gyro Configuration
		I2C_setBit(I2Cx, IMU_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_2000); // Gyro = Full scale range; 2000deg/s
	  	dDelay(0xFFF);
		a = I2C_readRegister(I2Cx, IMU_ADDRESS, MPU6050_RA_GYRO_CONFIG);
		sprintf(ss, "%x\n", a);
		USART_puts(USART2, ss);

	  // Register 28 - MPU6050_RA_ACCEL_CONFIG - Accelerometer Configuration
		I2C_setBit(I2Cx, IMU_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_16);  // Accel = Full scale range; 16g
 	  	dDelay(0xFFF);
		I2C_setBit(I2Cx, IMU_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x8);
 	  	dDelay(0xFFF);
		I2C_setBit(I2Cx, IMU_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x18);
 	  	dDelay(0xFFF);

		I2C_clearBit(I2Cx, IMU_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_DHPF_RESET); // Accel High Pass Filter = none/disabled
			dDelay(0xFFF);
		a = I2C_readRegister(I2Cx, IMU_ADDRESS, MPU6050_RA_ACCEL_CONFIG);
		sprintf(ss, "%x\n", a);
		USART_puts(USART2, ss);

		// Register 26 - MPU6050_RA_CONFIG - Configuration
		I2C_clearBit(I2Cx, IMU_ADDRESS, MPU6050_RA_CONFIG, MPU6050_EXT_SYNC);          // Disable External Sync
	  	dDelay(0xFFF);
		I2C_clearBit(I2Cx, IMU_ADDRESS, MPU6050_RA_CONFIG, 0x00);       // Bandwidth of Low Pass Filter: Accel = 260Hz, Gyro = 256Hz
 			dDelay(0xFFF);
		a = I2C_readRegister(I2Cx, IMU_ADDRESS, MPU6050_RA_CONFIG);
		sprintf(ss, "%x\n", a);
		USART_puts(USART2, ss);
}

void Config_GPIO(void)
{ // configure GPIO ports for onboard LEDs

	// LED Red  = PB14
	// LED Blue = PB15

	GPIO_InitTypeDef  GPIO_InitStructure;

	/*GPIOB Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIOB->BSRRL = (1<<14) | (1<<15);  // TURN OFF AHEAD OF TIME

	/* Configure PB14, PB15*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Config_RGB(void)
{ // configure GPIO ports for RGB LED

	// LED Red   = PA0
	// LED Green = PA6
	// LED Blue  = PA1

	GPIO_InitTypeDef  GPIO_InitStructure;

	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIOA->BSRRL = (1<<0) | (1<<1) | (1<<6);  // TURN OFF AHEAD OF TIME

	/* Configure PA0, PA1, PA6 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Config_RGB_PWM(void)
{ // configure TIM3 CH1 (PA6), TIM5 CH1,2 (PA0,PA1) for PWM output

	// RED   : PA0 : TIM5, CH1
	// GREEN : PA6 : TIM3, CH1
	// BLUE  : PA1 : TIM5, CH2

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM3, TIM5 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // locations: stm32f4xx_rcc.h(stm32f4xx.h, X)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); // locations: stm32f4xx_rcc.h(stm32f4xx.h, X)


  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIOA Configuration: TIM3 CH1 (PA6), TIM5 CH1,2 (PA0,PA1) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Connect TIM pins to AF */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 200-1; //ARR - [ms], e.g. "20000-1" sets up a 20ms pulse (50Hz)
  TIM_TimeBaseStructure.TIM_Prescaler = 84-1; //PSC - *don't change* - sets up TIM_Period in [microsecond] units
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Period = 200-1; //ARR - [ms], e.g. "20000-1" sets up a 20ms pulse (50Hz)
  TIM_TimeBaseStructure.TIM_Prescaler = 84-1; //PSC - *don't change* - sets up TIM_Period in [microsecond] units
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: TIM 3 Channel 1 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);  // TIM_*OC1*Init... initializes Channel 1, change these to switch channel
		TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); // switch channel

	/* PWM1 Mode configuration: TIM 5 Channel 1 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OC1Init(TIM5, &TIM_OCInitStructure);  // TIM_*OC1*Init... initializes Channel 1, change these to switch channel
		TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable); // switch channel

	/* PWM1 Mode configuration: TIM 5 Channel 2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OC2Init(TIM5, &TIM_OCInitStructure);  // TIM_*OC2*Init... initializes Channel 2, change these to switch channel
		TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable); // switch channel

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
	/* TIM3 enable counter */
  TIM_Cmd(TIM5, ENABLE);

}

void Config_Motor_B(void)
{ // configure TIM1 CH1, on PA8 for PWM output (PWM_B, Motor B speed control)

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM1 clock enable */
//   RCC_AHB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // locations: stm32f4xx_rcc.h(stm32f4xx.h, X)
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // locations: stm32f4xx_rcc.h(stm32f4xx.h, X)


  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIOA Configuration: TIM1 CH1 (PA8) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Connect TIM1 pins to AF1 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 200-1; //ARR - [ms], e.g. "20000-1" sets up a 20ms pulse (50Hz)
  TIM_TimeBaseStructure.TIM_Prescaler = 84-1; //PSC - *don't change* - sets up TIM_Period in [microsecond] units
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel 1 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  // TIM_*OC1*Init... initializes Channel 1, change these to switch channel
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); // switch channel

  /* TIM1 enable counter */
  TIM_Cmd(TIM1, ENABLE);
// 	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	/* Configure Direction Pins */
	// Motor B Directions
	GPIOC->BSRRH = (1<<2);   // PC2; BSRR H = LOW
	dDelay(0xFFF);
	GPIOD->BSRRL = (1<<12);  // PD12; BSRR L = HIGH
	dDelay(0xFFF);
	GPIOB->BSRRL = (1<<6);   // PB6; BSRR L = HIGH

	}

void Config_Motor_A(void)
{ // configure TIM4 CH4, on PB9 for PWM output (PWM_A, Motor A speed control)

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM4 clock enable */
//   RCC_AHB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // locations: stm32f4xx_rcc.h(stm32f4xx.h, X)
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // locations: stm32f4xx_rcc.h(stm32f4xx.h, X)
   /////////////// fix ^^ ??

  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);


  /* GPIOB Configuration: TIM4 CH4 (PB9) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Connect TIM4 pins to AF2 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 200-1; //ARR - "200-1" gives 200uS pulse (5kHz) - [ms], e.g. "20000-1" sets up a 20ms pulse (50Hz)
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1; //PSC - *don't change* - sets up TIM_Period in [millisecond] units
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel 4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OC4Init(TIM4, &TIM_OCInitStructure); // Channel 4 is set here
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  // Channel 4

// 	TIM_ARRPreloadConfig(TIM4, ENABLE);  //????
  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);
// 	TIM_CtrlPWMOutputs(TIM4, ENABLE);

	// Motor A Directions
	GPIOB->BSRRL = (1<<8);  // PB8
	dDelay(0xFFF);
	GPIOB->BSRRH = (1<<7);  // PB7
	dDelay(0xFFF);
	GPIOB->BSRRH = (1<<6);   // PB6; BSRR L = HIGH

}

void Config_QEI(void)
{	// configure Quadrature Encoder Interface on TIM3
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// PA2, PA3 for encoder
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Connect TIM3 pins to AF
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);	// TIM3_CH1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); // TIM3_CH2

	TIM3->SMCR = 1;          // Encoder mode 1
  TIM3->CCER = 0;          // rising edge polarity
  TIM3->ARR = 0xFFFF;      // count from 0-ARR or ARR-0
  TIM3->CCMR1 = 0xC1C1;    // f_DTS/16, N=8, IC1->TI1, IC2->TI2
  TIM3->CNT = 0;           // Initialize counter
  TIM3->EGR = 1;           // Generate an update event
  TIM3->CR1 = 1;           // Enable the counter
}

static void RTC_Config(void)
{ // configure the RTC

  /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to RTC */
  PWR_BackupAccessCmd(ENABLE);

#if defined (RTC_CLOCK_SOURCE_LSI)  /* LSI used as RTC source clock*/
/* The RTC Clock may varies due to LSI frequency dispersion. */
  /* Enable the LSI OSC */
  RCC_LSICmd(ENABLE);

  /* Wait till LSI is ready */
  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

  /* ck_spre(1Hz) = RTCCLK(LSI) /(uwAsynchPrediv + 1)*(uwSynchPrediv + 1)*/
  uwSynchPrediv = 0x03FFF;
  uwAsynchPrediv = 0x01;

				#elif defined (RTC_CLOCK_SOURCE_LSE) /* LSE used as RTC source clock */
					/* Enable the LSE OSC */
					RCC_LSEConfig(RCC_LSE_ON);

					/* Wait till LSE is ready */
					while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
					{
					}

					/* Select the RTC Clock Source */
					RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
					/* ck_spre(1Hz) = RTCCLK(LSE) /(uwAsynchPrediv + 1)*(uwSynchPrediv + 1)*/
					uwSynchPrediv = 0x03FFF;
					uwAsynchPrediv = 0x01;

				#else
					#error Please select the RTC Clock source inside the main.c file
				#endif /* RTC_CLOCK_SOURCE_LSI */

   /* Configure the RTC data register and RTC prescaler */
  RTC_InitStructure.RTC_AsynchPrediv = uwAsynchPrediv;
  RTC_InitStructure.RTC_SynchPrediv = uwSynchPrediv;
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;

  /* Check on RTC init */
  if (RTC_Init(&RTC_InitStructure) == ERROR)
  {
		sprintf(ss, "RTC Prescaler Config failed \r\n");
		USART_puts(USART2, ss);
  }

  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

//   /* Enable The TimeStamp */
//   RTC_TimeStampCmd(RTC_TimeStampEdge_Falling, ENABLE);

	RTC_TimeStructInit(&RTC_TimeStructure);  //sets everything to 0 defaults
	RTC_DateStructInit(&RTC_DateStructure);
}

// ***** End of CONFIGURATIONS ***** //

//**********//
//   MAIN   //
//**********//
int main(void)
{
		int blah = 0;
	// Enable floating point
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // set CP10 and CP11 Full Access

	//***** Initialize the hardware *****//

 	Config_USART();
		sprintf(ss, "\r\n:::::WELCOME:::::\r\nNuBoard, Version 1.0\r\n2015 Intelligent Prosthetic Systems, LLC\r\n");
		USART_puts(USART2, ss);

		sprintf(ss, "\r\nInitializing System...\r\n\r");
		USART_puts(USART2, ss);

		sprintf(ss, "configured USART2\r\n");
		USART_puts(USART2, ss);

	Config_GPIO();
		sprintf(ss, "configured LED\r\n");
		USART_puts(USART2, ss);

 	Config_Motor_A();
 	  sprintf(ss, "configured Motor A\r\n");
 		USART_puts(USART2, ss);

 	Config_Motor_B();
 	  sprintf(ss, "configured Motor B\r\n");
 		USART_puts(USART2, ss);

// 	Config_QEI();
// 	  sprintf(ss, "configured QEI\r\n");
// 		USART_puts(USART2, ss);

 	ADC_config();
 	  sprintf(ss, "configured ADC\r\n");
 		USART_puts(USART2, ss);

	Config_I2C(I2C2);
		sprintf(ss, "configured I2C2\r\n");
		USART_puts(USART2, ss);

	Config_MPU6050(I2C2);
		sprintf(ss, "configured MPU6050 IMU\r\n");
		USART_puts(USART2, ss);

	TM_USB_VCP_Init();
		sprintf(ss, "configured VCP\r\n");
		USART_puts(USART2, ss);

 	Config_RGB_PWM();
		sprintf(ss, "configured RGB\r\n");
		USART_puts(USART2, ss);

	RTC_Config();
		sprintf(ss, "configured RTC\r\n");
		USART_puts(USART2, ss);
	RTC_Config();  // Repeated to configure prescalars properly

	sprintf(ss, "\r\nSystem is ready...\r\n");
		USART_puts(USART2, ss);

	/***** System is ready *****/
	flashLED(RED);
	flashLED(BLUE);

	sprintf(ss, "\r\nBefore 'if f_mount'...\r\n");
		USART_puts(USART2, ss);

	//***** Prepare SD card for logging data... *****//
	//Mount drive
	blah = f_mount(&FatFs, "", 1);
	sprintf(ss, "\r\n f_mount status %d ...\r\n",blah);
		USART_puts(USART2, ss);

	//Read configuration file from SD card
	readConfig();
	setRange();

	if (blah == FR_OK) //(f_mount(&FatFs, "", 1) == FR_OK)
	{
	sprintf(ss, "\r\nEntered 'if f_mount'...\r\n");
		USART_puts(USART2, ss);

		//Mounted OK, turn on RED LED
		ledON(RED);

	sprintf(ss, "\r\nAfter 'ledON(Red)'...\r\n");
		USART_puts(USART2, ss);

			if(create_file("Sen", ".dat")){

	sprintf(ss, "\r\nEntered 'if create_file'...\r\n");
		USART_puts(USART2, ss);

			//File opened, turn off RED and turn on BLUE led
			ledON(BLUE);
			ledOFF(RED);

			//If we put more than 0 characters (everything OK)
			if (f_puts("LOG RATE: 200 Hz\n", &fil) > 0)
			{
				//Turn on both leds
				ledON(BLUE);
				ledON(RED);
			}

	sprintf(ss, "\r\nBefore 'f_puts(FORMAT:...)'...\r\n");
		USART_puts(USART2, ss);

			// Write values from config file to header of data file
			sprintf(configText, "SUBJECT MASS: %f\n", subjectMass);
					f_puts(configText,&fil);
			sprintf(configText, "LOW SPEED MAP LIMIT: %f\n", lowSpeed);
					f_puts(configText,&fil);
			sprintf(configText, "HIGH SPEED MAP LIMIT: %f\n", highSpeed);
					f_puts(configText,&fil);
			sprintf(configText, "NUMBER OF STRIDES FOR MEAN WALKING SPEED: %i\n", numStrides);
					f_puts(configText,&fil);
			sprintf(configText, "MAPPING DIRECTION: %i\n", mapDir);
					f_puts(configText,&fil);
			sprintf(configText, "VSF STATE: %i\n", vsfState);
					f_puts(configText,&fil);
			sprintf(configText, "STIFFNESS SETTING: %i\n", stiffnessSetting);
					f_puts(configText,&fil);
			f_puts("Ax, Ay, Az, Gx, Gy, Gz, Walking Speed, Support Point Position\n", &fil);
			f_sync(&fil);


		}
	}  //END of f_mount

	sprintf(ss, "\r\nAfter 'if f_mount' conditional...\r\n");
		USART_puts(USART2, ss);

	if(runFlag_timerLoop){
	Config_TIM7_Timer(); }

	// Set up the classifiers
	initialize_classifier(classifiers);
	blah = 0;
	setRGB_B(0);
	setRGB_G(0);
	setRGB_R(0);
	while(1)
	{
		// save data in buffer to SD and flush when done
		saveSD_flush_Buffer();

		/*if (class == 1)
		{
			setRGB_B(255);
			setRGB_R(0);
			setRGB_G(0);
		}
		else if (class == 0)
		{
			setRGB_B(0);
			setRGB_R(0);
			setRGB_G(255);
		}
		else if (class == 2)
		{
			setRGB_B(0);
			setRGB_R(255);
			setRGB_G(0);
		}*/

		blah++;
	}
}  // end of main

void TIM7_IRQHandler(void)
{
  if (TIM7->SR & TIM_SR_UIF)
  {
	// In here is where the code you might want to execute should be.
  // It is in a conditional because it is possible to get into this IRQ from a DAC interrupt as well.

  //***** EDIT BELOW THIS LINE *****//
		// Update RTC structures for Date and Time
		RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
		RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
		subSec = RTC_GetSubSecond();
		/****/

		//***** IMU integration essentials *****//
		// Poll the sensors for new data
		Read_All_Sensors_toString(I2C2);
		readADC1_Channel_11();

		// integration
		class = imu_integrate(G,A,Q,V,P,&K,&count,classifiers,walkingSpeed);

		// estimate the stiffness from the fulcrum position
		estimatedStiffness = linearMap(ADC_CH11,softPosition,stiffPosition,absoluteMinStiffness,absoluteMaxStiffness);

		// control the foot using variable stiffness mode
		if (vsfState==1) {
			// get the mean walking speed of the last three strides
			meanWalkingSpeed = WalkingSpeedRollingMean(walkingSpeed);

			// drive the carriage
			desiredStiffness = MapSpeedtoStiffness(meanWalkingSpeed); // should be between 10 and 46
			desiredFulcrumPosition = MapStiffnesstoFulcrumPosition(desiredStiffness, minStiffness, maxStiffness); // should be between 200 and 2200

			vMag=norm(V);
			if (vMag > 0.4) { // Only drive the motor during swing
				DriveToFulcrumPosition(desiredFulcrumPosition);
			}
			else {
				setDutyCycle_MotorA(0);
			}
		// use a fixed stiffness
		} else	{
			desiredStiffness = setStaticStiffness(stiffnessSetting);
			desiredFulcrumPosition = MapStiffnesstoFulcrumPosition(desiredStiffness, minStiffness, maxStiffness);
			DriveToFulcrumPosition(desiredFulcrumPosition);
			vMag=norm(V);
			if (vMag > 0.4) { // Only drive the motor during swing
				DriveToFulcrumPosition(desiredFulcrumPosition);
			}
			else {
				setDutyCycle_MotorA(0);
			}
		}


		// create sensor data string
		// don't print too much data, it will significantly slow the loop (printing Ax,Ay,Az,Gx,Gy,Gz cuts the speed in half)
//		sprintf(dataString, "%f %i\n" , walkingSpeed[0], ADC_CH11);
		sprintf(dataString, "%f,%f,%f,%f,%f,%f,%f,%i,%f,%f,%i,%i,%f\n" , A[0], A[1], A[2], G[0], G[1], G[2], walkingSpeed[0], ADC_CH11, desiredStiffness, desiredFulcrumPosition, vsfState, stiffnessSetting, maxStiffness);


		loopCount+=1;
		if (loopCount==200) {
			//sprintf(serialString, "%i %i %i %0.2f %0.1f %0.1f %i\n" , vsfState, mapDir, stiffnessSetting, walkingSpeed[0], estimatedStiffness, maxStiffness, ADC_CH11); //use this line to see analog values from potentiometer
			sprintf(serialString, "%i %i %i %0.2f %0.1f %0.1f\n" , vsfState, mapDir, stiffnessSetting, walkingSpeed[0], estimatedStiffness, maxStiffness);
				USART_puts(USART2, serialString);
			loopCount=0;
		}
		if (strcmp((char*)received_string,(char*)control_string)) {
			strcpy(control_string, (char*) received_string);
			sprintf(rd, "%s\n", (char*)control_string);
				USART_puts(USART2, rd);
			parseControlString();
			//write2config();
		}

		// send sensor data string to buffer
		bufferdata( (char*) dataString, strlen(dataString));

		//
	//***** EDIT ABOVE THIS LINE *****//
  } // end of TIM status register IF conditional
  TIM7->SR = 0x0; // reset the status register.
}  // end of TIMER IRQ handler function

void USART2_IRQHandler(void){

	// check if the USART2 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE) ){
		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART2->DR & 0x1FF; // the character from the USART1 data register is saved in t

		// check if the received character is not the LF character (used to determine end of string)
		// or the if the maximum string length has been been reached
		//
		if( (t != '\n') && (cnt < MAX_STRLEN) ){
			received_string[cnt] = t;
			cnt++;

		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;
			//USART_puts(USART2, (char *)received_string);
		}

	}
}

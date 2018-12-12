/**
* @Library for ADXL345 3-axis accelometer 
* @Hardware dependencies: Could be changed very easily.
						STM32L152R uC
						SPI2 
						Some GPIOs
* @Author Iman Hosseinzadeh iman[dot]hosseinzadeh AT gmail
  https://github.com/ImanHz
						
*/
 /**
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
**/




#include "ADXL.h"


	float GAINX = 0.0f;
	float GAINY = 0.0f;
	float GAINZ = 0.0f;
	
/** Writing ADXL Registers. 
* @address: 8-bit address of register
* @value  : 8-bit value of corresponding register
* Since the register values to be written are 8-bit, there is no need to multiple writing
*/
static void writeRegister(uint8_t address,uint8_t value)
{
		if (address > 63)
		address = 63;
	
	// Setting R/W = 0, i.e.: Write Mode
    address &= ~(0x80);

	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&SPIhandler,&address,1,10);
	HAL_SPI_Transmit(&SPIhandler,&value,1,10);
	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_SET);
	

}


/** Reading ADXL Registers. 
* @address: 8-bit address of register
* @retval value  : array of 8-bit values of corresponding register
* @num		: number of bytes to be written
*/

static void readRegister(uint8_t address,uint8_t * value, uint8_t num)
{
		if (address > 63)
		address = 63;
		
		// Multiple Byte Read Settings
		if (num > 1)
		address |= 0x40;
		else	
		address &= ~(0x40);
		
		// Setting R/W = 1, i.e.: Read Mode
    address |= (0x80);		
		
	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&SPIhandler,&address,1,10);
	HAL_SPI_Receive(&SPIhandler,value,num,10);
	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_SET);
	
	
}


/**
Bandwidth Settings:
 Setting BW_RATE register
 BWRATE[4] = LOW POWER SETTING
 BWRATE[0-3] = DATA RATE i.e. 0110 for 6.25 Hz // See Table 6,7
 @param LPMode = 0 // Normal mode, Default
							 = 1 // Low Power Mode
 @param BW : Badwidth; See Tables 6 and 7
				
								NORMAL MODE
				BW value 	|  Output Data Rate (Hz)
				---------------------------------
						6 		|  				6.25 // Default		
						7 		|  				12.5		
						8 		|  				25
						9 		|  				50		
						10 		|  				100
						11 		|  				200
						12 		|  				400
						13 		|  				800
						14 		|  				1600
						15 		|  				3200
								
								
								LOWPOWER MODE
				BW value 	|  Output Data Rate (Hz)
				---------------------------------
						7 		|  				12.5	// Default
						8 		|  				25
						9 		|  				50		
						10 		|  				100
						11 		|  				200
						12 		|  				400
			*/
static void adxlBW(ADXL_InitTypeDef * adxl)
		{
		uint8_t bwreg=0;
		writeRegister(BW_RATE,bwreg);
		if (adxl->LPMode == LPMODE_LOWPOWER) 
						{
						// Low power mode
						bwreg |= (1 << 4);
						if ( ((adxl->Rate) <7) && ((adxl->Rate)>12) ) bwreg += 7;
								else bwreg +=(adxl->Rate);
						writeRegister(BW_RATE,bwreg);	
						} 
		else
				{
				// Normal Mode
	
				if ( ((adxl->Rate) <6) && ((adxl->Rate)>15) ) bwreg += 6;
						else bwreg +=(adxl->Rate);
				writeRegister(BW_RATE,bwreg);	
				}
		}

	
/**
	Data Format Settings
	DATA_FORMAT[7-0] = SELF_TEST  SPI  INT_INVERT  0  FULL_RES  Justify  Range[2]
	
	SPI bit: 			0 = 4-wire (Default) 
								1 = 3-wire
	INT_Invert:   		0 = Active High (Default) 
								1 = Active Low
	Full Res:			0 = 10-bit (Default) 
								1 = Full Resolution
	Justify:			0 = Signed (Default) 
								1 = MSB
	Range:
								
					 

				Range value 	|  Output Data Rate (Hz)
				---------------------------------
						0 		|  				+-2g	// Default
						1 		|  				+-4g
						2 		|  				+-8g		
						3 		|  				+-16g
	 									
		*/

static void adxlFormat(ADXL_InitTypeDef * adxl)
			{
			uint8_t formatreg=0;
			writeRegister(DATA_FORMAT,formatreg);
			formatreg = (adxl->SPIMode << 6) | (adxl->IntMode << 5) | (adxl->Justify << 2) | (adxl->Resolution << 3);
			formatreg += (adxl -> Range);
			writeRegister(DATA_FORMAT,formatreg);
			}

// Public Functions

// Initializes the ADXL unit
adxlStatus ADXL_Init(ADXL_InitTypeDef * adxl)
{
	// CS is active low. Here we deselect the chip. In each function the CS signal is asserted individually
	HAL_GPIO_WritePin(ADXLCS_GPIO_Port,ADXLCS_Pin,GPIO_PIN_SET);
	// Unknown delay should apply
	HAL_Delay(5);
	uint8_t testval = 0;
	// The Device Address register is constant, i.e. = 0xE5
	readRegister(DEVID,&testval,1);
	if (testval != 0xE5) return ADXL_ERR;
	// Init. of BW_RATE and DATAFORMAT registers
	adxlBW(adxl);
	adxlFormat(adxl);
	
	// Settings gains 
	if (adxl->Resolution == RESOLUTION_10BIT)
			{
			switch (adxl->Range) {
							case RANGE_2G:
								GAINX = GAINY = GAINZ = 1/255.0f;
								break;
							case RANGE_4G:
								GAINX = GAINY = GAINZ = 1/127.0f;
								break;
							case RANGE_8G:
								GAINX = GAINY = GAINZ = 1/63.0f;
								break;
							case RANGE_16G:
								GAINX = GAINY = GAINZ = 1/31.0f;
								break;
								}
			} else 
			{
			GAINX = GAINY = GAINZ = 1/255.0f;
			}
	// Setting AutoSleep and Link bits
			uint8_t reg;
			readRegister(POWER_CTL,&reg,1);
			if ( (adxl->AutoSleep) == AUTOSLEEPON) reg |= (1 << 4); else reg &= ~(1 << 4);
			if ( (adxl->LinkMode) == LINKMODEON) reg |= (1 << 5); else reg &= ~(1 << 5);
			writeRegister(POWER_CTL,reg);
			
	return ADXL_OK;
	
}


/** Reading Data
* @retval : data				: array of accel. 
						outputType	: OUTPUT_SIGNED: signed int
													OUTPUT_FLOAT: float
						if output is float, the GAIN(X-Y-Z) should be defined in definitions.
* @usage :	Depending on your desired output, define an array of type uint16_t or float with 3 cells:
						uint16_t acc[3];
						ADXL_getAccel(acc,OUTPUT_SIGNED);
						and so on...
*/
void ADXL_getAccel(void *Data , uint8_t outputType)
	{
	uint8_t data[6]={0,0,0,0,0,0};	
	readRegister(DATA0,data,6);
	
	
	if (outputType == OUTPUT_SIGNED)
		{
		int16_t * acc = Data;	
	  // Two's Complement
	  acc[0] = (int16_t) ((data[1]*256+data[0]));
	  acc[1] = (int16_t) ((data[3]*256+data[2]));
	  acc[2] = (int16_t) ((data[5]*256+data[4]));
	  }
	else if (outputType == OUTPUT_FLOAT)
						{
						float * fdata = Data;
						fdata[0] = ( (int16_t) ((data[1]*256+data[0])))*GAINX;
						fdata[1] = ( (int16_t) ((data[3]*256+data[2])))*GAINY;
						fdata[2] = ( (int16_t) ((data[5]*256+data[4])))*GAINZ;
						
						}
	}
	
	
/** Starts Measure Mode
* @param: s = ON or OFF				

*/
void ADXL_Measure(Switch s)
		{
			uint8_t reg;
			readRegister(POWER_CTL,&reg,1);
			switch (s) {
				case ON: 
				reg &= ~(1<<2);
				reg |= (1<<3);
				writeRegister(POWER_CTL,reg);
				break;
				case OFF:
				reg &= ~(1<<3);
				writeRegister(POWER_CTL,reg);
				break;				
				}
		}

/** Starts Sleep Mode
* @param: s 		=  ON or OFF				 				
* @param: rate  =  SLEEP_RATE_1HZ
									 SLEEP_RATE_2HZ
									 SLEEP_RATE_4HZ
									 SLEEP_RATE_8HZ
*/
void ADXL_Sleep(Switch s,uint8_t rate)
		{
			uint8_t reg;
			readRegister(POWER_CTL,&reg,1);
			switch (s) {
				case ON: 
				reg |= (1<<2);
				reg &= ~(1<<3);
				reg += rate;
				writeRegister(POWER_CTL,reg);
				break;
				case OFF:
				reg &= ~(1<<2);
				writeRegister(POWER_CTL,reg);
				break;				
				}
			
		}

/** Starts Standby Mode
* @param: s = ON or OFF		
		OFF: Takes the module into sleep mode.

*/
void ADXL_Standby(Switch s)
		{
			uint8_t reg;
			readRegister(POWER_CTL,&reg,1);
			switch (s) {
				case ON: 
				reg &= ~(1<<2);
				reg &= ~(1<<3);
				writeRegister(POWER_CTL,reg);
				break;
				case OFF:
				reg |= (1<<2);
				writeRegister(POWER_CTL,reg);
				break;				
				}
			
		}

	
	
/** Reading Main Registers
regs[0] = BW_RATE
regs[1] = DATA_FORMAT
regs[2] = POWER_CTL
*/
void ADXL_test(uint8_t * regs)
		{
			readRegister(BW_RATE,&regs[0],1);
			readRegister(DATA_FORMAT,&regs[1],1);
			readRegister(POWER_CTL,&regs[2],1);
			
		}
		
		 /**
 Enables the self Test mode
 */
void ADXL_enableSelfTest(void)
			{
			uint8_t formatreg=0;
			writeRegister(DATA_FORMAT,formatreg);
			formatreg |= (1<<7);	
			writeRegister(DATA_FORMAT,formatreg);
			}


 /**
 Disables the self Test mode
 */
void ADXL_disableSelfTest(void)
			{
			uint8_t formatreg=0;
			writeRegister(DATA_FORMAT,formatreg);
			formatreg &= ~(1<<7);	
			writeRegister(DATA_FORMAT,formatreg);
				}
			
				
/**
 Setting the offsets for calibration
* @param 	user-set offset adjustments in twos complement format
					with a scale factor of 15.6 mg/LSB (that is, 0x7F = +2 g).
				
*/				
void ADXL_SetOffset(int8_t off_x,int8_t off_y,int8_t off_z)
			{
			writeRegister(OFFX,(uint8_t) off_x );
			writeRegister(OFFY,(uint8_t) off_y );
			writeRegister(OFFZ,(uint8_t) off_z );
			}




				//////////////////////////////////////////// I N T E R R U P T S //////////////////////
	
/** Setting TAP Int.
* @param out : ADXL has two Int. pins.
* @param axes: The axes of tap. Could be OR'ed.
* @param Duration: The minimum duration for tap detection. The scale factor is 625 us/LSB. Should not be 0!
* @param Threshold: The threshold value for tap interrupt. The scale factor is 62.5 mg/LSB. Should not be 0!
*/

void ADXL_enableSingleTap(ADXL_IntOutput out, uint8_t axes, uint8_t Duration, uint8_t Threshold)
	
{
	uint8_t reg=0;
	
	writeRegister(DUR,Duration);
	writeRegister(THRESH_TAP,Threshold);
	
	//Setting the Axes
	readRegister(TAP_AXES,&reg,1);
	reg |= axes;
	
	writeRegister(TAP_AXES,reg);
	
	// Settings Int output
	readRegister(INT_MAP,&reg,1);
	if (out == INT1) reg &= ~(1<<6); else reg |= (1<<6);
	writeRegister(INT_MAP,reg);
	
	// Enabling the TAP interrupt
	readRegister(INT_ENABLE,&reg,1);
	reg |= (1<<6);
	writeRegister(INT_ENABLE,reg);
	
}
	
/** Disabling TAP Int.

 The settings are preserved.

*/

void ADXL_disableSingleTap(void)
	
{
	uint8_t reg=0;
	// Disabling the TAP interrupt
	readRegister(INT_ENABLE,&reg,1);
	reg &= ~(1<<6);
	writeRegister(INT_ENABLE,reg);
	
	
}

/**  Enabling Double TAP Int.
* @param out : 			ADXL has two Int. pins.
* @param axes: 			The axes of tap. Could be OR'ed.
* @param Duration: 	The minimum duration for tap detection. The scale factor is 625 us/LSB. Should not be 0!
* @param Threshold: The threshold value for tap interrupt. The scale factor is 62.5 mg/LSB. Should not be 0!
* @param Latent: 		The delay time after the first Tap. Scale factor is : 1.25 ms/LSB. Should not be 0!
* @param Windows:		The time interval between two Taps. Scale factor is : 1.25 ms/LSB.  Should not be 0!
*/

void ADXL_enableDoubleTap(ADXL_IntOutput out, uint8_t axes, uint8_t Duration, uint8_t Threshold, uint8_t Latent, uint8_t Window)
		{
		uint8_t reg=0;
	
	writeRegister(DUR,Duration);
	writeRegister(THRESH_TAP,Threshold);
	writeRegister(LATENT,Latent);
	writeRegister(WINDOW,Window);
	
				
	//Setting the Axes
	readRegister(TAP_AXES,&reg,1);
	reg += axes;
	writeRegister(TAP_AXES,reg);
	
	// Settings Int output
	readRegister(INT_MAP,&reg,1);
	if (out == INT1) reg &= ~(1<<5); else reg |= (1<<5);
	writeRegister(INT_MAP,reg);
	
	// Enabling the TAP interrupt
	readRegister(INT_ENABLE,&reg,1);
	reg |= (1<<5);
	writeRegister(INT_ENABLE,reg);
	 }


/** Disabling Double TAP Int.

 The settings are preserved.

*/

void ADXL_disableDoubleTap(void)
	
  {
	uint8_t reg=0;
	// Disabling the Double TAP interrupt
	readRegister(INT_ENABLE,&reg,1);
	reg &= ~(1<<5);
	writeRegister(INT_ENABLE,reg);
	}

/**  Enabling Activity Int.
* @param out : 			ADXL has two Int. pins.
* @param axes: 			The axes of activity. Could be OR'ed.
* @param Threshold: The threshold value for activity interrupt. The scale factor is 62.5 mg/LSB. Should not be 0!
*/

void ADXL_enableActivity(ADXL_IntOutput out, uint8_t axes, uint8_t Threshold, uint8_t AcDc)
	
		{
			uint8_t reg=0;
	
	writeRegister(THRESH_ACT,Threshold);

				
	//Setting the Axes
	readRegister(ACT_INACT_CTL,&reg,1);
	reg += (axes << 4);
	if (AcDc == ACTIVITY_AC) reg |= (1<<7); else reg &= ~(1<<7); 
			writeRegister(TAP_AXES,reg);
	
	// Settings Int output
	readRegister(INT_MAP,&reg,1);
	if (out == INT1) reg &= ~(1<<4); else reg |= (1<<4);
	writeRegister(INT_MAP,reg);
	
	// Enabling the TAP interrupt
	readRegister(INT_ENABLE,&reg,1);
	reg |= (1<<4);
	writeRegister(INT_ENABLE,reg);
			
			
		}


/** Disabling Double TAP Int.

 The settings are preserved.

*/

void ADXL_disableActivity(void)
  {
	uint8_t reg=0;
	// Disabling the Double TAP interrupt
	readRegister(INT_ENABLE,&reg,1);
	reg &= ~(1<<4);
	writeRegister(INT_ENABLE,reg);
	}

/**  Enables FreeFall Int.
* @param out : 			ADXL has two Int. pins.
* @param time: 			Representing the minimum time that the RSS value of all axes
										must be less than Threshold to generate a free-fall interrupt.
										A value of 0 may result in undesirable
										behavior if the free-fall interrupt is enabled. Values between 100 ms
										and 350 ms (0x14 to 0x46) are recommended
* @param Threshold: The root-sumsquare (RSS) value of all axes is calculated and compared with
										the value in Threshold to determine if a free-fall event occurred.
										The scale factor is 62.5 mg/LSB. Note that a value of 0 mg may
										result in undesirable behavior if the free-fall interrupt is enabled.
										Values between 300 mg and 600 mg (0x05 to 0x09) are
										recommended.
					*/

void ADXL_enableFreeFall(ADXL_IntOutput out, uint8_t Threshold, uint8_t Time)
			{
			uint8_t reg=0;
			writeRegister(TIME_FF,Time);
			writeRegister(THRESH_FF,Threshold);
			// Settings Int output
			readRegister(INT_MAP,&reg,1);
			if (out == INT1) reg &= ~(1<<2); else reg |= (1<<2);
			writeRegister(INT_MAP,reg);
	
			readRegister(INT_ENABLE,&reg,1);
			reg |= (1<<2);
			writeRegister(INT_ENABLE,reg);
			}


/** Disabling Double TAP Int.

 The settings are preserved.

*/

void ADXL_disableFreeFall(void)
{
			uint8_t reg=0;
			readRegister(INT_ENABLE,&reg,1);
			reg %= ~(1<<2);
			writeRegister(INT_ENABLE,reg);
	
}

/** Interrupt prototype
* @brief In order to interrupt flags being reset, the address 0x30 should be read.
* Put this function wherever you want to implement interrupt routines, e.g. EXTI_Callback
*/

void ADXL_IntProto(void)
	
{
			uint8_t reg=0;
			readRegister(INT_SOURCE,&reg,1);
	
}


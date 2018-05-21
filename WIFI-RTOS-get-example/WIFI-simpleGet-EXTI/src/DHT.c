#include "DHT.h"
#include "asf.h"

//----- Auxiliary data ----------//
enum DHT_Status_t __DHT_STATUS;

#define __DHT_Temperature_Min	-40
#define __DHT_Temperature_Max	80
#define __DHT_Humidity_Min		0
#define __DHT_Humidity_Max		100
#define __DHT_Delay_Read		20

//General use bit manipulating commands
#define BitSet(		x, y)			(	x |=	 (1UL<<y)			)
#define BitClear(	x, y)			(	x &=	(~(1UL<<y))			)
#define BitToggle(	x, y)			(	x ^=	 (1UL<<y)			)
#define BitCheck(	x, y)			(	x &		 (1UL<<y)	? 1 : 0	)

#define DHT_PIO PIOC
#define DHT_PIO_ID 12
#define DHT_PIO_PIN 13
#define DHT_PIO_PIN_MASK (1 << DHT_PIO_PIN)
//-------------------------------//

//----- Prototypes ----------------------------//
static double DataToTemp(uint8_t Data3, uint8_t Data4);
static double DataToHum(uint8_t Data1, uint8_t Data2);
//---------------------------------------------//

//----- Functions -----------------------------//
//Setup sensor.
void DHT_Setup(void)
{
	delay_ms(__DHT_Delay_Setup);
	__DHT_STATUS = DHT_Ok;
}

//Get sensor status.
enum DHT_Status_t DHT_status(void)
{
	return (__DHT_STATUS);
}

//Read raw buffer from sensor.
void DHT_ReadRaw(uint8_t Data[4])
{
	uint8_t buffer[5] = {0, 0, 0, 0, 0};
	uint8_t retries, i;
	int8_t j;
	__DHT_STATUS = DHT_Ok;
	retries = i = j = 0;

	//----- Step 1 - Start communication -----
	if (__DHT_STATUS == DHT_Ok)
	{
		//Request data
		//#define DHT_Pin		A, 7
		pio_set_output(DHT_PIO, DHT_PIO_PIN_MASK, 1, 0, 0);
		pio_clear(DHT_PIO, DHT_PIO_PIN_MASK);
		//DigitalWrite(DHT_Pin, Low);			//DHT_PIN = 0
		//PinMode(DHT_Pin, Output);			//DHT_PIN = Output
		
		delay_ms(__DHT_Delay_Read);
		
		//Setup DHT_PIN as input with pull-up resistor so as to read data
		pio_set(DHT_PIO, DHT_PIO_PIN_MASK);
		//DigitalWrite(DHT_Pin, High);		//DHT_PIN = 1 (Pull-up resistor)
		//PinMode(DHT_Pin, Input);			//DHT_PIN = Input
		pio_set_input(DHT_PIO, DHT_PIO_PIN_MASK, PIO_PULLUP);

		//Wait for response for 20-40us
		retries = 0;
		//while (DigitalRead(DHT_Pin))
		while (pio_get(DHT_PIO, PIO_INPUT, DHT_PIO_PIN_MASK))
		{
			delay_us(2);
			retries += 2;
			if (retries > 60)
			{
				__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
				break;
			}
		}
	}
	//----------------------------------------

	//----- Step 2 - Wait for response -----
	if (__DHT_STATUS == DHT_Ok)
	{
		//Response sequence began
		//Wait for the first response to finish (low for ~80us)
		retries = 0;
		//while (!DigitalRead(DHT_Pin))
		while (!pio_get(DHT_PIO, PIO_INPUT, DHT_PIO_PIN_MASK))
		{
			delay_us(2);
			retries += 2;
			if (retries > 100)
			{
				__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
				break;
			}
		}
		//Wait for the last response to finish (high for ~80us)
		retries = 0;
		//while(DigitalRead(DHT_Pin))
		while(pio_get(DHT_PIO, PIO_INPUT, DHT_PIO_PIN_MASK))
		{
			delay_us(2);
			retries += 2;
			if (retries > 100)
			{
				__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
				break;
			}
		}
	}
	//--------------------------------------

	//----- Step 3 - Data transmission -----
	if (__DHT_STATUS == DHT_Ok)
	{
		//Reading 5 bytes, bit by bit
		for (i = 0 ; i < 5 ; i++)
		for (j = 7 ; j >= 0 ; j--)
		{
			//There is always a leading low level of 50 us
			retries = 0;
			//while(!DigitalRead(DHT_Pin))
			while(!pio_get(DHT_PIO, PIO_INPUT, DHT_PIO_PIN_MASK))
			{
				delay_us(2);
				retries += 2;
				if (retries > 70)
				{
					__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
					j = -1;								//Break inner for-loop
					i = 5;								//Break outer for-loop
					break;								//Break while loop
				}
			}

			if (__DHT_STATUS == DHT_Ok)
			{
				//We read data bit || 26-28us means '0' || 70us means '1'
				delay_us(35);
				//if (DigitalRead(DHT_Pin))				//If HIGH
				if (pio_get(DHT_PIO, PIO_INPUT, DHT_PIO_PIN_MASK))
				//BitSet definido nas Macros
				BitSet(buffer[i], j);				//bit = '1'

				retries = 0;
				//while(DigitalRead(DHT_Pin))
				while(pio_get(DHT_PIO, PIO_INPUT, DHT_PIO_PIN_MASK))
				{
					delay_us(2);
					retries += 2;
					if (retries > 100)
					{
						__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
						break;
					}
				}
			}
		}
	}
	//--------------------------------------


	//----- Step 4 - Check checksum and return data -----
	if (__DHT_STATUS == DHT_Ok)
	{
		if (((uint8_t)(buffer[0] + buffer[1] + buffer[2] + buffer[3])) != buffer[4])
		{
			__DHT_STATUS = DHT_Error_Checksum;	//Checksum error
		}
		else
		{
			//Build returning array
			//data[0] = Humidity		(int)
			//data[1] = Humidity		(dec)
			//data[2] = Temperature		(int)
			//data[3] = Temperature		(dec)
			//data[4] = Checksum
			for (i = 0 ; i < 4 ; i++)
			Data[i] = buffer[i];
		}
	}
	//---------------------------------------------------
}

//Read temperature in Celsius.
void DHT_ReadTemperature(double *Temperature)
{
	double waste[1];
	DHT_Read(Temperature, waste);
}

//Read humidity percentage.
void DHT_ReadHumidity(double *Humidity)
{
	double waste[1];
	DHT_Read(waste, Humidity);
}

//Read temperature and humidity.
void DHT_Read(double *Temperature, double *Humidity)
{
	uint8_t data[4] = {0, 0, 0, 0};

	//Read data
	DHT_ReadRaw(data);
	
	//If read successfully
	if (__DHT_STATUS == DHT_Ok)
	{
		//Calculate values
		*Temperature = DataToTemp(data[2], data[3]);
		*Humidity = DataToHum(data[0], data[1]);
		
		//Check values
		if ((*Temperature < __DHT_Temperature_Min) || (*Temperature > __DHT_Temperature_Max))
		__DHT_STATUS = DHT_Error_Temperature;
		else if ((*Humidity < __DHT_Humidity_Min) || (*Humidity > __DHT_Humidity_Max))
		__DHT_STATUS = DHT_Error_Humidity;
	}
}

//Convert temperature from Celsius to Fahrenheit.
double DHT_ConvertToFahrenheit(double Temperature)
{
	return (Temperature * 1.8 + 32);
}

//Convert temperature from Celsius to Kelvin.
double DHT_ConvertToKelvin(double Temperature)
{
	return (Temperature + 273.15);
}

//Convert temperature data to double temperature.
static double DataToTemp(uint8_t Data2, uint8_t Data3)
{
	double temp = 0.0;
	//BitCheck está no MacroIO.h
	temp = (BitCheck(Data2, 7) ? ((((Data2 & 0x7F) << 8) | Data3) / (-10.0)) : (((Data2 << 8) | Data3) / 10.0));
	
	return temp;
}

static double DataToHum(uint8_t Data0, uint8_t Data1)
{
	double hum = 0.0;
	hum = ((Data0<<8) | Data1) / 10.0;
	return hum;
}
//---------------------------------------------//

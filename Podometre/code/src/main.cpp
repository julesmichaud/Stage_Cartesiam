/* Includes ------------------------------------------------------------------*/
#include "mbed.h"
#include "bmi160.h"
#ifndef DATA_LOGGING
#include "NanoEdgeAI.h"
#endif
#include <math.h>

/* Defines -------------------------------------------------------------------*/
#if !defined(DATA_LOGGING) && !defined(NEAI_LIB)
#define DATA_LOGGING
#endif
#ifdef DATA_LOGGING
#define DATA_INPUT_USER 128
#define AXIS_NUMBER 3
#define LOG_NUMBER 50
#else
#define LEARNING_NUMBER 20 /* Number of learning signals */
#endif

/* Objects -------------------------------------------------------------------*/
Serial pc(USBTX, USBRX);
Serial bt(D5, D4, 9600);
Ticker toggle_led_ticker;
DigitalOut myled(LED2);
I2C i2c(D0, D1);
BMI160_I2C imu(i2c, BMI160_I2C::I2C_ADRS_SDO_HI);
BMI160::AccConfig accConfig;
BMI160::SensorData accData;

/* Variables -----------------------------------------------------------------*/
float acc_x = 0.F;
float acc_y = 0.F;
float acc_z = 0.F;
float last_acc_x = 0.F;
float last_acc_y = 0.F;
float last_acc_z = 0.F;
#ifndef DATA_LOGGING
uint8_t similarity = 0;
uint16_t learn_cpt = 0;
#endif

/* Buffer with accelerometer values for x-, y- and z-axis --------------------*/
float acc_buffer[DATA_INPUT_USER * AXIS_NUMBER] = {0.F};


/* Functions prototypes ------------------------------------------------------*/
#ifdef DATA_LOGGING
void data_logging_mode(void);
#endif
#ifdef NEAI_LIB
void neai_library_test_mode(void);
#endif
void init(void);
void init_bmi160(void);
void toggle_led(void);
void fill_acc_buffer(void);
void fill_acc_buffer_print(void);
void get_acc_values(void);

/* BEGIN CODE-----------------------------------------------------------------*/

int main()
{
	/* Initialization */
	init();
	
	#ifdef DATA_LOGGING
		/* Data logging mode */
		/* Compiler flag: -DDATA_LOGGING */
		data_logging_mode();
	#endif

	#ifdef NEAI_LIB
		/* NanoEdge AI Library test mode */
		/* Compiler flag -DNEAI_LIB */
		neai_library_test_mode();
	#endif
}

/* Functions definition ------------------------------------------------------*/

#ifdef DATA_LOGGING

void data_logging_mode()
{
	float start = 0;
	while(1) 
	{
		get_acc_values();
		start = fabs(acc_x)+fabs(acc_y)+fabs(acc_z);

		/* Waiting for the logging process start */
		if (start >= 2.0)
		{
			/* Blink LED  during logging process */
			toggle_led_ticker.attach(&toggle_led, 0.1);
			
			/* Logging process */
			fill_acc_buffer();

			/* Stop blink LED (end of logging process) */
			toggle_led_ticker.detach();
			myled = 0;

			/* Print logging process results */
			fill_acc_buffer_print();

			wait_ms(1000);
		}

		
	}
}

#endif

#ifdef NEAI_LIB

void neai_library_test_mode()
{
	float start = 0;
	/* Learning process */
	while (learn_cpt < LEARNING_NUMBER)
	{
		get_acc_values();
		start = fabs(acc_x)+fabs(acc_y)+fabs(acc_z);

		/* Waiting for the logging process start */
		if (start >= 4.0)
		{
			/* Blink LED  during logging process */
			toggle_led_ticker.attach(&toggle_led, 0.1);
			
			/* Logging process */
			fill_acc_buffer();
			NanoEdgeAI_learn(acc_buffer);
			pc.printf("%d percent \n", (int)(learn_cpt * 100) / LEARNING_NUMBER);
			bt.printf("%d percent \n", (int)(learn_cpt * 100) / LEARNING_NUMBER);
			learn_cpt ++;

			wait_ms(1000);

			/* Stop blink LED (end of logging process) */
			toggle_led_ticker.detach();
			myled = 0;
		}
	}
		
	/* Detection process */
	/* LED off: no movement */
	/* LED on: one more point */

	int points = 0;

	while(1)
	{
		get_acc_values();
		start = fabs(acc_x)+fabs(acc_y)+fabs(acc_z);
		if (start >= 3.0)
		{
			fill_acc_buffer();
			similarity = NanoEdgeAI_detect(acc_buffer);
			// pc.printf("Similarity : %d\n", similarity);
			// bt.printf("Similarity : %d\n", similarity);
			if (similarity > 90)
			{
				myled = 1;
				points ++;
				pc.printf("Points : %d\n", points);
				bt.printf("Points : %d\n", points);
				wait_ms(2000);
				myled = 0;
			}
			if (points >= 10)
			{
				pc.printf("End of the game\n");
				bt.printf("End of the game\n");
				points = 0;
			}

		}
	}
}
#endif

void init()
{
	pc.baud(115200);
	init_bmi160();
	#ifdef NEAI_LIB
		NanoEdgeAI_initialize();
	#endif
}

void init_bmi160()
{
	imu.setSensorPowerMode(BMI160::ACC, BMI160::NORMAL);
	wait_ms(10);
	accConfig.range = BMI160::SENS_2G; /* Accelerometer range +-2G */
	accConfig.us = BMI160::ACC_US_OFF;
	accConfig.bwp = BMI160::ACC_BWP_2;
	accConfig.odr = BMI160::ACC_ODR_12; /* Accelerometer output data rate < 800Hz */
	imu.setSensorConfig(accConfig);
	wait_ms(100);
}


void toggle_led()
{
	
	myled = !myled;
}

void fill_acc_buffer()
{
	for (uint16_t i = 0; i < DATA_INPUT_USER; i++)
	{
		get_acc_values();
		acc_buffer[AXIS_NUMBER * i] = acc_x;
		acc_buffer[AXIS_NUMBER * i + 1] = acc_y;
		acc_buffer[AXIS_NUMBER * i + 2] = acc_z;
	}
}

void fill_acc_buffer_print()
{
	for (uint16_t isample = 0; isample < AXIS_NUMBER * DATA_INPUT_USER - 1; isample++)
	{
		pc.printf("%.4f ", acc_buffer[isample]);
		bt.printf("%.4f ", acc_buffer[isample]);
	}
	pc.printf("%.4f\n", acc_buffer[AXIS_NUMBER * DATA_INPUT_USER - 1]);
	bt.printf("%.4f\n", acc_buffer[AXIS_NUMBER * DATA_INPUT_USER - 1]);
}

void get_acc_values()
{
	/* Polling method to get a complete buffer */
	do {
		imu.getSensorXYZ(accData, accConfig.range);
		acc_x = accData.xAxis.scaled;
		acc_y = accData.yAxis.scaled;
		acc_z = accData.zAxis.scaled;
	}
	while (acc_x == last_acc_x && acc_y == last_acc_y && acc_z == last_acc_z);
	
	last_acc_x = acc_x;
	last_acc_y = acc_y;
	last_acc_z = acc_z;
}

/* END CODE-------------------------------------------------------------------*/
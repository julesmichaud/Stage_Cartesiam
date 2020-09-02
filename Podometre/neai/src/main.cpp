/**
*******************************************************************************
* @file   main.cpp
* @brief  Main program body
*******************************************************************************
* Main program for collecting data and testing NanoEdge AI solution
*
* Compiler Flags
* -DDATA_LOGGING : data logging mode for collecting data
* -DNEAI_LIB     : test mode with NanoEdge AI Library
*
* @note   if no compiler flag then data logging mode by default
*******************************************************************************
*/

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
#define DATA_INPUT_USER 256
#define AXIS_NUMBER 3
#define LOG_NUMBER 50
#else
#define LEARNING_NUMBER 70 /* Number of learning signals */
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
float acc_buffer_x[DATA_INPUT_USER] = {0.F};
float acc_buffer_y[DATA_INPUT_USER] = {0.F};
float acc_buffer_z[DATA_INPUT_USER] = {0.F};


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
void get_acc_values(void);
void fill_acc_buffer_2(void);
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
	while(1) {
		/* Make a little pressure to start the learning process */
		/* Blink LED: running logging process */		
			
		/* Waiting for a little preasure*/
		int start = 0;
		while (not start){
			get_acc_values();
			if (fabs(acc_x)+fabs(acc_y)+fabs(acc_z) >= 4.0) {
				start = 1;
				wait_ms(3000);
			}
		}

		/* Blink LED  during logging process */
		toggle_led_ticker.attach(&toggle_led, 0.1);

		/* Wait one seconds before launching logging process */
		wait_ms(1000);

		/* Logging process */
			
		for (uint8_t ilog = 0; ilog < LOG_NUMBER; ilog++) {
			fill_acc_buffer();
		}

		/* Stop blink LED (end of logging process) */
		toggle_led_ticker.detach();
		myled = 0;
		
	}
}
#endif

#ifdef NEAI_LIB

void neai_library_test_mode()
{
	/* Learning process */
	/* Make a little pressure to start the learning process */

	/* Waiting for a little preasure*/
	while (learn_cpt < LEARNING_NUMBER) {

		int start = 0;
		while (not start){
			get_acc_values();
			if (fabs(acc_x)+fabs(acc_y)+fabs(acc_z) >= 4.0) {
				start = 1;
				wait_ms(3000);
			}
		}

		/* Blink LED  during logging process */
		toggle_led_ticker.attach(&toggle_led, 0.1);

		/* Wait one seconds before starting learning process */
		wait_ms(1000);

		/* Learning process */
		for (uint16_t i = 0; i < LEARNING_NUMBER; i++) {
			fill_acc_buffer_2();
			NanoEdgeAI_learn(acc_buffer);
			pc.printf("%d percent \n", (int)(learn_cpt * 100) / LEARNING_NUMBER);
			bt.printf("%d percent \n", (int)(learn_cpt * 100) / LEARNING_NUMBER);
			learn_cpt++;
		
		}

		/* Stop blink LED (end of logging process) */
		toggle_led_ticker.detach();
		myled = 0;
	}
		
	/* Detection process */
	/* LED off: no movement */
	/* LED on: one more step */
	int nb_pas = 0;
	int first = 1;
	float threshold;
	float threshold_2;
	char var;
	while(1) {
		myled = 0;
		fill_acc_buffer_2();
		similarity = NanoEdgeAI_detect(acc_buffer);
		if (similarity >= 90 && first) {
			bt.printf("MARCHE_0\n");
			float max_x = -100.0; float max_y = -100.0; float max_z = -100.0;
			float min_x = 100.0; float min_y = 100.0; float min_z = 100.0;
			for (uint16_t i = 0; i < DATA_INPUT_USER; i++) {
				//bt.printf("%d\n", i);
				if (acc_buffer_x[i] > max_x) {max_x = acc_buffer_x[i];}
				else if (acc_buffer_x[i] < min_x) {min_x = acc_buffer_x[i];}
				if (acc_buffer_y[i] > max_y) {max_y = acc_buffer_y[i];}
				else if (acc_buffer_y[i] < min_y) {min_y = acc_buffer_y[i];}
				if (acc_buffer_z[i] > max_z) {max_z = acc_buffer_z[i];}
				else if (acc_buffer_z[i] < min_z) {min_z = acc_buffer_z[i];}
			}
			//bt.printf("Milieu\n");
			if (fabs(max_x - min_x) >= fabs(max_y - min_y) && fabs(max_x - min_x) >= fabs(max_z - min_z)) {threshold = (max_x + min_x)/2 ;var ='x';}
			else if (fabs(max_y - min_y) >= fabs(max_x - min_x) && fabs(max_y - min_y) >= fabs(max_z - min_z)) {threshold = (max_y + min_y)/2;var='y';}
			else {threshold = (max_z + min_z)/2;var='z';}
			first = 0;
			// nb_pas ++;
			myled = 1;
			pc.printf("Steps : %d\n", nb_pas);
			bt.printf("Steps : %d\n", nb_pas);
			//bt.printf("End\n");
			//wait_ms(300);

		} 
		else if (similarity >= 90){
			bt.printf("MARCHE_1\n");
			float max_x = -100.0; float max_y = -100.0; float max_z = -100.0;
			float min_x = 100.0; float min_y = 100.0; float min_z = 100.0;
			if (var == 'x') {
				for (uint16_t i = 0; i < DATA_INPUT_USER ; i++) {
					if (acc_buffer_x[i] > max_x) {max_x = acc_buffer_x[i];}
					else if (acc_buffer_x[i] < min_x) {min_x = acc_buffer_x[i];}
				}
				threshold_2 = (max_x+min_x)/2;
			}
			else if (var == 'y') {
				for (uint16_t i = 0; i < DATA_INPUT_USER; i++) {
					if (acc_buffer_y[i] > max_y) {max_y = acc_buffer_y[i];}
					else if (acc_buffer_y[i] < min_y) {min_y = acc_buffer_y[i];}
				}
				threshold_2 = (max_y+min_y)/2;
			}
			else {
				for (uint16_t i = 0; i < DATA_INPUT_USER; i++) {
					if (acc_buffer_z[i] > max_z) {max_z = acc_buffer_z[i];}
					else if (acc_buffer_z[i] < min_z) {min_z = acc_buffer_z[i];}
				}
				threshold_2 = (max_z+min_z)/2;
			}
			if (threshold_2 > 1.1*threshold){
				nb_pas++;
				pc.printf("Steps : %d\n", nb_pas);
				bt.printf("Steps : %d\n", nb_pas);
				myled = 1;
				//wait_ms(300);
			}
			threshold = threshold_2;
		}
		else {
			bt.printf("MARCHE PAS\n");
			first = 1;
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
	accConfig.odr = BMI160::ACC_ODR_11; /* Accelerometer output data rate < 800Hz */
	imu.setSensorConfig(accConfig);
	wait_ms(100);
}


void toggle_led(void)
{
	myled = !myled;
}


void fill_acc_buffer()
{
	for (uint16_t i = 0; i < DATA_INPUT_USER; i++) {
		get_acc_values();
		acc_buffer[AXIS_NUMBER * i] = acc_x;
		acc_buffer[AXIS_NUMBER * i + 1] = acc_y;
		acc_buffer[AXIS_NUMBER * i + 2] = acc_z;
		acc_buffer_x[i] = acc_x;
		acc_buffer_y[i] = acc_y;
		acc_buffer_z[i] = acc_z;
		wait_ms(10);
	}

#ifndef NEAI_LIB
	/* Print accelerometer buffer for data logging and neai emulator test modes */
	for (uint16_t isample = 0; isample < AXIS_NUMBER * DATA_INPUT_USER - 1; isample++) {
		pc.printf("%.4f ", acc_buffer[isample]);
		bt.printf("%.4f ", acc_buffer[isample]);
	}
	pc.printf("%.4f\n", acc_buffer[AXIS_NUMBER * DATA_INPUT_USER - 1]);
	bt.printf("%.4f\n", acc_buffer[AXIS_NUMBER * DATA_INPUT_USER - 1]);
	wait_ms(100);
#endif	
}

void fill_acc_buffer_2()
{
	for (uint16_t i = 0; i < DATA_INPUT_USER; i++) {
		get_acc_values();
		acc_buffer[AXIS_NUMBER * i] = acc_x;
		acc_buffer[AXIS_NUMBER * i + 1] = acc_y;
		acc_buffer[AXIS_NUMBER * i + 2] = acc_z;
		acc_buffer_x[i] = acc_x;
		acc_buffer_y[i] = acc_y;
		acc_buffer_z[i] = acc_z;
		wait_ms(0.05);
	}
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

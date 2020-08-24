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
#define LOG_NUMBER 25
#else
#define LEARNING_NUMBER 50 /* Number of learning signals */
#endif

/* Objects -------------------------------------------------------------------*/
Serial pc(USBTX, USBRX);
Serial bt(D5, D4, 9600);
Ticker toggle_led_ticker;
DigitalOut myled(LED2);
/* Blue player : */
I2C i2c_b(D0, D1);
BMI160_I2C imu_b(i2c_b, BMI160_I2C::I2C_ADRS_SDO_HI);
/* Red player : */
I2C i2c_r(D12, A6);
BMI160_I2C imu_r(i2c_r, BMI160_I2C::I2C_ADRS_SDO_HI);

BMI160::AccConfig accConfig;
BMI160::SensorData accData;

/* Variables -----------------------------------------------------------------*/
/* Blue player : */
float acc_x_b = 0.F;
float acc_y_b = 0.F;
float acc_z_b = 0.F;
float last_acc_x_b = 0.F;
float last_acc_y_b = 0.F;
float last_acc_z_b = 0.F;
/* Red player : */
float acc_x_r = 0.F;
float acc_y_r = 0.F;
float acc_z_r = 0.F;
float last_acc_x_r = 0.F;
float last_acc_y_r = 0.F;
float last_acc_z_r = 0.F;
#ifndef DATA_LOGGING
uint8_t similarity_b = 0;
uint16_t learn_cpt_b = 0;
uint8_t similarity_r = 0;
uint16_t learn_cpt_r = 0;
volatile bool newData_cs = false ;
volatile float inputs_cs[2];
volatile bool newData_ln = false ;
float inputs_ln[DATA_INPUT_USER * AXIS_NUMBER] = {0.F} ;
#endif

/* Buffer with accelerometer values for x-, y- and z-axis --------------------*/
float acc_buffer_b[DATA_INPUT_USER * AXIS_NUMBER] = {0.F};
float acc_buffer_r[DATA_INPUT_USER * AXIS_NUMBER] = {0.F};

/* Functions prototypes ------------------------------------------------------*/
#ifdef DATA_LOGGING
void data_logging_mode(void);
#endif
#ifdef NEAI_LIB
void neai_library_mode(void);
void change_score(void);
void learning_function(void);
#endif
void init(void);
void init_bmi160(void);
void toggle_led(void);
void fill_acc_buffer_b(void);
void fill_acc_buffer_r(void);
void fill_acc_buffer_print_b(void);
void fill_acc_buffer_print_r(void);
void get_acc_values_b(void);
void get_acc_values_r(void);

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
		neai_library_mode();
	#endif
}

/* Functions definition ------------------------------------------------------*/

#ifdef DATA_LOGGING

/* Logging with the blue player accelerometer */

void data_logging_mode()
{
	float start_b = 0;
	float start_r = 0;
	int compteur_b = 0;
	int compteur_r = 0;
	while(1) 
	{
		pc.printf("Blue goal logging\n");
		bt.printf("Blue goal logging\n");
		while(compteur_b < LOG_NUMBER) 
		{
			get_acc_values_b();
			start_b = fabs(acc_x_b)+fabs(acc_y_b)+fabs(acc_z_b);

			/* Waiting for the blue logging process start */
			if (start_b >= 4.0)
			{
				/* Blink LED  during logging process */
				toggle_led_ticker.attach(&toggle_led, 0.1);
				
				/* Logging process */
				fill_acc_buffer_b();

				/* Stop blink LED (end of logging process) */
				toggle_led_ticker.detach();
				myled = 0;

				/* Print logging process results */
				fill_acc_buffer_print_b();

				compteur_b ++;

				wait_ms(1000);
			}
		}

		compteur_b =0;

		pc.printf("Red goal logging\n");
		bt.printf("Red goal logging\n");
		while(compteur_r < LOG_NUMBER)
		{
			get_acc_values_r();
			start_r = fabs(acc_x_r)+fabs(acc_y_r)+fabs(acc_z_r);

			/* Waiting for the red logging process start */
			if (start_r >= 4.0)
			{
				/* Blink LED  during logging process */
				toggle_led_ticker.attach(&toggle_led, 0.1);
				
				/* Logging process */
				fill_acc_buffer_r();

				/* Stop blink LED (end of logging process) */
				toggle_led_ticker.detach();
				myled = 0;

				/* Print logging process results */
				fill_acc_buffer_print_r();

				compteur_r ++;

				wait_ms(1000);
			}
		}

		compteur_r = 0;
	}
}

#endif

#ifdef NEAI_LIB

void neai_library_mode()
{
	
	/* Learning process manual */

	/* Learning process with the blue goal accelerometer */
	
	float start_b = 0;
	pc.printf("Blue goal learning process");
	bt.printf("Blue goal learning process");
	while (learn_cpt_b < LEARNING_NUMBER)
	{
		get_acc_values_b();
		start_b = fabs(acc_x_b)+fabs(acc_y_b)+fabs(acc_z_b);

		/* Waiting for the logging process start */
		if (start_b >= 4.0)
		{
			/* Blink LED  during logging process */
			toggle_led_ticker.attach(&toggle_led, 0.1);
			
			/* Logging process */
			learn_cpt_b ++;

			fill_acc_buffer_b();
			NanoEdgeAI_learn(acc_buffer_b);
			pc.printf("%d percent \n", (int)(learn_cpt_b * 100) / LEARNING_NUMBER);
			bt.printf("%d percent \n", (int)(learn_cpt_b * 100) / LEARNING_NUMBER);

			wait_ms(3000);

			/* Stop blink LED (end of logging process) */
			toggle_led_ticker.detach();
			myled = 0;
		}
	}
	float start_r = 0;
	pc.printf("Red goal learning process");
	bt.printf("Red goal learning process");
	/* Learning process with the red player accelerometer */
	while (learn_cpt_r < LEARNING_NUMBER)
	{
		get_acc_values_r();
		start_r = fabs(acc_x_r)+fabs(acc_y_r)+fabs(acc_z_r);

		/* Waiting for the logging process start */
		if (start_r >= 4.0)
		{
			/* Blink LED  during logging process */
			toggle_led_ticker.attach(&toggle_led, 0.1);
			
			/* Logging process */
			learn_cpt_r ++;

			fill_acc_buffer_r();
			NanoEdgeAI_learn(acc_buffer_r);
			pc.printf("%d percent \n", (int)(learn_cpt_r * 100) / LEARNING_NUMBER);
			bt.printf("%d percent \n", (int)(learn_cpt_r * 100) / LEARNING_NUMBER);

			wait_ms(3000);

			/* Stop blink LED (end of logging process) */
			toggle_led_ticker.detach();
			myled = 0;
		}
	}

	/* Learning process auto*/

// 	pc.printf("Learning process : please launch 'learning.py' to start the learning\n");
// 	bt.printf("Learning process : please launch 'learning.py' to start the learning\n");
	
// 	bt.attach(&learning_function);

// 	int learning_lines = 5;
// 	int nb_lines = 0;

//   	while(nb_lines < learning_lines)
//   	{
// 	    if (newData_ln)
// 	    {
// 	      	newData_ln =  false ;
// 	      	pc.printf("Before learn\n");
// 	      	NanoEdgeAI_learn(inputs_ln);
// 	      	pc.printf("After learn\n");
// 	      	for (int i = 0; i < DATA_INPUT_USER * AXIS_NUMBER; i++)
// 	      	{
// 	      		// pc.printf("%f \n\r", inputs_ln[i]);
// 	      		inputs_ln[i] = 0;
// 	      	}
// 	      	nb_lines++;
// 	      	pc.printf("%d percent \n", (int)(nb_lines * 100) / learning_lines);
// 			bt.printf("%d percent \n", (int)(nb_lines * 100) / learning_lines);
// 	    }
// 	}

	/* Play process */
	int goals_b = 0;
	int goals_r = 0;

	pc.printf("\n Let's play ! To change the score, enter an instruction of the form 'B -1' or 'R +2' for example, then press Enter\n");
	bt.printf("\n Let's play ! To change the score, enter an instruction of the form 'B -1' or 'R +2' for example, then press Enter\n");
	
	pc.attach(&change_score);
	while(1)
	{
		if (newData_cs)
		{
		    newData_cs =  false ;
		    if (inputs_cs[0] == 'B')
		    {
		    	goals_r += inputs_cs[1];
		    	pc.printf("New score is : Blue player : %d | Red player : %d \n", goals_r, goals_b);
		    	bt.printf("New score is : Blue player : %d | Red player : %d \n", goals_r, goals_b);
		    }
		   	else if (inputs_cs[0] == 'R')
		   	{
		   		goals_b += inputs_cs[1];
		   		pc.printf("New score is : Blue player : %d | Red player : %d \n", goals_r, goals_b);
		    	bt.printf("New score is : Blue player : %d | Red player : %d \n", goals_r, goals_b);		   	
		   	}
		  	else
		   	{
		   		pc.printf("You entered an incorrect formulation\n");
		   		bt.printf("You entered an incorrect formulation\n");
		   	}
		}
		get_acc_values_r();
		get_acc_values_b();
		start_r = fabs(acc_x_r)+fabs(acc_y_r)+fabs(acc_z_r);
		start_b = fabs(acc_x_b)+fabs(acc_y_b)+fabs(acc_z_b);
		if (start_b >= 3.0 || start_r >= 3.0) 
		{
			fill_acc_buffer_r();
			fill_acc_buffer_b();
			similarity_r = NanoEdgeAI_detect(acc_buffer_r);
			similarity_b = NanoEdgeAI_detect(acc_buffer_b);
			//pc.printf("Similarity : %d blue_g and %d red_g \n", similarity_b, similarity_r);
			//bt.printf("Similarity : %d blue_g and %d red_g \n", similarity_b, similarity_r);
			if (similarity_b > 90 || similarity_r > 90)
			{
				myled = 1;
				if (start_b > start_r)
				{
					goals_b++;
				}
				else
				{
					goals_r++;
				}
				pc.printf("Blue player : %d | Red player : %d \n", goals_r, goals_b);
				bt.printf("Blue player : %d | Red player : %d \n", goals_r, goals_b);
				wait_ms(2000);
				myled = 0;
			}
		}
		if (goals_r >= 10)
		{
			pc.printf("End of the game ! Blue player won\n");
			bt.printf("End of the game ! Blue player won\n");
			goals_r = 0;
			goals_b = 0;
		}
		if (goals_b >= 10)
		{
			pc.printf("End of the game ! Red player won\n");
			bt.printf("End of the game ! Red player won\n");
			goals_r = 0;
			goals_b = 0;
		}
	}
}

void change_score()
{
	static char serialInBuffer_cs[32];
	static int serialCount_cs =  0;
	while (bt.readable())
	{
		char byteIn_cs = bt.getc();
	    if ((byteIn_cs ==  0x0D) || (byteIn_cs == 0x0A))
	    { // si une fin de ligne est trouvée
	    	serialInBuffer_cs[serialCount_cs] ==  0 ;  // null met fin à l'entrée
	    	int score;
	    	char player;
	     	if (sscanf(serialInBuffer_cs, "%c %d" , &player, &score) == 2)
	     	{  // a réussi à lire les 3 valeurs
	     		inputs_cs[0] = player;
		      	inputs_cs[1] = score;
		      	newData_cs = true;
	    	}
	    	serialCount_cs = 0; // réinitialise le tampon
	   	}
	   	else 
	   	{
	    	serialInBuffer_cs[serialCount_cs] = byteIn_cs; // stocke le caractère
	     	if (serialCount_cs < 32) // augmente le compteur
	      	{
	      		serialCount_cs ++;
	      	}
	  	}
	}
}

// void learning_function()
// {
// 	static char serialInBuffer_ln[32];
// 	static int serialCount_ln =  0;
// 	for (int i = 0; i < 384;)
// 	{
// 		while (bt.readable())
// 		{
// 			char byteIn_ln = bt.getc();
// 		    if (byteIn_ln == 0x0A)
// 		    { 
// 		    	serialInBuffer_ln[serialCount_ln] == 0 ; 
// 		    	float x;
// 		     	if (sscanf(serialInBuffer_ln, "%f" , &x) == 1)
// 		     	{
// 		     		inputs_ln[i] = x;
// 			      	newData_ln = true;
// 			      	i++;
// 			      	for (int isample = 0; isample < 32; isample ++)
// 			      	{
// 			      		serialInBuffer_ln[isample] = 0;
// 			      	}
// 		    	}
// 		    	serialCount_ln = 0;
// 		   	}
// 		   	else 
// 		   	{
// 		   		serialInBuffer_ln [serialCount_ln] = byteIn_ln;
// 		     	if (serialCount_ln < 32) 
// 		      	{
// 		      		serialCount_ln ++;
// 		      	}
// 		  	}
// 		}
// 	}
// }

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
	imu_r.setSensorPowerMode(BMI160::ACC, BMI160::NORMAL);
	imu_b.setSensorPowerMode(BMI160::ACC, BMI160::NORMAL);
	wait_ms(10);
	accConfig.range = BMI160::SENS_2G; /* Accelerometer range +-2G */
	accConfig.us = BMI160::ACC_US_OFF;
	accConfig.bwp = BMI160::ACC_BWP_2;
	accConfig.odr = BMI160::ACC_ODR_12; /* Accelerometer output data rate < 800Hz */
	imu_r.setSensorConfig(accConfig);
	imu_b.setSensorConfig(accConfig);
	wait_ms(100);
}


void toggle_led()
{
	myled = !myled;
}

// void fill_acc_buffer()
// {
// 	for (uint16_t i = 0; i < DATA_INPUT_USER; i++)
// 	{
// 		get_acc_values_b();
// 		get_acc_values_r();
// 		acc_buffer[6 * i] = acc_x_b;
// 		acc_buffer[6 * i + 1] = acc_y_b;
// 		acc_buffer[6 * i + 2] = acc_z_b;
// 		acc_buffer[6 * i + 3] = acc_x_r;
// 		acc_buffer[6 * i + 4] = acc_y_r;
// 		acc_buffer[6 * i + 5] = acc_z_r;
// 	}
// }

void fill_acc_buffer_b()
{
	for (uint16_t i = 0; i < DATA_INPUT_USER; i++)
	{
		get_acc_values_b();
		acc_buffer_b[AXIS_NUMBER * i] = acc_x_b;
		acc_buffer_b[AXIS_NUMBER * i + 1] = acc_y_b;
		acc_buffer_b[AXIS_NUMBER * i + 2] = acc_z_b;
	}
}

void fill_acc_buffer_r()
{
	for (uint16_t i = 0; i < DATA_INPUT_USER; i++)
	{
		get_acc_values_r();
		acc_buffer_r[AXIS_NUMBER * i] = acc_x_r;
		acc_buffer_r[AXIS_NUMBER * i + 1] = acc_y_r;
		acc_buffer_r[AXIS_NUMBER * i + 2] = acc_z_r;
	}
}

// void fill_acc_buffer_print()
// {
// 	for (uint16_t isample = 0; isample < 6 * DATA_INPUT_USER - 1; isample++)
// 	{
// 		if ((isample+1) % 6 == 0)
// 		{
// 			pc.printf("%.4f\n", acc_buffer[isample]);
// 			bt.printf("%.4f\n", acc_buffer[isample]);
// 		}
// 		else 
// 		{
// 			pc.printf("%.4f ", acc_buffer[isample]);
// 			bt.printf("%.4f ", acc_buffer[isample]);
// 		}
// 	}
// 	pc.printf("%.4f\n", acc_buffer[6 * DATA_INPUT_USER - 1]);
// 	bt.printf("%.4f\n", acc_buffer[6 * DATA_INPUT_USER - 1]);
// }

void fill_acc_buffer_print_b()
{
	for (uint16_t isample = 0; isample < AXIS_NUMBER * DATA_INPUT_USER - 1; isample++)
	{
		pc.printf("%.4f ", acc_buffer_b[isample]);
		bt.printf("%.4f ", acc_buffer_b[isample]);
	}
	pc.printf("%.4f\n", acc_buffer_b[AXIS_NUMBER * DATA_INPUT_USER - 1]);
	bt.printf("%.4f\n", acc_buffer_b[AXIS_NUMBER * DATA_INPUT_USER - 1]);
}

void fill_acc_buffer_print_r()
{
	for (uint16_t isample = 0; isample < AXIS_NUMBER * DATA_INPUT_USER - 1; isample++)
	{
		pc.printf("%.4f ", acc_buffer_r[isample]);
		bt.printf("%.4f ", acc_buffer_r[isample]);
	}
	pc.printf("%.4f\n", acc_buffer_r[AXIS_NUMBER * DATA_INPUT_USER - 1]);
	bt.printf("%.4f\n", acc_buffer_r[AXIS_NUMBER * DATA_INPUT_USER - 1]);
}

void get_acc_values_b()
{
	/* Polling method to get a complete buffer */
	do {
		imu_b.getSensorXYZ(accData, accConfig.range);
		acc_x_b = accData.xAxis.scaled;
		acc_y_b = accData.yAxis.scaled;
		acc_z_b = accData.zAxis.scaled;
	}
	while (acc_x_b == last_acc_x_b && acc_y_b == last_acc_y_b && acc_z_b == last_acc_z_b);
	
	last_acc_x_b = acc_x_b;
	last_acc_y_b = acc_y_b;
	last_acc_z_b = acc_z_b;
}

void get_acc_values_r()
{
	/* Polling method to get a complete buffer */
	do {
		imu_r.getSensorXYZ(accData, accConfig.range);
		acc_x_r = accData.xAxis.scaled;
		acc_y_r = accData.yAxis.scaled;
		acc_z_r = accData.zAxis.scaled;
	}
	while (acc_x_r == last_acc_x_r && acc_y_r == last_acc_y_r && acc_z_r == last_acc_z_r);
	
	last_acc_x_r = acc_x_r;
	last_acc_y_r = acc_y_r;
	last_acc_z_r = acc_z_r;
}
/* END CODE------------------------------------------------------------------- */
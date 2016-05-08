/*
 *     SocialLedge.com - Copyright (C) 2013 
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "tasks.hpp"
#include "examples/examples.hpp"
#include <stdio.h>
#include"LPC17xx.h"
#include "utilities.h"
#include "io.hpp"
#include "lpc_pwm.hpp"
#include "uart0_min.h"
#include "eint.h"
#include "event_groups.h"
#include "storage.hpp"
#include <string.h>
#include <ctime>
#include "adc0.h"
#include <iostream>
#include "i2c2.hpp"
#include "i2c_base.hpp"
#include <math.h>

using namespace std;


void * task1;
void * task2;

/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */


struct botDriver
{
	char servo;
	char dc;
}motor;

bool obstacleDetected = false;


class obstacleAvoider : public scheduler_task
{
public:
	obstacleAvoider(uint8_t priority) : scheduler_task("obstacleAvoider",2000,priority){}
	bool init(void)
	{
	    LPC_PINCON->PINSEL1 |= (1 << 20);
		LPC_GPIO1->FIODIR |= (1<<20);

		return true;
	}
	bool run(void *p)
	{
	    int adc3 = adc0_get_reading(3);

			LPC_GPIO1 -> FIOSET = (1<<20);

			printf("%d\n",adc3);
			if(adc3 < 250)
			{
				obstacleDetected = true;

	 			motor.dc &= ~(3<<0);//stop

	 			vTaskDelay(1000);

	 			motor.dc |= (2<<0);//front
				motor.servo &= ~(3<<0);
				motor.servo |= (1<<0);//left

				vTaskDelay(500);
	 			motor.dc &= ~(3<<0);//stop
				motor.servo &= ~(3<<0);
				vTaskDelay(500);

	 			motor.dc |= (2<<0);//front
				motor.servo &= ~(3<<0);
				motor.servo |= (2<<0);//right

				vTaskDelay(750);
	 			motor.dc &= ~(3<<0);//stop
				motor.servo &= ~(3<<0);
				vTaskDelay(500);

	 			motor.dc |= (2<<0);//front
				motor.servo &= ~(3<<0);
				motor.servo |= (1<<0);//left

				vTaskDelay(400);
				motor.servo &= ~(3<<0);
				motor.dc &= ~(3<<0);

				obstacleDetected = false;

			}


	    vTaskDelay(50);
 		return true;
	}
};

class dcDriveL298N : public scheduler_task
{
public:
	dcDriveL298N(uint8_t priority) : scheduler_task("dcDriveL298N",2000,priority){}
	bool init(void)
	{
		LPC_GPIO1->FIODIR |= (1<<20);
		LPC_GPIO1->FIODIR |= (1<<19);

		return true;
	}
	bool run(void *p)
	{
	    PWM dc(PWM::pwm2, 1000);
		dc.set(100);

		if((motor.dc & (1<<0)) && !(motor.dc & (1<<1))) //d0 = 1 and d1 = 0 //front
		{
 			LPC_GPIO1 -> FIOSET = (1<<19); // pwm
 			LPC_GPIO1 -> FIOCLR = (1<<20); // dir
 			//printf("front");
		}
		if(!(motor.dc & (1<<0)) && (motor.dc & (1<<1))) //d0 = 0 and d1 = 1 //back
		{
 			LPC_GPIO1 -> FIOSET = (1<<20);// dir
 			LPC_GPIO1 -> FIOCLR = (1<<19);// pwm
 			//printf("back");
		}
		if(!(motor.dc & (1<<0)) && !(motor.dc & (1<<1))) //d0 = 0 and d1 = 0 //stop
 		{
 			LPC_GPIO1 -> FIOCLR = (1<<19); // pwm
 			LPC_GPIO1 -> FIOCLR = (1<<20); // dir
 		}
		vTaskDelay(100);
 		return true;
	}
};

class servoDrive : public scheduler_task
{
public:
	servoDrive(uint8_t priority) : scheduler_task("servoDrive",2000,priority)
	{
			//nothing
	}
	bool init(void)
	{
		//setRunDuration(100);
	    return true;
	}
	bool run(void *p)
	{
		PWM servo(PWM::pwm1, 50);

		//cout<<motor.servo<<"\n";
		if(!(motor.servo & (1<<0)) && !(motor.servo & (1<<1))) //s0 = 0 and s1 = 0
		{//printf("servo stop\n");
 			servo.set(7.5);
		}
		else if((motor.servo & (1<<0)) && !(motor.servo & (1<<1))) //s0 = 1 and s1 = 0
		{//printf("leftp\n");
 			servo.set(10.0);
		}
		else if(!(motor.servo & (1<<0)) && (motor.servo & (1<<1))) //s0 = 0 and s1 = 1
		{//printf("rightp\n");
 			servo.set(5.0);
		}
		vTaskDelay(100);
		return true;
	}
};




void left()
{
		//motor.dc &= ~(3<<0);//stop
		//motor.dc |= (2<<0);//front
	motor.servo &= ~(3<<0);
	motor.servo |= (1<<0);//left
	//vTaskDelay(750);
		//motor.dc &= ~(3<<0);//stop
	//motor.servo &= ~(3<<0);
}
void right()
{
		//motor.dc &= ~(3<<0);//stop
		//motor.dc |= (2<<0);//front
	motor.servo &= ~(3<<0);
	motor.servo |= (2<<0);//right
	//vTaskDelay(750);
		//motor.dc &= ~(3<<0);//stop
	//motor.servo &= ~(3<<0);
}
void middle()
{
		//motor.dc &= ~(3<<0);//stop
		//motor.dc |= (2<<0);//front
	//motor.servo &= ~(3<<0);
	//motor.servo |= (1<<0);//left
	//vTaskDelay(1500);
		//motor.dc &= ~(3<<0);//stop
	motor.servo &= ~(3<<0);
}
void stop()
{
	motor.dc &= ~(3<<0);//stop
}
void forward()
{
	motor.dc |= (2<<0);//front
}

void back()
{
	motor.dc |= (1<<0);//back
}

enum direction: uint8_t{N,E,S,W};

int16_t min_x = 0, max_x = 0, min_y = 0, max_y =0;
static char rxData;
extern "C"
{
	void UART3_IRQHandler(void)
	{
		rxData = LPC_UART3->RBR;
		printf("receiver: %c\n",rxData);

	}
}

class i2cTask : public scheduler_task
{
    public:
	I2C2& i2c = I2C2::getInstance(); //Get I2C driver instance
	i2cTask(uint8_t priority) : scheduler_task("i2cTask", 2000, priority)
        {
        }

        bool init(void)
        {
        	LPC_I2C2->I2MASK0 = 0x1F;
        	LPC_I2C2->I2CONSET = 0x40;
        	LPC_SC->PCONP |= (1<<25);

        	//Peripheral clock select
        	LPC_SC->PCLKSEL1 &= ~(3<<18);
        	LPC_SC->PCLKSEL1 |= (1<<18);

        	//Selecting TXD3, RXD3
        	LPC_PINCON->PINSEL9 &= ~((3<<24)|(3<<26));
        	LPC_PINCON->PINSEL9 |= ((3<<24)|(3<<26));

        	//set DLM and DLL for desired baudrate
        	set_baud_rate(9600);

        	//Enable Interrupt for UART3
        	enable_interrupt();

        	return true;
        }
        void set_baud_rate(uint16_t req_baudrate)
        {
        	//Setting the Baud Rate 9600: Baud Rate = PCLK/16(DLM:DLL) = 48000000/16(9600)
        	uint16_t baudrate_div;

        	baudrate_div = sys_get_cpu_clock()/(16*req_baudrate); //baudrate_div = 48000000/(16*req_baudrate);

        	LPC_UART3->LCR = 128;       				 //Setting DLAB 1
        	LPC_UART3->DLL = (baudrate_div & 0xFF);		 //Setting DLL
        	LPC_UART3->DLM = (baudrate_div >> 8);		 //Setting DLM
        	//Disabling DLAB to enable THR and RBR registers and setting word length to be 8 character
        	LPC_UART3->LCR = 3;
        }

        void enable_interrupt(void)
        {
        	//Enabling RBR Interrupt for UART3
        	NVIC_EnableIRQ(UART3_IRQn);
        	LPC_UART3->IER |= 1<<0;
        }

        void uart3_TransferByte(char out)
        {
        	LPC_UART3->THR = out;
        	while(!(LPC_UART3->LSR & (1<<5))); //wait until data is transmitted
        }

        bool run(void *p)
        {
        	direction slave_dir;
        	uint8_t slaveAddr = 0x1E;
        	LPC_I2C2->I2ADR0 = slaveAddr;
        	uint8_t xMHiByte = 0, xMLoByte = 0, yMHiByte =0, yMLoByte=0, zMHiByte=0, zMLoByte=0;
        	uint8_t xAHiByte = 0, xALoByte = 0, yAHiByte =0, yALoByte=0, zAHiByte=0, zALoByte=0;
        	int16_t xMagData =0, yMagData =0, zMagData=0, xAccData=0, yAccData=0, zAccData=0;
        	float heading = 0.0, absHeading = 0.0;
        	i2c.writeReg(0x3C,0x02,0x00);
        	xMHiByte = i2c.readReg(0x3D, 0x03);
        	xMLoByte = i2c.readReg(0x3D, 0x04);
        	yMHiByte = i2c.readReg(0x3D, 0x07);
        	yMLoByte = i2c.readReg(0x3D, 0x08);
        	zMHiByte = i2c.readReg(0x3D, 0x05);
        	zMLoByte = i2c.readReg(0x3D, 0x06);
        	xMagData = (xMHiByte << 8) | xMLoByte;
        	yMagData = (yMHiByte << 8) | yMLoByte;
        	zMagData = (zMHiByte << 8) | zMLoByte;
        	heading = (atan2(yMagData,xMagData)*180)/3.14159;

        	if(heading < 0)
        	{
        		heading = 360 + heading;
        	}

        	char rotation_character = char(((abs(heading) * (255-33)) / 360) + 33);
        	uart3_TransferByte(rotation_character);

        	printf("slaveAngle : %d\n",abs(heading));

        	if(rxData == '6')
        		right();

        	else if(rxData == '4')
        		left();

        	else if(rxData == '5')
        		middle();

        	else if(rxData == '8')
        		forward();

        	else if(rxData == '2')
        		stop();

        	else if(rxData == '3')
        		back();

/*

        	if(heading <=225 && heading >=135)
        	{
        		slave_dir = S;
        	}
        	else if(heading >=225 && heading <=315)
        	{
        		slave_dir = W;
        	}
        	else if(heading <=15 || heading >=315)
        	{
        		slave_dir = N;
        	}
        	else if(heading >=15 && heading <=135)
        	{
        		slave_dir = E;
        	}
        	//printf("Magnetometer: %02x    	%02x	   %02x\n",xMagData,yMagData,zMagData);
        	//printf("Heading: %f       direction %c\n",heading,direction );
        	//printf("Master's direction: %c\n",masterDir);
        	uint8_t masterDir,masterMotion;
        	int desiredDir;
        	masterDir = (rxData>>4) & 0x0F;
        	masterMotion = (rxData & 0x0F);


        	if (masterMotion == 0)
        	{
        		printf("\nStop\n");
        		stop();
        	}

        	else if (masterMotion == 1)
        	{
        		printf("\nRun\n");
        		forward();
        		//vTaskDelay(1000);
        		//stop();

        	}
        	else if(masterMotion == 2)
        	{
        		printf("\nContinue\n");
        		forward();
        		//vTaskDelay(1000);
        		//stop();
        	}

        	//printf("rxData: %d\n",rxData);

        	printf("Slave's direction: %d\n",slave_dir);
        	printf("Master's direction: %d\n",masterDir);

        	desiredDir = slave_dir - masterDir;
        	printf("desiredDir: %d\n",desiredDir);


        	if(desiredDir == -1 || desiredDir == 3)
        	{
           		right();
        		printf("right\n");
        	}
        	else if(desiredDir == 1 || desiredDir == -3)
        	{
        		printf("left\n");
        		left();
        	}
        	else if(desiredDir == 0)
        	{
        		//forward();
        		middle();
        		printf("Forward\n");
        	}
        	else if(abs(desiredDir) == 2)
        	{
        		left();
        		printf("u_turn\n");
        	}


*/

        	vTaskDelay(100);
        	return true;
        }
};



/*
class pwmTest : public scheduler_task
{
public:
	pwmTest(uint8_t priority) : scheduler_task("pwmTest",2000,priority){}
	bool init(void)
	{
		LPC_GPIO1->FIODIR |= (1<<20);//dir


		LPC_GPIO1->FIODIR &= ~(1<<9);
		LPC_GPIO1->FIODIR &= ~(1<<10);
		LPC_GPIO1->FIODIR &= ~(1<<14);
 		LPC_GPIO1->FIODIR &= ~(1<<15);

		return true;
	}
	bool run(void *p)
	{
		PWM servo(PWM::pwm1, 50);

		PWM dc(PWM::pwm2, 1000);

		if(LPC_GPIO1 -> FIOPIN & (1<<9))
		{
			dc.set(50);
 			LPC_GPIO1 -> FIOCLR = (1<<20); // dir
 			printf("front");
		}
		else if(LPC_GPIO1 -> FIOPIN & (1<<10))
		{
 			LPC_GPIO1 -> FIOSET = (1<<20);// dir
 			dc.set(50);
 			printf("back");
		}
		else
 		{
			dc.set(100);
 			LPC_GPIO1 -> FIOCLR = (1<<20); // dir
 		}


		if(LPC_GPIO1 -> FIOPIN & (1<<14))
		{
 			servo.set(5);
		}
		else if(LPC_GPIO1 -> FIOPIN & (1<<15))
		{
 			servo.set(10.0);
		}
		else
		{
 			servo.set(7.5);
		}

		vTaskDelay(100);
 		return true;
	}
};
*/




int main(void)
{
	//scheduler_add_task(new dcDrive(PRIORITY_HIGH));
	scheduler_add_task(new dcDriveL298N(PRIORITY_HIGH));
	scheduler_add_task(new servoDrive(PRIORITY_HIGH));
	scheduler_add_task(new i2cTask(PRIORITY_HIGH));
	//scheduler_add_task(new uart_lab(PRIORITY_HIGH));
	//scheduler_add_task(new switchController(PRIORITY_HIGH));
	//scheduler_add_task(new xBeeReceiver(PRIORITY_HIGH));
	//scheduler_add_task(new obstacleAvoider(PRIORITY_HIGH));

//	vSemaphoreCreateBinary(mySemaphore);
//	xSemaphoreTake(mySemaphore, 0);
//	xTaskCreate(giveSemaphore,(const char *)"giveSemaphore",STACK_BYTES(2048),0,3,0);
//	xTaskCreate(takeSemaphore,(const char *)"takeSemaphore",STACK_BYTES(2048),0,3,0);
//

//  scheduler_add_task(new gpioTask(PRIORITY_HIGH));
//  scheduler_add_task(new uart_lab(PRIORITY_HIGH));
//	scheduler_add_task(new pwm_self(PRIORITY_HIGH));

//	dirQueue = xQueueCreate(1, sizeof(orientation_t));
//	xTaskCreate(dirProducer,(const char *)"dirProducer",STACK_BYTES(2048),0,1,0);
//	xTaskCreate(dirConsumer,(const char *)"dirConsumer",STACK_BYTES(2048),0,2,0);
//


/*	lightQueue = xQueueCreate(1, sizeof(int));
	xEventGroup = xEventGroupCreate();
	xTaskCreate(producer,(const char *)"producer",STACK_BYTES(2048),0,PRIORITY_MEDIUM,&task1);
	xTaskCreate(consumer,(const char *)"consumer",STACK_BYTES(2048),0,PRIORITY_MEDIUM,&task2);
	xTaskCreate(watchDog,(const char *)"watchDog",STACK_BYTES(2048),0,PRIORITY_HIGH,0);

	xTaskCreate(cpu_usage,(const char *)"cpu_usage",STACK_BYTES(2048),0,PRIORITY_HIGH,0);*/


    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    scheduler_add_task(new periodicSchedulerTask());
    #endif

    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}
//master.cpp

#include "tasks.hpp"
#include "examples/examples.hpp"
#include "command_handler.hpp"
#include <stdio.h>
#include <math.h>
#include"LPC17xx.h"
#include "utilities.h"
#include "io.hpp"
#include "lpc_pwm.hpp"
#include "uart0_min.h"
#include "eint.h"
#include "event_groups.h"
#include "i2c2.hpp"
#include "i2c_base.hpp"

int slaveAngle,masterAngle;
int zAccData[3]={0};
SemaphoreHandle_t dataRx;

extern "C"
{
	void UART3_IRQHandler(void)
	{
		int rotation_integer_received = int(LPC_UART3->RBR);
		slaveAngle = (360 * (rotation_integer_received - 33)) / (255 - 33);
		xSemaphoreGiveFromISR(dataRx,NULL);

	}
}


class lsm303 : public scheduler_task
{
    public:
	I2C2& i2c = I2C2::getInstance(); //Get I2C driver instance
	lsm303(uint8_t priority) : scheduler_task("lsm303", 2000, priority)
        {
            /* Nothing to init */
        }

        bool init(void)
        {
    		LPC_GPIO1->FIODIR &= ~(1<<9);


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


        //Function to determine the direction command for the Bot
        char direction_command()
        {

        	int16_t checkMasterClockwise,checkMasterAntiClockwise;
        	int8_t turningAngle = 25;
        	int angleDifference;
        	char steerCommand;
        	uint8_t xMHiByte = 0, xMLoByte = 0, yMHiByte =0, yMLoByte=0,zMHiByte=0, zMLoByte=0;
        	int16_t xMagData =0, yMagData =0, zMagData=0;
        	float heading = 0.0;

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

			masterAngle = heading;

          	angleDifference = masterAngle - slaveAngle;

			if((abs(angleDifference) < turningAngle) || (abs(angleDifference) > (360 -turningAngle)))
			{
    			steerCommand = '5';		//Command Bot to move forward
			}
			else if(((angleDifference < 0) && (abs(angleDifference) < 180)) || ((angleDifference > 0) && (abs(angleDifference) > 180)))
			{
    			steerCommand = '4';		//Command Bot to steer Left
			}
			else if(((angleDifference < 0) && (abs(angleDifference) > 180)) || ((angleDifference > 0) && (abs(angleDifference) < 180)))
			{
    			steerCommand = '6';		//Command Bot to steer Right
			}
        	return steerCommand;
        }


        //Function to determine the stop or move command for the Bot
        char motion_detect()
        {
        	static int numAccSample = 0;
        	char botCommand;
        	//Collect 3 consecutive data samples from accelerometer
        	if(numAccSample<3)
        	{
        		zAccData[numAccSample] = AS.getZ();
        	}
        	else
        	{
        		numAccSample=0;                    //if number of samples =3, start storing from 0 again
        		zAccData[numAccSample] = AS.getZ();

        	}
        	numAccSample++;
        	//Compare the data sample values to determine user's movement
        	if((abs(zAccData[0]-zAccData[1])<=100)&&(abs(zAccData[1]-zAccData[2])<=100))
        	{
        		botCommand = '2';                        //STOP
        	}
        	else
        	{
        		botCommand = direction_command();		//MOVE
        	}
        return botCommand;
        }

        bool run(void *p)
        {

        	uint8_t slaveAddr = 0x1E;
        	char transmit = '0';
        	LPC_I2C2->I2ADR0 = slaveAddr;

        	if(xSemaphoreTake(dataRx,portMAX_DELAY))
        	{
        		//Transmit Master motion and direction data to the Bot
        		transmit = motion_detect();
        		uart3_TransferByte(transmit);
        	}
        	return true;
        }
};


int main(void)
{
	vSemaphoreCreateBinary(dataRx);    //Create the binary semaphore
	xSemaphoreTake(dataRx,0);
	scheduler_add_task(new lsm303(PRIORITY_HIGH));

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




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

//QueueHandle_t runQueue;

void * task1;
void * task2;
int slaveAngle,masterAngle;
char transmit = '0';


extern "C"
{
	void UART3_IRQHandler(void)
	{
		int rotation_integer_received = int(LPC_UART3->RBR);
		slaveAngle = (360 * (rotation_integer_received - 33)) / (255 - 33);
		//printf("rotation_integer_received: %d\n",rotation_integer_received);
	}
}

void uart3_TransferByte(char out)
{
	LPC_UART3->THR = out;
	while(!(LPC_UART3->LSR & (1<<5))); //wait until data is transmitted
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


        bool run(void *p)
        {

        	uint8_t slaveAddr = 0x1E;
        	LPC_I2C2->I2ADR0 = slaveAddr;
        	//uint8_t txData=0;
        	uint8_t xMHiByte = 0, xMLoByte = 0, yMHiByte =0, yMLoByte=0, zMHiByte=0, zMLoByte=0;
        	uint8_t xAHiByte = 0, xALoByte = 0, yAHiByte =0, yALoByte=0, zAHiByte=0, zALoByte=0;
        	int16_t xMagData =0, yMagData =0, zMagData=0, xAccData=0, yAccData=0, InbuiltAccX, InbuiltAccY, InbuiltAccZ;
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
        	//char direction;
        	if(heading < 0)
        	{
        		heading = 360 + heading;
        	}

        	masterAngle = heading;

        	//printf("masterAngle : %d\nslaveAngle : %d\n",masterAngle,slaveAngle);

        	int checkMasterClockwise 	 = masterAngle + 25;
        	int checkMasterAntiClockwise = masterAngle - 25;

    		if(checkMasterClockwise >= 360)
    			checkMasterClockwise -= 360;

    		if(checkMasterAntiClockwise < 0)
    			checkMasterAntiClockwise -= 0;


    		for(int x = checkMasterAntiClockwise; x < checkMasterClockwise; x++)
    		{
    			if(x > 359)
    				x = 0;

    			if(x == slaveAngle)
    			{
    				//printf("middle\n");
    				transmit = '5';
    			}
    		}

        	for(int x = 0; x < 155; x++)
        	{

        		if(checkMasterClockwise == slaveAngle)
        		{
        			//printf("steer left\n");
        			transmit = '4';
        		}

        		if(checkMasterAntiClockwise == slaveAngle)
        		{
        			//printf("steer right\n");
        			transmit = '6';
        		}

        		checkMasterClockwise++;
        		if(checkMasterClockwise >= 360)
        			checkMasterClockwise=0;

        		checkMasterAntiClockwise--;
        		if(checkMasterAntiClockwise < 0)
        			checkMasterAntiClockwise = 359;
        	}
        	uart3_TransferByte(transmit);

    		vTaskDelay(100);

        	if(AS.getY() < -550)
        	{
        		//printf("run\n");
				uart3_TransferByte('8');
        	}
        	else if(AS.getY() > 550)
        	{
				uart3_TransferByte('3');
        	}
        	else
        	{
        		//printf("stop\n");
				uart3_TransferByte('2');
        	}

        	vTaskDelay(100);

        	return true;
        }
};

void throttle(void *p)
{
	while(1)
	{
        	//printf("x : %d    y : %d    z : %d\n",AS.getX(),AS.getY(),AS.getZ());


        	int acc1, acc2, speed1, speed2;

        	acc1 = AS.getX()+AS.getY()+AS.getZ();
        	vTaskDelay(50);
        	acc2 = AS.getX()+AS.getY()+AS.getZ();
        	speed1 = ((acc1 + acc2) / 2) /10;

        	acc1 = AS.getX()+AS.getY()+AS.getZ();
        	vTaskDelay(50);
        	acc2 = AS.getX()+AS.getY()+AS.getZ();
        	speed2 = ((acc1 + acc2) / 2) /10;

        	if(abs(speed1 - speed2) > 5 )
        	{
        		transmit = '8';
				printf("run\n");
				uart3_TransferByte(transmit);
        	}

        	else
        	{
        		transmit = '2';
				printf("stop\n");
				uart3_TransferByte(transmit);
        	}


		vTaskDelay(100);
	}
}


int main(void)
{

	scheduler_add_task(new lsm303(PRIORITY_HIGH));
	//xTaskCreate(throttle,(const char *)"throttle",STACK_BYTES(2048),0,PRIORITY_HIGH,0);

    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    #if 0
    scheduler_add_task(new periodicSchedulerTask());
    #endif

    #if 0
        scheduler_add_task(new example_io_demo());
    #endif


    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

     #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start();
    return -1;
}









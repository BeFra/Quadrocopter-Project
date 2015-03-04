#include <avr/io.h>
#include <avr/pgmspace.h>

#include "utils/usb_debug_only.h"
#include "utils/print.h"
#include "mpu6050/sensor.h"
#include "i2c/i2c.h"
#include "utils/UsbTransmit.h"
#include "time/time.h"
#include <stdio.h>

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
#ifndef F_CPU
#define F_CPU    16000000UL //CPU Clock 16Mhz
#endif
#include <util/delay.h>
int main(void)
{
	// set for 16 MHz clock, and turn on the LED
	CPU_PRESCALE(0);


	/* initialize the USB, and then wait for the host
	 to set configuration.  If the Teensy is powered
	 without a PC connected to the USB port, this 
	 will wait forever.*/
	usb_init();
	while (!usb_configured()) /* wait */ ;
	
	
	init_timer();

	/*wait an extra second for the PC's operating system
	 to load drivers and do whatever it does to actually
	 be ready for input*/

	_delay_ms(1000);
	_delay_ms(1000);
	
	print("Quadrocopter\n");
	print("\n");
	
	init_sensor();
		

	float temperature = get_temperature();
	get_gyro(gyro);
	get_acc(acc);
	get_angles();
	//print_sensor_Data(gyro,acc,temperature);
	usb_debug_flush_output();

	 return 0;
}



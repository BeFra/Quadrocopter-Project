#include "usb_debug_only.h"
#include "print.h"
#include "UsbTransmit.h"
#include <stdio.h>


void print_sensor_Data(float* gyro, float* acc, float temperature) {
	char tmp[10];
	
	print("Gyroscope Data x, y, z:     ");

	for( int i = 0; i < 3; i++ ) {
		sprintf(tmp,"%f",gyro[i]);
// 		if(i == 0) {
// 			print("X-Achse: ");
// 		} else if( i == 1) {
// 			print("Y-Achse: ");
// 		} else if( i == 2 ) {
// 			print("Z-Achse: ");
// 		}
		for(int x = 0; x < 5; x++) {
			pchar(tmp[x]);
		}
		print(", ");
	}
	print("\nAccelerometer Data x, y, z: ");
	for( int i = 0; i < 3; i++ ) {
		sprintf(tmp,"%f",acc[i]);
// 		if(i == 0) {
// 			print("X-Achse: ");
// 		} else if( i == 1) {
// 				print("Y-Achse: ");
// 		} else if( i == 2 ) {
// 			print("Z-Achse: ");
// 		}
		for(int x = 0; x < 5; x++) {
			pchar(tmp[x]);
		}
		print(", ");
	}
	print("\nTemperature Data: ");
	sprintf(tmp,"%f",temperature);
	for( int i = 0; i < 5; i++ ) {
		pchar(tmp[i]);
	}
	print("\n");
	usb_debug_flush_output();	
}
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <library_extansion.h>

#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)



static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;


int main(void)
{
	//initialisation
    halInit();
    chSysInit();
    mpu_init();
    VL53L0X_start();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //init the motors
    motors_init();

    //Mettre perpendiculairement aux parois
    calibration_angle(-1);
    //Déterminer axe des y le plus long
	determine_x_y_axis();
	//Placer le robot dans un coin
	placement_corner();
	//aller au premier
	go_from_to(0,0,200,200);

	return 1;

}


void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

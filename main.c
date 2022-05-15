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
#include <audio/play_melody.h>
#include <audio/audio_thread.h>
#include <audio/play_sound_file.h>

#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

//static struct Mypoint tab_point[2] = {{40, 40, 2, TARAUDAGE}, {90, 90, 3, PERCAGE}};
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
/*
//est-ce que struct est bien d'implementer dans le projet?
void go_through_points(struct Mypoint *tab_point[]){
	for(int i=1;i<2;i++){
		//go_from_to(Points_X[i-1],Points_Y[i-1],Points_X[i],Points_Y[i]);
		go_from_to(tab_point[i-1]->x, tab_point[i-1]->y, tab_point[i]->x, tab_point[i]->y);
*/


int main(void)
{
	//initialisation
    halInit();
    chSysInit();
    mpu_init();
    dac_start();
    setSoundFileVolume(1);
    playMelodyStart();
    VL53L0X_start();
    // Enable GPIOD peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIODEN;
    // Enable GPIOB peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN;

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //init the motors
    motors_init();

    start_program();
    start_music();
    start_microphone();
//    led();


//	while (1) {
////		chprintf((BaseSequentialStream *)&SD3, "Bonjour");
//		int avg_distance=0;
//		for(int i=0;i<5000;i++){
//			tab_angle[i] = VL53L0X_get_dist_mm();
//			avg_distance+=tab_angle[i];
//		}
//		avg_distance=avg_distance/5000;
//		chprintf((BaseSequentialStream *)&SD3, "Distance = %d\r\n\n", avg_distance);
//	    chThdSleepMilliseconds(1000);
//	}
//    calibration_angle();
    while(1){}

	return 0;
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

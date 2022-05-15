#include <main.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chprintf.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>

#include <motors.h>
#include <audio/microphone.h>
#include <musique.h>
#include <sensors/VL53L0X/VL53L0X.h>
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

void initialisation(void){
	halInit();
	chSysInit();
	mpu_init();
	dac_start();
	setSoundFileVolume(1);
	playMelodyStart();
	VL53L0X_start();

   //starts the serial communication
   serial_start();
   //starts the USB communication
   usb_start();
   //init the motors
   motors_init();
}


int main(void)
{
	//initialisation
	initialisation();

	//Lancer les threads mouvements, musique et microphone
    start_program();
    start_music();
    start_microphone();

    while(1){
    	chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

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
/*
//structure
struct Mypoint {
	int x; //coordonnée en x en mm
	int y; //coordonnée en y en mm
	int diametre; //diamètre en mm
	uint8_t type; //type de trou : perçage = p, taraudage = t
};*/

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
    //Initiation du tableau de valeur

    start_program();
    /*
    //Mettre perpendiculairement aux parois
    calibration_angle(-1);
    //Déterminer axe des y le plus long
	determine_x_y_axis();
	//Placer le robot dans un coin
	placement_corner();
	chprintf((BaseSequentialStream *)&SDU1, "Distance = %d\n", VL53L0X_get_dist_mm());
	//aller au premier
	go_from_to(0,0,200,200);
	*/

	while (1) {
		//chprintf((BaseSequentialStream *)&SD3, "Bonjour");
		//chprintf((BaseSequentialStream *)&SDU1, "Distance = %d\n", VL53L0X_get_dist_mm());
	    chThdSleepMilliseconds(100);
	}


}


void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

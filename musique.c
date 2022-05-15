#include "ch.h"
#include "chbsem.h"
#include "hal.h"
#include "motors.h"
#include "audio_processing.h"
#include "library_extansion.h"
#include "motor_extansion.h"
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <audio/play_melody.h>

#define TARAUDAGE			0
#define PERCAGE				1

typedef enum {
	two_mm=0,
	four_mm,
	six_mm,
	eight_mm
};

// semaphore
static BSEMAPHORE_DECL(mvtplay_sem, TRUE);

void LED_Toggle(uint8_t diametre ){

	switch(diametre){
		case two_mm:
			palTogglePad(GPIOD, GPIOD_LED7);
			break;
		case four_mm:
			palTogglePad(GPIOD, GPIOD_LED7);
			palTogglePad(GPIOD, GPIOD_LED5);
			break;
		case six_mm:
			palTogglePad(GPIOD, GPIOD_LED7);
			palTogglePad(GPIOD, GPIOD_LED5);
			palTogglePad(GPIOD, GPIOD_LED3);
			break;
		case eight_mm:
			palTogglePad(GPIOD, GPIOD_LED7);
			palTogglePad(GPIOD, GPIOD_LED5);
			palTogglePad(GPIOD, GPIOD_LED3);
			palTogglePad(GPIOD, GPIOD_LED1);
			break;
		default:
			break;
	}

}
static THD_WORKING_AREA(waMusic, 256);
static THD_FUNCTION(Music, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	chBSemWait(&mvtplay_sem);
		if(get_tab_point() == TARAUDAGE){
			LED_Toggle(get_diametre());
//			wait(50000000);
			playMelody(MARIO_START, ML_SIMPLE_PLAY, NULL);
			waitMelodyHasFinished();
			stopCurrentMelody();
			LED_Toggle(get_diametre());
		} else {
			LED_Toggle(get_diametre());
//			wait(50000000);
			playMelody(MARIO_DEATH, ML_SIMPLE_PLAY, NULL);
			waitMelodyHasFinished();
			stopCurrentMelody();
			LED_Toggle(get_diametre());
		}
		set_semamicro();
    }

}

void set_semamvtplay(void){
	chBSemSignal(&mvtplay_sem);
}

void start_music(void){
	chThdCreateStatic(waMusic, sizeof(waMusic), NORMALPRIO, Music, NULL);
}

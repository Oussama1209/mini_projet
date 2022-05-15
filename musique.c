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

// semaphore
static BSEMAPHORE_DECL(mvtplay_sem, TRUE);

static THD_WORKING_AREA(waMusic, 256);
static THD_FUNCTION(Music, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	//Tant que la thread mouvement n'a pas atteint un trou, alors on ne joue aucune musique
    	chBSemWait(&mvtplay_sem);

    	//Si le type est taraudage, on joue Mario start, s'il est Perçage, on joue Mario death
    	//Et si c'est le premier trou, on ne joue aucune musique
		if(get_tab_point() == TARAUDAGE){
			playMelody(MARIO_START, ML_SIMPLE_PLAY, NULL);
			//on s'assure que la musique a été complètement joué avant de permettre à la thread
			//microphone de se lancer et d'écouter des sons
			waitMelodyHasFinished();
			stopCurrentMelody();
		} else if (get_tab_point() == PERCAGE) {
			playMelody(MARIO_DEATH, ML_SIMPLE_PLAY, NULL);
			waitMelodyHasFinished();
			stopCurrentMelody();
		}
		set_semamicro();
    }

}

//Envoie le signal de la sémaphore mvtplay
void set_semamvtplay(void){
	chBSemSignal(&mvtplay_sem);
}

//Crée la thread musique
void start_music(void){
	chThdCreateStatic(waMusic, sizeof(waMusic), NORMALPRIO, Music, NULL);
}

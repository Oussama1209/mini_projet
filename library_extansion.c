#include "ch.h"
#include "chbsem.h"
#include "hal.h"
#include "motors.h"
#include "audio_processing.h"
#include "library_extansion.h"
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <audio/play_melody.h>
//#include <leds.h>

//Semaphore
static BSEMAPHORE_DECL(sendasound_sem, TRUE);
//static BSEMAPHORE_DECL(sendamusic_sem, TRUE);
static BSEMAPHORE_DECL(mvtplay_sem, TRUE);

#define SPEED_MOTOR   		600 // speed of motor
#define NSTEP_ONE_TURN      1000 // number of steps for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13 // [cm]
#define PI                  3.1415926536f
#define ONE_SEC 			SystemCoreClock/16
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
//Dimension de la planche
#define distance_x			200		//largeur in mm
#define	distance_y			260		//longueur in mm
#define SAMPLE_SIZE			10000	//number of samples used
#define SIDES				4		//number of sides of the board
#define POINTS				2		//number of points to determine on the board

#define MUR_DETECTER		0
#define AUCUN_MUR			1
#define SIZE_TAB			2		//taille du tableau
#define TARAUDAGE			0
#define PERCAGE				1

static int avg_distance=0;
static int distance_min=8000;
static int tab_angle[SAMPLE_SIZE];
static int sides[SIDES];

static bool detection_mur = AUCUN_MUR;
static uint8_t pos_tab = 0;
static bool programme_fini = 1;

//static bool microphone_begin = 1;
static bool music_begin = 0;

//structure
struct Mypoint {
	uint16_t x; //coordonnée en x en mm
	uint16_t y; //coordonnée en y en mm
	uint8_t diametre; //diamètre en mm
	uint8_t type; //type de trou : perçage = p, taraudage = t
};

static struct Mypoint tab_point[2] = {{10, 10, 2, TARAUDAGE}, {20, 20, 3, PERCAGE}};

// Simple delay function
void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
}

void init_position_motor(void){
	right_motor_set_pos(0);
	left_motor_set_pos(0);
}

void set_motor_speed(uint16_t speed){
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
}

//Faire un quart de tour
void quarter_turns(uint8_t num_of_quarter_turns, int8_t direction){
	init_position_motor();
	left_motor_set_speed(-direction * SPEED_MOTOR);
	right_motor_set_speed(direction * SPEED_MOTOR);
	if (direction == 1){
		//Si moteur droite tourne positivement
		while(right_motor_get_pos() < num_of_quarter_turns*PERIMETER_EPUCK/4* NSTEP_ONE_TURN / WHEEL_PERIMETER);
	}else{
		//Si moteur gauche tourne positivement
		while(left_motor_get_pos() < num_of_quarter_turns*PERIMETER_EPUCK/4* NSTEP_ONE_TURN / WHEEL_PERIMETER);
	}
	set_motor_speed(0);
}
//Tourner d'un n ième tour
void nieme_turn(uint8_t nieme_value, int8_t direction){
	init_position_motor();
	left_motor_set_speed(-direction * SPEED_MOTOR);
	right_motor_set_speed(direction * SPEED_MOTOR);
	if (direction == 1){
		while(right_motor_get_pos() < PERIMETER_EPUCK/nieme_value* NSTEP_ONE_TURN / WHEEL_PERIMETER){}
	}else
	{
		while(left_motor_get_pos() < PERIMETER_EPUCK/nieme_value* NSTEP_ONE_TURN / WHEEL_PERIMETER){}
	}
	set_motor_speed(0);
}

//the TOF waits a certain time before calculating distance
int check_distance(void){
//	wait(5000000); //mettre la fonction Thdsleepmilisecond...
	return VL53L0X_get_dist_mm();
}

//sets the motors to go forward
void go_forward(void){
	right_motor_set_speed(600);
	left_motor_set_speed(600);
}

//stops the motor
void stop_motor(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);

}

//sets the robot on a perpendicular line to the side that the user faced it towards
void calibration_angle(void){

	int direction;
	int calibration_check=0;
	int min=0;

	// approaches the side that the user puts it in front of
	while(VL53L0X_get_dist_mm()>200){
		go_forward();
	}

	bool turn_right=false;
	delay(ONE_SEC);
	int test_1=VL53L0X_get_dist_mm();
	delay(ONE_SEC);
	nieme_turn(10,-1);
	delay(ONE_SEC);
	int test_2=VL53L0X_get_dist_mm();
	delay(ONE_SEC);
	nieme_turn(10,-1);
	delay(ONE_SEC);
	int test_3=VL53L0X_get_dist_mm();
	delay(ONE_SEC);
	turn_right=(test_1>test_2)&&(test_2<test_3)&&(test_3<test_1);
	if(turn_right) {
		direction=-1;
//		nieme_turn(5,1);
	}
	else{
		direction=1;
		nieme_turn(5,1);
	}

	//makes small turns (1/200 turn) and compares the distance it receives with the ones before
	//if the distance gets lower and goes higher after then it means it was perpendicular to the side it faces
	while(calibration_check<8){

		//takes average of the vales of distance the TOF receives
		for(int i=0;i<SAMPLE_SIZE;i++){
//			wait(500);
			tab_angle[i] = VL53L0X_get_dist_mm();
			avg_distance+=tab_angle[i];
		}
		avg_distance=avg_distance/SAMPLE_SIZE;

		//if distance gets lower it means the perpendicular line is still not obtained
		if(avg_distance<distance_min) {
			distance_min=avg_distance;
			min++;
			calibration_check=0;
		}

		//if distance gets higher after it was getting lower for a number of checks then it
		//confirms the perpendicular line
		if(avg_distance>distance_min && avg_distance<400 && min>2) {
			calibration_check++;
		}

		//makes 1/200 turn
		nieme_turn(200,direction);
		delay(ONE_SEC);
	}

	//returns back to the perpendicular line
	for(int i=0;i<7;i++) nieme_turn(200,-direction);

	//sets the values to their original ones for another calibration
	distance_min=8000;
	avg_distance=0;
}

void perpendiculaire(void){
	static uint8_t position = 0;
	static uint8_t good_position = 0;
	for(int i=0 ;i < 200; i++){
		//takes average of the vales of distance the TOF receives
		for(int i=0;i<SAMPLE_SIZE;i++){
			//chThdSleepMilliseconds(10);
//			wait(500);
			avg_distance+=VL53L0X_get_dist_mm();
		}
		avg_distance=avg_distance/SAMPLE_SIZE;
		chprintf((BaseSequentialStream *)&SD3, "Dist min =  %d\n", distance_min);
		if(avg_distance < distance_min){
			good_position = position;
			distance_min = avg_distance;
			chprintf((BaseSequentialStream *)&SD3, "goodpos =  %d\n", good_position);
		}
		//chprintf((BaseSequentialStream *)&SD3, "Oooooh");
		nieme_turn(200,1);
		position += 1;
	}
	nieme_turn(200/good_position, 1);
}

//determine the y and x axis on the board
void determine_x_y_axis(void){

	//turn quarter turns and calculates distance to each side from the robot
	for(int i=0;i<SIDES;i++){
		sides[i]=check_distance();
		quarter_turns(1,1);
	}

	//addition of both distances to calculate the longueur and largeur
	int dist_prov_1=sides[0]+sides[2];
	int dist_prov_2=sides[1]+sides[3];

	//compare both values
	//y axis is always the bigger, it's a convention to avoid problems reading the coordinates
	if(dist_prov_1<dist_prov_2) quarter_turns(1,1);


}

//places the robot in the corner of the board
void placement_corner(void){

	while(VL53L0X_get_dist_mm()>50){
		go_forward();
	}
	quarter_turns(1,-1);
	while(VL53L0X_get_dist_mm()>50){
		go_forward();
	}
	quarter_turns(1,-1);
	stop_motor();

}


//moves the robot according to the initial and arrival coordinates of y axis
//because its in mm it's okey to have int instead of float
void go_y(int y_i, int y_f){

	//compares value of initial and final coordinates
	int compare = y_f - y_i;

	//if final coordinate is bigger then move forward to the coordinate
	if (compare>0) {
		while(check_distance()>distance_y-y_f){
			go_forward();
		}
		stop_motor();
	}

	//if final coordinate is smaller then turn back and go to the coordinate
	else{
		quarter_turns(2,-1);
		while(check_distance()> y_f){
			go_forward();
		}
		quarter_turns(2,1);
		stop_motor();
	}
}

//moves the robot according to the initial and arrival coordinates of x axis
//because its in mm it's okay to have int instead of float
void go_x(int x_i, int x_f){

	//compares value of initial and final coordinates
	int compare = x_f - x_i;

	//if final coordinate is bigger then turn right and move forward to the coordinate
	if (compare>0) {
		quarter_turns(1,-1);
		while(check_distance()>distance_x-x_f){
			go_forward();
		}
		quarter_turns(1,1);
		stop_motor();
	}

	//if final coordinate is smaller then turn left and move forward to the coordinate
	else{
		quarter_turns(1,1);
		while(check_distance()> x_f){
			go_forward();
		}
		quarter_turns(1,-1);
		stop_motor();
	}
}

//moves the robot from one point to another
void go_from_to(int x_i, int y_i, int x_f, int y_f){

	//moves according the y coordinates
	go_y(y_i,y_f);

	//moves according the x coordinates
	go_x(x_i,x_f);

}

void detection_dun_mur(void){
	if(VL53L0X_get_dist_mm() < 600) {
		detection_mur = MUR_DETECTER;
	}
}


static THD_WORKING_AREA(waMouvement, 256);
static THD_FUNCTION(Mouvement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	if(programme_fini){
			//Regarde si on détecte un mur, si oui, on commence le programme
			while(detection_mur){
				detection_dun_mur();
			}
			//on le met perpendiculaire au mur
			calibration_angle();
			//chprintf((BaseSequentialStream *)&SD3, "HELLO");
			//Déterminer axe des y le plus long
			determine_x_y_axis();
			//perpendiculaire();
			placement_corner();
			//aller de l'origine jusqu'au premier point
			chBSemSignal(&mvtplay_sem);
			chBSemWait(&sendasound_sem);
			go_from_to(0,0,tab_point[0].x,tab_point[0].y);
			while(pos_tab < SIZE_TAB-1){
				//Début partie Microphone
				chBSemSignal(&mvtplay_sem);
				chBSemWait(&sendasound_sem);
				//fin partie microphone
				go_from_to(tab_point[pos_tab].x, tab_point[pos_tab].y, tab_point[pos_tab+1].x, tab_point[pos_tab+1].y);
				pos_tab++;
			}
			chBSemSignal(&mvtplay_sem);
			chBSemWait(&sendasound_sem);
			programme_fini = 0;
			stop_motor();
    	}
    }
}

//static THD_WORKING_AREA(waMicrophone, 256);
//static THD_FUNCTION(Microphone, arg) {
//
//    chRegSetThreadName(__FUNCTION__);
//    (void)arg;
//
//    while(1){
//    	if(microphone_begin) chBSemWait(&sendamusic_sem);
//    	microphone_begin = 0;
//    	//Commence à écouter s'il y a un son ou non
//    	mic_start(&processAudioData);
//   		//Si on a reçu le signal pour redémarrer, on reprend la thread mouvement là où on l'a laissé
//   		if(get_ok()){
//   			microphone_begin = 1;
//   			chBSemSignal(&sendasound_sem);
//   		}
//    }
//}

static THD_WORKING_AREA(waMusic, 256);
static THD_FUNCTION(Music, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	chBSemWait(&mvtplay_sem);
//    	palTogglePad(GPIOD, GPIOD_LED7);
//		delay(3*ONE_SEC);
//		palTogglePad(GPIOD, GPIOD_LED7);
		if(tab_point[pos_tab].type == TARAUDAGE){
			playMelody(MARIO_START, ML_SIMPLE_PLAY, NULL);
			waitMelodyHasFinished();
			stopCurrentMelody();
		} else {
			playMelody(MARIO_DEATH, ML_SIMPLE_PLAY, NULL);
			waitMelodyHasFinished();
			stopCurrentMelody();
//			chBSemSignal(&sendamusic_sem);
		}
//		chBSemSignal(&sendamusic_sem);
		set_semamicro();
    }
}


void start_program(void){
	chThdCreateStatic(waMouvement, sizeof(waMouvement), NORMALPRIO, Mouvement, NULL);
//	chThdCreateStatic(waMicrophone, sizeof(waMicrophone), NORMALPRIO, Microphone, NULL);
	chThdCreateStatic(waMusic, sizeof(waMusic), NORMALPRIO, Music, NULL);
}

void set_semamvt(void){
	chBSemSignal(&sendasound_sem);
}

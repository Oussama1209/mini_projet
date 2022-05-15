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

typedef enum {
	two_mm=0,
	four_mm,
	six_mm,
	eight_mm
};

//Semaphore
static BSEMAPHORE_DECL(sendasound_sem, TRUE);

#define ONE_SEC 			SystemCoreClock/16
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
//Dimension de la planche
#define distance_x			320		//largeur in mm
#define	distance_y			500		//longueur in mm
#define SIDES				4		//number of sides of the board

#define CALIBRATION_FINI	0
#define CALIBRATION_EN_COURS	1
#define MUR_DETECTER		0
#define AUCUN_MUR			1
#define SIZE_TAB			6		//taille du tableau
#define RIGHT				-1
#define	LEFT				1
#define TARAUDAGE			0
#define PERCAGE				1

static uint16_t avg_distance=0;
static uint16_t distance_min=2000;
static int sides[SIDES];

static bool detection_mur = AUCUN_MUR;
static bool detection_calibration = CALIBRATION_EN_COURS;
static uint8_t pos_tab = 0;
static bool programme_fini = 1;

//structure
struct Mypoint {
	uint16_t x; //coordonnée en x en mm
	uint16_t y; //coordonnée en y en mm
	uint8_t diametre; //diamètre en mm
	uint8_t type; //type de trou : perçage = p, taraudage = t
};

static struct Mypoint tab_point[SIZE_TAB] = {{50, 50, two_mm, TARAUDAGE}, {100, 100, four_mm, PERCAGE},
											{200, 200, six_mm, TARAUDAGE}, {280, 250, eight_mm, PERCAGE},
											{200, 300, 6, TARAUDAGE}, {100, 100, eight_mm, PERCAGE}};

// Simple delay function
void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
}

uint8_t get_diametre(void){
	return tab_point[pos_tab].diametre;
}


//determine the y and x axis on the board
void determine_x_y_axis(void){

	//turn quarter turns and calculates distance to each side from the robot
	for(int i=0;i<SIDES;i++){
		sides[i]=VL53L0X_get_dist_mm();
		quarter_turns(1,LEFT);
		delay(ONE_SEC);
	}

	//addition of both distances to calculate the longueur and largeur
	uint16_t dist_prov_1=sides[0]+sides[2];
	uint16_t dist_prov_2=sides[1]+sides[3];

	//compare both values
	//y axis is always the bigger, it's a convention to avoid problems reading the coordinates
	if(dist_prov_1<dist_prov_2) quarter_turns(1,LEFT);

}

//places the robot in the corner of the board
void placement_corner(void){

	while(VL53L0X_get_dist_mm()>30){
		go_forward();
	}
	quarter_turns(1,RIGHT);
	while(VL53L0X_get_dist_mm()>30){
		go_forward();
	}
	quarter_turns(1,RIGHT);
	stop_motor();

}

//moves the robot according to the initial and arrival coordinates of y axis
//because its in mm it's okey to have int instead of float
void go_y(uint16_t y_i, uint16_t y_f){

	//compares value of initial and final coordinates
	int16_t compare = y_f - y_i;

	//if final coordinate is bigger then move forward to the coordinate
	if (compare>0) {
		while(VL53L0X_get_dist_mm()>distance_y-y_f){
			go_forward();
		}
		stop_motor();
	}

	//if final coordinate is smaller then turn back and go to the coordinate
	else{
		quarter_turns(2,RIGHT);
		while(VL53L0X_get_dist_mm()> y_f){
			go_forward();
		}
		quarter_turns(2,LEFT);
		stop_motor();
	}
}

//moves the robot according to the initial and arrival coordinates of x axis
//because its in mm it's okay to have int instead of float
void go_x(uint16_t x_i, uint16_t x_f){

	//compares value of initial and final coordinates
	int16_t compare = x_f - x_i;

	//if final coordinate is bigger then turn right and move forward to the coordinate
	if (compare>0) {
		quarter_turns(1,RIGHT);
		while(VL53L0X_get_dist_mm()>distance_x-x_f){
			go_forward();
		}
		quarter_turns(1,LEFT);
		stop_motor();
	}

	//if final coordinate is smaller then turn left and move forward to the coordinate
	else{
		quarter_turns(1,LEFT);
		while(VL53L0X_get_dist_mm()> x_f){
			go_forward();
		}
		quarter_turns(1,RIGHT);
		stop_motor();
	}

}

//moves the robot from one point to another
void go_from_to(uint16_t x_i, uint16_t y_i, uint16_t x_f, uint16_t y_f){

	//moves according the y coordinates
	go_y(y_i,y_f);

	//moves according the x coordinates
	go_x(x_i,x_f);

}

//detect a wall
//mainly used to know when to start the program
void detection_dun_mur(void){

	if(VL53L0X_get_dist_mm() < 2000) {
		detection_mur = MUR_DETECTER;
	}

}

//sets the robot on a perpendicular line to the side that the user faced it towards
static THD_WORKING_AREA(waCalibration, 256);
static THD_FUNCTION(Calibration, arg) {

    VL53L0X_start();

//	while(1){
	    if(detection_calibration){


			//Regarde si on détecte un mur, si oui, on commence le programme
			while(detection_mur){
				detection_dun_mur();
			}

			chRegSetThreadName(__FUNCTION__);
			(void)arg;


			int8_t direction;
			uint8_t calibration_check=0;
			uint8_t min=0;

			// approaches the side that the user puts it in front of
			while(VL53L0X_get_dist_mm()>100){
				go_forward();
			}
			stop_motor();
			bool turn_right=false;
			delay(ONE_SEC);
			int test_1=VL53L0X_get_dist_mm();
			delay(ONE_SEC);
			nieme_turn(20,RIGHT);
			delay(ONE_SEC);
			int test_2=VL53L0X_get_dist_mm();
			delay(ONE_SEC);
			nieme_turn(20,RIGHT);
			delay(ONE_SEC);
			int test_3=VL53L0X_get_dist_mm();
			delay(ONE_SEC);
			turn_right=(test_2<test_3)&&(test_3>test_1);
			if(turn_right) {
				direction=RIGHT;
				nieme_turn(5,LEFT);
			}
			else{
				direction=LEFT;
			}

			//makes small turns (1/200 turn) and compares the distance it receives with the ones before
			//if the distance gets lower and goes higher after then it means it was perpendicular to the side it faces
			while(calibration_check<8){

				delay(ONE_SEC/4);
				avg_distance=VL53L0X_get_dist_mm();
				delay(ONE_SEC/4);

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
				nieme_turn(200,direction);
			}

			//returns back to the perpendicular line
			for(int i=0;i<7;i++) {
				delay(ONE_SEC/2);
				nieme_turn(200,-direction);
			}
			detection_calibration=CALIBRATION_FINI;
		}
//	}
}

static THD_WORKING_AREA(waMouvement, 256);
static THD_FUNCTION(Mouvement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    VL53L0X_start();



    while(1){
    	if(programme_fini){
			//Regarde si on détecte un mur, si oui, on commence le programme
//			while(detection_mur){
//				detection_dun_mur();
//			}
			//on le met perpendiculaire au mur
//			calibration_angle();
			delay(ONE_SEC);
//			delay(ONE_SEC*20);
			//chprintf((BaseSequentialStream *)&SD3, "HELLO");
			//Déterminer axe des y le plus long
			determine_x_y_axis();
			//perpendiculaire();
			placement_corner();
			//aller de l'origine jusqu'au premier point
			set_semamvtplay();
			chBSemWait(&sendasound_sem);
			go_from_to(0,0,tab_point[0].x,tab_point[0].y);
			while(pos_tab < SIZE_TAB-1){
				//Début partie Microphone
				set_semamvtplay();
				chBSemWait(&sendasound_sem);
				//fin partie microphone
				go_from_to(tab_point[pos_tab].x, tab_point[pos_tab].y, tab_point[pos_tab+1].x, tab_point[pos_tab+1].y);
				pos_tab++;
			}
			set_semamvtplay();
			chBSemWait(&sendasound_sem);
			programme_fini = 0;
			stop_motor();
    	}
    }

}

void start_program(void){
	chThdCreateStatic(waCalibration, sizeof(waCalibration), NORMALPRIO + 1, Calibration, NULL);
	chThdCreateStatic(waMouvement, sizeof(waMouvement), NORMALPRIO, Mouvement, NULL);
}

void set_semamvt(void){
	chBSemSignal(&sendasound_sem);
}

uint8_t get_pos_tab(void){
	return pos_tab;
}

uint8_t get_tab_point(void){
	return tab_point[pos_tab].type;
}

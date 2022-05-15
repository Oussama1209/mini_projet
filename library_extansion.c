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
#include <musique.h>

typedef enum {
	two_mm=0,
	four_mm,
	six_mm,
	eight_mm,
	no_mm
};

//Sémaphore
static BSEMAPHORE_DECL(sendasound_sem, TRUE);

//Les define
#define ONE_SEC 			SystemCoreClock/16
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
//Dimension de la planche
#define distance_x			320		//largeur in mm
#define	distance_y			500		//longueur in mm
#define SIDES				4		//number of sides of the board

//Pour la calibration de l'angle du robot
#define CALIBRATION_FINI	0
#define CALIBRATION_EN_COURS	1
#define MUR_DETECTER		0
#define AUCUN_MUR			1
#define RIGHT				-1
#define	LEFT				1

//Pour les informations du tableau de donnée de la position des trous
#define SIZE_TAB			7		//taille du tableau
#define TARAUDAGE			0
#define PERCAGE				1
#define PREMIER_TROU		2

//Variable
static uint16_t avg_distance=0;
static uint16_t distance_min=2000;
static int sides[SIDES];			//Pour déterminer l'axe x et y
static bool detection_mur = AUCUN_MUR;
static bool detection_calibration = CALIBRATION_EN_COURS;	//Pour savoir si on a déjà calibré ou non
static uint8_t pos_tab = 0;		//Pour savoir où on est dans le tableau de donnée
static bool programme_fini = 1;		//Pour savoir si on n'a fini le programme

//structure
struct Mypoint {
	uint16_t x; //coordonnée en x en mm
	uint16_t y; //coordonnée en y en mm
	uint8_t diametre; //diamètre en mm
	uint8_t type; //type de trou : perçage = p, taraudage = t
};

//Tableau des données de la position des trous, de leur type et de leur dimension
static struct Mypoint tab_point[SIZE_TAB] = {{0, 0, no_mm ,PREMIER_TROU}, {50, 50, two_mm, TARAUDAGE}, {100, 100, four_mm, PERCAGE},
											{200, 200, six_mm, TARAUDAGE}, {280, 250, eight_mm, PERCAGE},
											{200, 300, six_mm, TARAUDAGE}, {100, 100, eight_mm, PERCAGE}};

// Simple delay function
void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
}

//Fonction pour allumer les leds en fontion du diamètre du trou
void LED_Toggle(void){

	switch(tab_point[pos_tab].diametre){
		//Si le trou est de 2mm, on allume 1 Led, s'il est de 4mm, on allume 2 leds
		//S'il est de 6mm, on allume 3 leds et s'il est de 8mm, on allume les 4 leds
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
		case no_mm:			//Si c'est le premier trou, on ne fait rien
			break;
		default:
			break;
	}
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
	//Si le robot mesure qu'il est à une distance plus petite que 20cm de quelque chose, alors il y a un mur
	if(VL53L0X_get_dist_mm() < 2000) {
		detection_mur = MUR_DETECTER;
	}
}
int8_t check_direction(void){
	int8_t direction=0;
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
	return direction;
}

//sets the robot on a perpendicular line to the side that the user faced it towards
static THD_WORKING_AREA(waCalibration, 256);
static THD_FUNCTION(Calibration, arg) {

    VL53L0X_start();

    //Initialisation des variables de la thread
	int8_t direction;
	uint8_t calibration_check=0;
	uint8_t min=0;

	while(1){
		//Si on n'a pas encore fait de calibration, alors on en fait une
	    if(detection_calibration){
			//Regarde si on détecte un mur, si oui, on commence le programme
			while(detection_mur){
				detection_dun_mur();
			}

			chRegSetThreadName(__FUNCTION__);
			(void)arg;

			// approaches the side that the user puts it in front of
			while(VL53L0X_get_dist_mm()>100){
				go_forward();
			}
			direction=check_direction();

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
	    chThdSleepMilliseconds(100);
	}
}

//Thread reponsible for the movement of the e-puck through the the points
static THD_WORKING_AREA(waMouvement, 256);
static THD_FUNCTION(Mouvement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    VL53L0X_start();

    while(1){
    	if(programme_fini){
			delay(ONE_SEC);
			//Déterminer axe des y le plus long
			determine_x_y_axis();
			//perpendiculaire();
			placement_corner();

			//Dit à la musique qu'elle peut lancer le microphone (car c'est le premier trou)
			set_semamvtplay();
			//Attend que microphone entende 984Hz
			chBSemWait(&sendasound_sem);

			//Parcours le tableau pour aller de points en points
			while(pos_tab < SIZE_TAB-1){
				go_from_to(tab_point[pos_tab].x, tab_point[pos_tab].y, tab_point[pos_tab+1].x, tab_point[pos_tab+1].y);
				pos_tab++;
				//Allume les leds qui détermine la taille du trou
				LED_Toggle();
				//Dit à musique qu'elle peut se lancer
				set_semamvtplay();
				//Attend que microphone entende une certaine fréquence
				chBSemWait(&sendasound_sem);
				//éteinds les leds
				LED_Toggle();
			}
			programme_fini = 0;
			stop_motor();
    	}
    }

}

//Crée la thread callibration et mouvement
void start_program(void){
	chThdCreateStatic(waCalibration, sizeof(waCalibration), NORMALPRIO + 1, Calibration, NULL);
	chThdCreateStatic(waMouvement, sizeof(waMouvement), NORMALPRIO, Mouvement, NULL);
}

//Envoie le signal de la sémaphore sendasound
void set_semamvt(void){
	chBSemSignal(&sendasound_sem);
}

//Renvoie le type du trou sur lequel est le robot
uint8_t get_tab_point(void){
	return tab_point[pos_tab].type;
}

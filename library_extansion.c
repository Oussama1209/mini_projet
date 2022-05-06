#include "ch.h"
#include "hal.h"
#include "motors.h"
#include <chprintf.h>
#include <sensors/VL53L0X/VL53L0X.h>

#define SPEED_MOTOR   		600 // speed of motor
#define NSTEP_ONE_TURN      1000 // number of steps for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13 // [cm]
#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
//Dimension de la planche
#define distance_x				400		//largeur in mm
#define	distance_y				400		//longueur in mm
#define SAMPLE_SIZE				5000	//number of samples used
#define SIDES					4		//number of sides of the board
#define POINTS					8		//number of points to determine on the board

static int avg_distance=0;
static int distance_min=8000;
static int tab_angle[SAMPLE_SIZE];
static int sides[SIDES];
static int Points_X[POINTS];
static int Points_Y[POINTS];


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
//Tourner d'un n i�me tour
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
	wait(5000000); //mettre la fonction Thdsleepmilisecond...
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
void calibration_angle(int direction){

	int calibration_check=0;
	int min=0;

	// approaches the side that the user puts it in front of
	while(VL53L0X_get_dist_mm()>150){
		go_forward();
	}

	//makes small turns (1/200 turn) and compares the distance it receives with the ones before
	//if the distance gets lower and goes higher after then it means it was perpendicular to the side it faces
	while(calibration_check<8){

		//takes average of the vales of distance the TOF receives
		for(int i=0;i<SAMPLE_SIZE;i++){
			tab_angle[i] = VL53L0X_get_dist_mm();
			wait(500);
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
	}

	//returns back to the perpendicular line
	for(int i=0;i<6;i++) nieme_turn(200,1);

	//sets the values to their original ones for another calibration
	distance_min=8000;
	avg_distance=0;
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

//est-ce que struct est bien d'implementer dans le projet?
void go_through_points(void){
	for(int i=1;i<POINTS;i++){
		go_from_to(Points_X[i-1],Points_Y[i-1],Points_X[i],Points_Y[i]);
	}
}




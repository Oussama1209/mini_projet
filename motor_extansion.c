#include "ch.h"
#include "chbsem.h"
#include "hal.h"
#include "motors.h"
#include "motor_extansion.h"
#include <chprintf.h>


#define SPEED_MOTOR   		550 // speed of motor
#define NSTEP_ONE_TURN      1000 // number of steps for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13 // [cm]
#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define RIGHT				-1
#define	LEFT				1

//Initialize motor position
void init_position_motor(void){
	right_motor_set_pos(0);
	left_motor_set_pos(0);
}

//set moor speed to a certain speed
void set_motor_speed(uint16_t speed){
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
}

//Turn a number of quarter turns
void quarter_turns(uint8_t num_of_quarter_turns, int8_t direction){

	//initialize motor position
	init_position_motor();

	//invert rotation direction to turn
	left_motor_set_speed(-direction * SPEED_MOTOR);
	right_motor_set_speed(direction * SPEED_MOTOR);
	if (direction == LEFT){
		//Si moteur droite tourne positivement
		while(right_motor_get_pos() < num_of_quarter_turns*PERIMETER_EPUCK/4* NSTEP_ONE_TURN / WHEEL_PERIMETER);
	} else{
		//Si moteur gauche tourne positivement
		while(left_motor_get_pos() < num_of_quarter_turns*PERIMETER_EPUCK/4* NSTEP_ONE_TURN / WHEEL_PERIMETER);
	}
	set_motor_speed(0);

}

//Turn 1/n turn
void nieme_turn(uint8_t nieme_value, int8_t direction){
	init_position_motor();
	left_motor_set_speed(-direction * SPEED_MOTOR);
	right_motor_set_speed(direction * SPEED_MOTOR);
	if (direction == LEFT){
		while(right_motor_get_pos() < PERIMETER_EPUCK/nieme_value* NSTEP_ONE_TURN / WHEEL_PERIMETER){}
	}else {
		while(left_motor_get_pos() < PERIMETER_EPUCK/nieme_value* NSTEP_ONE_TURN / WHEEL_PERIMETER){}
	}
	set_motor_speed(0);
}

//sets the motors to go forward
void go_forward(void){
	right_motor_set_speed(SPEED_MOTOR);
	left_motor_set_speed(SPEED_MOTOR);
}

//stops the motor
void stop_motor(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);

}

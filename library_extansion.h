#ifndef MOTORS_LIB_H_
#define MOTORS_LIB_H_

#define LED1     	GPIOD, 5
#define LED3     	GPIOD, 6
#define LED5     	GPIOD, 10
#define LED7     	GPIOD, 11
#define FRONT_LED	GPIOD, 14
#define BODY_LED	GPIOB, 2

struct Mypoint;

// Simple delay function
void delay(unsigned int n);

void start_program(void);
void start_music(bool ok_microphone);

// Initialize right motor position to 0
//IMPORTANT TO CALL BEFORE THE WHILE LOOP
void init_position_motor(void);

//set speed of left and right motor
void set_motor_speed(uint16_t speed);

//turns num_of_quarter_turns * 90 degrees
void quarter_turns(uint8_t num_of_quarter_turns, int8_t direction);

//turns nieme of a turn
void nieme_turn(uint8_t nieme_value, int8_t direction);

//the TOF waits a certain time before calculating distance
int check_distance(void);

//sets the motors to go forward
void go_forward(void);

//stops the motor
void stop_motor(void);

//sets the robot on a perpendicular line to the side that the user faced it towards
void calibration_angle(void);

//Mettre le robot perpendiculaire à un bord
void perpendiculaire(void);

//determine the y and x axis on the board
void determine_x_y_axis(void);

//places the robot in the corner of the board
void placement_corner(void);

//moves the robot according to the initial and arrival coordinates of y axis
//because its in mm it's okey to have int instead of float
void go_y(int y_i, int y_f);

//moves the robot according to the initial and arrival coordinates of x axis
//because its in mm it's okay to have int instead of float
void go_x(int x_i, int x_f);

//moves the robot from one point to another
void go_from_to(int x_i, int y_i, int x_f, int y_f);



#endif /* MOTORS_LIB_H  */


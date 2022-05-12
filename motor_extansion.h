#ifndef MOTOR_EXTANSION_H_
#define MOTORS_EXTANSION_H_

// Initialize right and left motor position to
void init_position_motor(void);

//set speed of left and right motor
void set_motor_speed(uint16_t speed);

//sets the motors to go forward
void go_forward(void);

//stops the motor
void stop_motor(void);

//turns num_of_quarter_turns * 90 degrees
void quarter_turns(uint8_t num_of_quarter_turns, int8_t direction);

//turns nieme of a turn
void nieme_turn(uint8_t nieme_value, int8_t direction);

#endif /* MOTORS_LIB_H  */

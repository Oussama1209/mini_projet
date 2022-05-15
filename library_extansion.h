#ifndef LIBRARY_EXTANSION_H_
#define LIBRARY_EXTANSION_H_

#define LED1     	GPIOD, 5
#define LED3     	GPIOD, 6
#define LED5     	GPIOD, 10
#define LED7     	GPIOD, 11
#define FRONT_LED	GPIOD, 14
#define BODY_LED	GPIOB, 2

struct Mypoint;

// Simple delay function
void delay(unsigned int n);

//Fonction pour allumer les leds en fontion du diamètre du trou
void LED_Toggle(void);

//determine the y and x axis on the board
void determine_x_y_axis(void);

//places the robot in the corner of the board
void placement_corner(void);

//moves the robot according to the initial and arrival coordinates of y axis
//because its in mm it's okey to have int instead of float
void go_y(uint16_t y_i, uint16_t y_f);

//moves the robot according to the initial and arrival coordinates of x axis
//because its in mm it's okay to have int instead of float
void go_x(uint16_t x_i, uint16_t x_f);

//moves the robot from one point to another
void go_from_to(uint16_t x_i, uint16_t y_i, uint16_t x_f, uint16_t y_f);

//Détecte s'il y a un mur à moins de 20cm
void detection_dun_mur(void);

//Création de la thread mouvement
void start_program(void);

//Envoie le signal de la sémaphore sendasound
void set_semamvt(void);

//Renvoie le type du trou sur lequel est le robot
uint8_t get_tab_point(void);

#endif /* LIBRARY_EXTANSION_H  */


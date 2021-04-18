//Arduino code for a pseudo P loop by measuring the motor RPM, developed from the motor car code
//Compares the number of ticks to match the speed
//Jeremy Mang
//25-03-2021

#include <PID_v1.h>

//Important constants
int motor_pins[] = {10, 9, 8, 7, 6, 5};
#define NUM_OF_MOTOR_PIN 5
#define ENA 10
#define IN1 9
#define IN2 8
#define ENB 5
#define IN3 7
#define IN4 6
#define MOTOR_SPEED 220

#define MOTOR_A_ENCODER 2
#define MOTOR_B_ENCODER 3
#define ENCODER_TICKS 20

//PID loop constants
#define TIME_STEP 20 //Tells the Uno to sample and adjust the motor speed in milliseconds
#define MOTOR_OFFSET 5  

#define MAX_PWM 255
#define MIN_PWM 0

//IR-line following (IRLF) pins A0 - A5
//Front IR line followers
#define IR_LF_I A0 //OUT1 Prev A10, A11, A12, A13, A14
#define IR_LF_A A1 //OUT2 
#define IR_LF_B A2 //OUT3
#define IR_LF_C A3 //OUT4
#define IR_LF_D A4 //OUT5
#define SPEED_REDUCE 200
//Tracks the ticks
volatile unsigned long enc_l = 0;
volatile unsigned long enc_r = 0;
#define TIME_STEP 20
char direction = 'f';

//Exponential smoothing constants
double prev_term = 0;
#define ALPHA 0.1

//Function prototypes
//Initialisers
void init_motor();
void init_encoder();
void init_IR_follower();
//Motor directions
void motor_direction(char c);
void forward();
void reverse();
void left();
void right();
void halt();
void adjust_trajectory();
//PID control
void count_left();
void count_right();

void setup() {
    Serial.begin(9600);
    //Initialise systems
    init_motor();
    init_encoder();
    init_IR_follower();
    
    //Setup interrupts
    attachInterrupt(digitalPinToInterrupt(MOTOR_A_ENCODER), count_left, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_B_ENCODER), count_right, CHANGE);
    Serial.println("Motor PID loop test via encoder frequency:\n");
    delay(1000);
}

void loop() {

    unsigned long num_ticks_l;
    unsigned long num_ticks_r;

    int power_l = MOTOR_SPEED;
    int power_r = MOTOR_SPEED;
    /*
    // Set initial motor power
    if (direction == 'f') {
        power_l = MOTOR_SPEED;
        power_r = MOTOR_SPEED - 70;
    }
    else if (direction == 'b') {
        power_l = MOTOR_SPEED;
        power_r = MOTOR_SPEED;
    }
    */
    
    //Determine the turn to adjust trajectory
    unsigned long diff_l;
    unsigned long diff_r;
    
    // Reset encoder counts
    enc_l = 0;
    enc_r = 0;
    
    // Remember previous encoder counts
    unsigned long enc_l_prev = enc_l;
    unsigned long enc_r_prev = enc_r;

    unsigned long target_count = 1000;

    while ( (enc_l < target_count) && (enc_r < target_count) ) {

        
        
        // Sample number of encoder ticks
        num_ticks_l = enc_l;
        num_ticks_r = enc_r;

        //Drive
        drive(power_l, power_r);
        
        // Print out current number of ticks
        Serial.print(num_ticks_l);
        Serial.print("\t");
        Serial.println(num_ticks_r);

        // Number of ticks counted since last time
        //Proportional
        diff_l = num_ticks_l - enc_l_prev;
        diff_r = num_ticks_r - enc_r_prev;
    
        // Store current tick counter for next time
        enc_l_prev = num_ticks_l;
        enc_r_prev = num_ticks_r;

        // If left is faster, slow it down and speed up right
        //output
        if ( diff_l > diff_r ) {
            power_l -= MOTOR_OFFSET;
            power_r += MOTOR_OFFSET;
        }
        
        // If right is faster, slow it down and speed up left
        if ( diff_l < diff_r ) {
            power_l += MOTOR_OFFSET;
            power_r -= MOTOR_OFFSET;
        }
        
        adjust_trajectory();

        delay(TIME_STEP);
    }

    //End program
    while(true){
        halt();
    }
    
}
//////////////////////////////////////////////////////////////////////////////////////
//Functions here
//Initialise IR line followers for trajectory control
void init_IR_follower() {
	//Front IR_LF's
	pinMode(IR_LF_A, INPUT);
	pinMode(IR_LF_B, INPUT);
	pinMode(IR_LF_C, INPUT);
	//Rear IR_LF's
	pinMode(IR_LF_D, INPUT);
//	pinMode(IR_LF_E, INPUT);
}

//Correct the trajectory of the vehicle if it veers away from the path
//Refer to sensor diagram
void adjust_trajectory() {
    //During moving forwards
	if (direction == 'f') {
		//Adjust to the right lightly, if it goes to the left
		//Manual adjustment
		if (digitalRead(IR_LF_I) && digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && !digitalRead(IR_LF_C) && digitalRead(IR_LF_D)) {
            
			analogWrite(ENA, MAX_PWM - SPEED_REDUCE - 20);
            //left();
            Serial.println("turn left");
			delay(100*TIME_STEP);
            //forward();
		}
        //HARD RIGHT
        if (digitalRead(IR_LF_I) && digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && digitalRead(IR_LF_C) && !digitalRead(IR_LF_D)) {
            
            analogWrite(ENA, MAX_PWM - SPEED_REDUCE - 40);
            //left();
            Serial.println("turn left");
            delay(100*TIME_STEP);
            //forward();
        }
		//Adjust to the left, if it veers to the right
		if (digitalRead(IR_LF_I) && !digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && digitalRead(IR_LF_C) && digitalRead(IR_LF_D)) {
			analogWrite(ENB, MAX_PWM - SPEED_REDUCE);
            //right();
            Serial.println("turn right");
			delay(50*TIME_STEP);
            //forward();
        }
        //HARD LEFT
        if (!digitalRead(IR_LF_I) && digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && digitalRead(IR_LF_C) && digitalRead(IR_LF_D)) {
            analogWrite(ENB, MAX_PWM - SPEED_REDUCE - 20);
            //right();
            Serial.println("turn right");
            delay(100*TIME_STEP);
            //forward();
        }

	}
	else if (direction == 'b') {
		//During reverse
        //Adjust right
		if (digitalRead(IR_LF_I) && !digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && digitalRead(IR_LF_C) && digitalRead(IR_LF_D)) {
            
            //analogWrite(ENB, MAX_PWM - SPEED_REDUCE);
            right();
            Serial.println("turn left");
            delay(5 * TIME_STEP);
            //forward();
        }
        //HARD RIGHT
        if (!digitalRead(IR_LF_I) && digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && digitalRead(IR_LF_C) && digitalRead(IR_LF_D)) {
            analogWrite(ENB, MAX_PWM - SPEED_REDUCE);
            //Inverted controls
            Serial.println("turn left");
            delay(10*TIME_STEP);
        }
        //Adjust to the left, if it veers to the right
        if (digitalRead(IR_LF_I) && digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && !digitalRead(IR_LF_C) && digitalRead(IR_LF_D)) {
            analogWrite(ENA, MAX_PWM - SPEED_REDUCE);
            //right();
            Serial.println("turn right");
            delay(5 * TIME_STEP);
            //forward();
        }
        //HARD LEFT
        if (digitalRead(IR_LF_I) && !digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && digitalRead(IR_LF_C) && !digitalRead(IR_LF_D)) {
            analogWrite(ENA, MAX_PWM - SPEED_REDUCE - 20);
            //right();
            Serial.println("turn right");
            delay(10 * TIME_STEP);
            //forward();
        }
	}
}

//Return the number of ticks of the left or right motor
void count_left() {
    enc_l +=1;
}
void count_right() {
    enc_r +=1;
}


//Control the motor direction
void motor_direction(char c) {
        
        //Move Forward
        if (c == 'f') {
            Serial.print("Moving forwards\n");
            forward();
        }
        //Rotate right
        else if (c == 'r') {
            Serial.print("Rotating right\n");
            right();
        }
        //Rotate left
        else if (c == 'l') {
            Serial.print("Rotating left\n");
            left();
        }
        //Reverse
        else if (c == 'b') {
            Serial.print("Reversing\n");
            reverse();
        }
        //Stop
        else if (c == 's') {
            Serial.print("Stop\n");
            halt();
        }
}
//Motor directions here:
void forward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}
void reverse() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}
void left() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}
void right() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}
void halt() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

//Drive and adjust the car's speed
void drive(int power_a, int power_b) {
    // Constrain power to between -255 and 255
    power_a = constrain(power_a, -255, 255);
    power_b = constrain(power_b, -255, 255);

    if (direction == 'f') {
        forward();
    }
    else {
        reverse();
    }
    

    //adjust the speed of the motors
    analogWrite(ENA, power_a);
    analogWrite(ENB, power_b);
}


//Initialise motor pins
void init_motor() {
    //Set the all motor pins to be output
    for (int i = 0; i < NUM_OF_MOTOR_PIN; i +=1) {
        pinMode(motor_pins[i], OUTPUT);
    }
}
//Initialise encoder pins
void init_encoder() {
    //Set pin 2 and 3 as input
    pinMode(MOTOR_A_ENCODER, INPUT);
    pinMode(MOTOR_B_ENCODER, INPUT);
}

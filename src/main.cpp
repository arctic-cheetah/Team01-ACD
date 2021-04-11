//The prototype code to achieve team01's goals in the ACD project
//Jeremy Mang
//25-03-2021

#include <Arduino.h>
#include <PID_v1.h>

//Important constants
//Motor pins
int motor_pins[] = {10, 9, 8, 7, 6, 5};
#define NUM_OF_MOTOR_PIN 5
#define ENA 10
#define IN1 9
#define IN2 8
#define ENB 5
#define IN3 7
#define IN4 6
#define MOTOR_SPEED 255
#define MOTOR_A_ENCODER 2
#define MOTOR_B_ENCODER 3
#define ENCODER_TICKS 20
#define MOTOR_BIAS 80

//direction codes: f == forward, r == reverse, s == stop
char direction;

//Unloading motor pins
//controls speed
#define UNLOADING_MOTOR_EN 12
//controls direction
#define UNLOADING_MOTOR_PHASE 31
#define UNLOAD_STEPS 20

//IR-line following (IRLF) pins A0 - A5
//Front IR line followers
#define IR_LF_A A10 //OUT1
#define IR_LF_B A11 //OUT2
#define IR_LF_C A12 //OUT3
#define IR_LF_D A13 //OUT4
#define IR_LF_E A14 //OUT5

//IR obstacle detection (IR_OD) pin 22-26
//Front 
#define IR_OD_1 22
#define IR_OD_2 23
//Rear
#define IR_OD_3 24
#define IR_OD_4 25
//Egg detection
#define IR_OD_5 26

//Ultrasonic obstacle detector (US_OD) pin 2,3 and 11,12
//Front US_OD
#define USOD_F_TRIG A6
#define USOD_F_ECHO A7
//Rear US_OD
#define USOD_R_TRIG A8
#define USOD_R_ECHO A9
//Stopping distance for the USOD
#define STOP_DIST 15.0

//IR receivers pins 27-28
//Left side
#define IR_REC_III 27
#define IR_REC_IV 28

//Red LED
#define RED_LED 32
//Green LED
#define GREEN_LED 33
//Reversing truck sound
#define REV_TRUCK_SND_PIN 34
#define INTERVAL 1000 //ms
#define SOUND_DURATION_ON 18000//ms
#define FREQUENCY 30 //Hz
int duration = 3; //Duration for off time arduino
int lastTime = 10; //Keep track of time for arduino reversing truck sound

//VCC pins
#define VCC1 A0
#define VCC2 A1
#define VCC3 A2

//PID loop constants
#define TIME_STEP 50 //Tells the Uno to sample and adjust the motor speed in milliseconds
#define MAX_PWM 230
#define MOTOR_OFFSET 5
//Tracks the ticks
volatile unsigned long enc_l = 0;
volatile unsigned long enc_r = 0;

unsigned long num_ticks_l;
unsigned long num_ticks_r;

// Set initial motor power
int power_l = MOTOR_SPEED;
int power_r = MOTOR_SPEED;

//Determine the turn to adjust trajectory
unsigned long diff_l;
unsigned long diff_r;



// Remember previous encoder counts
unsigned long enc_l_prev = enc_l;
unsigned long enc_r_prev = enc_r;



//Function prototypes
//Initialisers
void init_motor();
void init_unloading_motor();
void init_encoder();
void init_IR_follower();
void init_IR_OD();
void init_USOD();
void init_IR_receiver();
void init_red_green_light();
void init_reverse_truck_sound();
void init_vcc();

//Motor directions
void motor_direction(char c);
void adjust_trajectory(char direction);
void forward();
void reverse();
void left();
void right();
void halt();
//Obstacle detection
bool is_object_nearby();
bool is_egg_inside();
//Ultrasonic sensor
double distance_by_USOD(int trig_pin, int echo_pin);
//IR receiver system
bool do_begin_unloading(bool egg_is_inside);
bool has_received_ir_signal();

//Unloading system functions
void open_gate();
void close_gate();

//PID control
double encoder_frequency(int motor_encoder);
void drive(int power_a, int power_b);
void count_left();
void count_right();


//Red-green light system
void on_green_light();
void off_green_light();
void on_red_light();
void off_red_light();

//Truck reversing sound
void reversing_truck_sound();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main code here:
void setup() {
	Serial.begin(9600);
    //Motor control
	init_motor();
	init_encoder();
	init_IR_follower();
    //Obstacle detection
    init_IR_OD();
    init_USOD();
    //IR system, the receivers
    init_IR_receiver();
    
    init_red_green_light();
    init_reverse_truck_sound();
    init_vcc();
	Serial.println("All systems green!");
    delay(1000);
}

void loop() {


	//Remote control
	/*
	if (Serial.available()) {
		motor_direction(Serial.read());
	}
	*/
	/*
	Serial.println(distance_by_USOD(USOD_F_TRIG, USOD_F_ECHO));
	Serial.println("\\\\\\\\\\\\\\\\\\");
	Serial.println(distance_by_USOD(USOD_R_TRIG, USOD_R_ECHO));
	//direction = motor_direction();
	*/


	on_green_light();
	//Egg is inside, go to loading zone
	if (is_egg_inside()) {
		//move forwards
		direction = 'f';
		Serial.print("Forward\n");
		forward();
	}
	//No egg in the car, return to loading zone by reversing
	//In future, may implement turn around
	else {
		//Go backwards
		direction = 'b';
		reverse();
		Serial.print("Backward\n");
		reversing_truck_sound();
	}



	//Consider if this needs to be an interrupt
	//Implement obstacle detection here:

	//3a)Stop moving until the obstacle has passed
	//Show the red light
	while (is_object_nearby()) {
		Serial.print("Object detected!\n");
		off_green_light();
		halt();
		on_red_light();
		delay(5 * TIME_STEP);
	}
	off_red_light();

	//3b,c,d)
	//Implement trajectory control here:
	//Operates by briefly slowing down the faster motor 
	adjust_trajectory(direction);

	//Adjust speed via PID loop
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



	//4)Check if the IR signal is received
	while(do_begin_unloading(is_egg_inside()) );
	//Time step = 100ms
	//May need to adjust this because of other sensor delays
	delay(TIME_STEP);
	
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Function declarations here

//Initialise motor pins
void init_motor() {
    //Set the all motor pins to be output
    for (int i = 0; i < NUM_OF_MOTOR_PIN; i +=1) {
        pinMode(motor_pins[i], OUTPUT);
    }
    //Set the speed of the motors
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);

}
//Initialise the unloading motor
void init_unloading_motor() {
    pinMode(UNLOADING_MOTOR_EN, OUTPUT);
    pinMode(UNLOADING_MOTOR_PHASE, OUTPUT);
}

//Initialise encoder pins
void init_encoder() {
    //Set pin 2 and 3 as input
    pinMode(MOTOR_A_ENCODER, INPUT);
    pinMode(MOTOR_B_ENCODER, INPUT);

	//Setup interrupts
	
    attachInterrupt(digitalPinToInterrupt(MOTOR_A_ENCODER), count_left, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_B_ENCODER), count_right, CHANGE);
}
//Initialise IR line followers for trajectory control
void init_IR_follower() {
	//Front IR_LF's
	pinMode(IR_LF_A, INPUT);
	pinMode(IR_LF_B, INPUT);
	pinMode(IR_LF_C, INPUT);
	//Rear IR_LF's
	pinMode(IR_LF_D, INPUT);
	pinMode(IR_LF_E, INPUT);
}
//Initialise obstacle detection
void init_IR_OD() {
    pinMode(IR_OD_1, INPUT);
    pinMode(IR_OD_2, INPUT);
    pinMode(IR_OD_3, INPUT);
    pinMode(IR_OD_4, INPUT);
    pinMode(IR_OD_5, INPUT);
}
//Initialise Ultrasonic obstacle detectors
void init_USOD() {
    //Front
    pinMode(USOD_F_ECHO, INPUT);
    pinMode(USOD_F_TRIG, OUTPUT);
    //Rear
    pinMode(USOD_R_ECHO, INPUT);
    pinMode(USOD_R_TRIG, OUTPUT);
}
//Initialise IR receivers
void init_IR_receiver() {
    //Left side
    pinMode(IR_REC_III, INPUT);
    pinMode(IR_REC_IV, INPUT);
}

//Initialise LED's
void init_red_green_light() {
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
}
//Initialise reversing truck sound
void init_reverse_truck_sound() {
    pinMode(REV_TRUCK_SND_PIN, OUTPUT);
}

//Initialise VCC pins
void init_vcc() {
    pinMode(VCC1, OUTPUT);
    digitalWrite(VCC1, HIGH);
    pinMode(VCC2, OUTPUT);
    digitalWrite(VCC2, HIGH);
    pinMode(VCC3, OUTPUT);
    digitalWrite(VCC3, HIGH);
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
        //Open gate
        else if (c == 'o') {
            Serial.print("Open gate\n");
            open_gate();
            
        }
        //Close gate
        else if (c == 'c') {
            Serial.print("Close gate\n");
            close_gate();
        }
}
//Correct the trajectory of the vehicle if it veers away from the path
//Refer to sensor diagram
void adjust_trajectory(char direction) {
    //During moving forwards
	if (direction == 'f') {
		//Adjust to the left lightly, if it goes to the right
		//Manual adjustment
		if (digitalRead(IR_LF_A) && !digitalRead(IR_LF_B) && !digitalRead(IR_LF_C)) {
            
			analogWrite(ENA, MAX_PWM - 30);
            Serial.println("turn left");
			delay(TIME_STEP);
            //forward();
		}
		//Adjust to the right, if it veers to the left
		else if (!digitalRead(IR_LF_A) && !digitalRead(IR_LF_B) && digitalRead(IR_LF_C)) {
			analogWrite(ENB, MAX_PWM - 30);
            Serial.println("turn right");
			delay(TIME_STEP);
            //forward();
        }

	}
	else if (direction == 'b') {
		//During reverse
		//Adjust to the right, if it goes to the left
		if (digitalRead(IR_LF_C)) {
			analogWrite(ENA, MAX_PWM);
			delay(TIME_STEP);
		}
		//Adjust to the left, if it veers to the right
		if (digitalRead(IR_LF_A) && !digitalRead(IR_LF_D)) {
			analogWrite(ENB, MAX_PWM);
			delay(TIME_STEP);
		}
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

    forward();

    //adjust the speed of the motors
    analogWrite(ENA, power_a);
    analogWrite(ENB, power_b);
}


//Returns true if the IR obstacle detectors finds an object in front or the rear
//Otherwise false
//Stopping distance is set to 15.0
bool is_object_nearby() {
    //Fetch the distance from the Ultrasonic sensors
    double front_dist = distance_by_USOD(USOD_F_TRIG, USOD_F_ECHO);
    double rear_dist = distance_by_USOD(USOD_R_TRIG, USOD_R_ECHO);

    //Debugging
    /*
    Serial.print("USOD_F: ");
    Serial.println(front_dist);
    Serial.print("USOD_R: ");
    Serial.println(rear_dist);
    */
    bool object_in_front = false;
    bool object_in_rear = false;

    if (0 < front_dist && front_dist < STOP_DIST) {
        object_in_front = true;
    }
    if (0 < front_dist && rear_dist < STOP_DIST) {
        object_in_rear = true;
    }
    //Check and return if an object is nearby the vehicle
    //BEWARE SOMETIMES SENSORS INVERT THE SIGNAL
    return object_in_front || object_in_rear || !digitalRead(IR_OD_1) || !digitalRead(IR_OD_2) || !digitalRead(IR_OD_3) || !digitalRead(IR_OD_4);
}
//Returns true if the egg is inside the vehicle
//otherwise false
bool is_egg_inside() {
    return !digitalRead(IR_OD_5);
}

//Returns the distance in cm from the Ultrasonic obstacle detector
double distance_by_USOD(int trig_pin, int echo_pin) {

    unsigned long TimeOut = 20000; //In microseconds
    
    //Send 8 pulses at 40Khz for approx 10ms
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);
    //Make Echo_pin high and  Trigger pin Low


    //Wait for the echo pin to become high
    unsigned long endTime = micros() + long(1000);//ENSURE THIS IS a LONG
    while (micros() < endTime && !digitalRead(echo_pin));

    //Begin timing for the echo pin
    unsigned long timeBegin = micros();
    endTime = timeBegin + TimeOut;
    while (micros() < endTime && digitalRead(echo_pin));

    //Calculate the distance
    double dist = double (micros() - timeBegin) * 34000 * (0.000001) / 2.0;
    
    //Print the results
    //Serial.print(dist);
    //Serial.print("cm \n");

    //return the result
    return dist;
}

//Return the number of ticks of the left or right motor
void count_left() {
    enc_l +=1;
}
void count_right() {
    enc_r +=1;
}



//Returns the frequency of the encoder signal of the motor and is
//related to rpm
double encoder_frequency(int motor_encoder) {

    unsigned long T_on = pulseIn(motor_encoder, HIGH);//in microseconds
    unsigned long T_off = pulseIn(motor_encoder, LOW);

    //No signal
    if (!T_on && !T_off) {
        return 0.0;
    }
    
    //Debugging
    //Serial.println(T_on);
    //Serial.println(T_off);

    //Find the time period of the wave = Ton + Toff (One tick)
    double T_period = double(T_on + T_off);
    //Serial.println(T_period);

    //Do not print inf
    if ( !isinf(T_period) || !isnan(T_period)) {
        double frequency = 1000000 / T_period; 
        Serial.println(frequency);
        return frequency;
    }
    //Serial.println(0.0);
    return 0.0;
}

//Returns true if the vehicle has arrived at the designated zone
//Otherwise false
bool do_begin_unloading(bool egg_is_inside) {
    if (has_received_ir_signal() && egg_is_inside) {
        //Align the vehicle's IR receivers with the IR transmitters
        //Since IR system hasn't been finalised. Only stop and unload the egg
        //Show the red light when stopped

        //0) Stop the car
        halt();
        //1)Show the red light
        off_green_light();
        on_red_light();
        //Implement unloading system
        //2)Open the gate till the egg rolls out
        open_gate();
        delay(2000);
        //3)Shut the gate
        close_gate();
    }
    else if (has_received_ir_signal() && !egg_is_inside) {
        //Align the vehicle's IR receivers with the IR transmitters. IR transmitters haven't been finalised
        //Wait for the egg to be loaded;
        //and show the red light when stopped

        //0)Stop the car 
        halt();
        //1)Show red light
        off_green_light();
        on_red_light();
        //2)Wait for the egg to be loaded
        while(!is_egg_inside());

    }
    off_red_light();
    return false;
}

//Returns true if one of the IR recievers detects a signal from the transmitters
bool has_received_ir_signal() {
    return !digitalRead(IR_REC_III) || !digitalRead(IR_REC_IV);
}

//Step down
void step_down () {
    analogWrite(UNLOADING_MOTOR_EN, MAX_PWM);
    digitalWrite(UNLOADING_MOTOR_PHASE, LOW);
    delay(50);
    //Turn off the motor
    analogWrite(UNLOADING_MOTOR_EN, 0);
    delay(200);
}

void step_up () {
    analogWrite(UNLOADING_MOTOR_EN, MAX_PWM);
    digitalWrite(UNLOADING_MOTOR_PHASE, HIGH);
    delay(50);
    //Turn off the motor
    analogWrite(UNLOADING_MOTOR_EN, 0);
    delay(200);
}

//Open the gate
void open_gate() {
    for (int i = 0; i < UNLOAD_STEPS; i +=1) {
        step_down();
    }
}
//Close the gate and turn off the motor
void close_gate() {
    for (int i = 0; i < UNLOAD_STEPS + 3; i +=1) {
        step_up();
    }
    
}

//Red-green light system
void on_green_light() {
    digitalWrite(GREEN_LED, HIGH);
}
void off_green_light() {
    digitalWrite(GREEN_LED, LOW);
}
void on_red_light() {
    digitalWrite(RED_LED, HIGH);
}
void off_red_light() {
    digitalWrite(RED_LED, LOW);
}

//Truck reversing sound
void reversing_truck_sound() {
    duration = millis();
    if (duration - lastTime > INTERVAL) {
        lastTime = duration;
        tone(REV_TRUCK_SND_PIN, FREQUENCY, SOUND_DURATION_ON);
    }
}

//The prototype code to achieve team01's goals in the ACD project
//Jeremy Mang
//25-03-2021

// alex's test push

#include <Arduino.h>
#include <Servo.h>

#define V2 1

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
#define MOTOR_SPEED 220
#define MOTOR_A_ENCODER 2
#define MOTOR_B_ENCODER 3
#define ENCODER_TICKS 20
#define MOTOR_BIAS 80
#define SPEED_REDUCE 200
//direction codes: f == forward, r == reverse, s == stop
char direction = 'f';


//Timing 
#define WAIT_EGG 100
#define WAIT_TO_BEGIN 1000
#define GET_OUT_OFF_IR_SIGNAL 500
#define UNLOAD_EGG 500


//Unloading motor pins
//controls speed
#define UNLOADING_MOTOR_EN 12
#define OPEN_GATE 0
#define CLOSE_GATE 90

//IR-line following (IRLF) pins A0 - A5
//Front IR line followers
#define IR_LF_I A0 //OUT1 Prev A10, A11, A12, A13, A14
#define IR_LF_A A1 //OUT2 
#define IR_LF_B A2 //OUT3
#define IR_LF_C A3 //OUT4
#define IR_LF_D A4 //OUT5

//Front IR line followers
#define IR_LF_1 A11//OUT1 
#define IR_LF_2 A12 //OUT2 
#define IR_LF_3 A13 //OUT3
#define IR_LF_4 A14 //OUT4
#define IR_LF_5 A15 //OUT5


//IR obstacle detection (IR_OD) pin 26
//Egg detection
#define IR_OD 26

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
#define IR_REC_I 27
//#define IR_REC_II 28

//Red LED
#define RED_LED 32
//Green LED
#define GREEN_LED 33
//Reversing truck sound
#define REV_TRUCK_SND_PIN 11
#define INTERVAL 1000 //ms
#define SOUND_DURATION_ON 18000//ms
#define FREQUENCY 30 //Hz
unsigned long duration = 0; //Duration for off time arduino
unsigned long lastTime = 0; //Keep track of time for arduino reversing truck sound

//PID loop constants
#define TIME_STEP 20 //Tells the Uno to sample and adjust the motor speed in milliseconds
#define MAX_PWM 230
#define MOTOR_OFFSET 5


//Tracks the ticks
volatile unsigned long left_encoder = 0;
volatile unsigned long right_encoder = 0;

unsigned long left_num_ticks;
unsigned long right_num_ticks;

// Set initial motor power
int left_power = MOTOR_SPEED;
int right_power = MOTOR_SPEED - 70;

//Determine the turn to adjust trajectory
unsigned long diff_left;
unsigned long diff_right;

//Remember previous encoder counts
unsigned long left_encoder_prev = left_encoder;
unsigned long right_encoder_prev = right_encoder;

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

//Motor directions
void command(char c);
void set_motor_speed(int speed);
void adjust_trajectory();
void forward();
void reverse();
void left();
void right();
void halt();
//Obstacle detection
void check_obstacle();
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
//Unloading motor object:
Servo unloading_motor;

//PID control (Technically only PI)
double encoder_frequency(int motor_encoder);
void drive(int power_a, int power_b);
void count_left();
void count_right();
void adjust_motor_speed();

//Red-green light system
void on_green_light();
void off_green_light();
void on_red_light();
void off_red_light();

//Truck reversing sound
void reversing_truck_sound_on();
void reversing_truck_sound_off();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main code here:
void setup() {
	Serial.begin(9600);
    Serial1.begin(9600);
    //Motor control
	init_motor();
    init_unloading_motor();
	init_encoder();
	init_IR_follower();
    //Obstacle detection
    init_IR_OD();
    init_USOD();
    //IR system, the receivers
    init_IR_receiver();
    
    //Human user interface
    init_red_green_light();
    init_reverse_truck_sound();

	Serial.println("All systems green!");
    delay(1000);

    //Move to pick up position
    //Set the motor speed to be low:
    set_motor_speed(MOTOR_SPEED - 120);
    Serial.print("Move to pickup location");
    on_green_light();
    while(!has_received_ir_signal()) {
        forward();
        direction = 'f';
        //adjust_motor_speed();
        adjust_trajectory();
        check_obstacle();
        delay(WAIT_EGG);
    }
    halt();
    //Make it fast again
    set_motor_speed(MOTOR_SPEED);
    //Wait for the egg to be deposited
    off_green_light();
    on_red_light();
    while (!is_egg_inside()) {
        Serial.print("Waiting for the egg to be deposited\n");
    }
    
    //Stop
    Serial.print("Egg received and IR signal accepted!\n");

    //Initiate
    forward();
    direction = 'f';
    //adjust_motor_speed();
    adjust_trajectory();
    delay(TIME_STEP * 25);
}

void loop() {

	//Remote control
	if (Serial.available()) {
		command(Serial.read());
	}
    
    off_red_light();
    on_green_light();
	
	//Egg is inside, go to loading zone
	if (is_egg_inside()) {
		//move forwards
        
		direction = 'f';	
        Serial.print("Forward\n");
		forward();
	}
	//No egg in the car, return to loading zone by reversing
	else {
		//Go backwards
		direction = 'b';
        reversing_truck_sound_on();
		reverse();
		Serial.print("Backward\n");
	}

	//Consider if this needs to be an interrupt
	//Implement obstacle detection here:

	//3a)Stop moving until the obstacle has passed
	//Show the red light
    check_obstacle();
    
    //3b,c,d)
	//Implement trajectory control here:
	//Operates by briefly turning the vehicle
	adjust_trajectory();
	//Adjust speed via PID loop
    //adjust_motor_speed();
	
	//4)Check if the IR signal is received
	while(do_begin_unloading(is_egg_inside()) );

    //Turn off the truck sound
    reversing_truck_sound_off();

	//Time step = 20ms
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
    unloading_motor.attach(UNLOADING_MOTOR_EN);
    unloading_motor.write(CLOSE_GATE);
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
	pinMode(IR_LF_I, INPUT);
	pinMode(IR_LF_A, INPUT);
	pinMode(IR_LF_B, INPUT);
	pinMode(IR_LF_C, INPUT);
	pinMode(IR_LF_D, INPUT);
    //Rear IR_LF's
    pinMode(IR_LF_1, INPUT);
	pinMode(IR_LF_2, INPUT);
	pinMode(IR_LF_3, INPUT);
	pinMode(IR_LF_4, INPUT);
	pinMode(IR_LF_5, INPUT);
}
//Initialise obstacle detection
void init_IR_OD() {
    pinMode(IR_OD, INPUT);
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
    pinMode(IR_REC_I, INPUT);
    //pinMode(IR_REC_II, INPUT);
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

//Control the motor direction
void command(char c) {
        
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
        //White LED
        else if (c == 'w') {
            Serial.print("White light on!\n");
            Serial1.write(c);
        }
        //Rainbow fading colours
        else if (c == 'R') {
            Serial.print("RGB fading colours on!\n");
            Serial1.write('r');
        }
}

//Set a motor speed manually
void set_motor_speed(int speed) {
    analogWrite(ENA, speed);
    analogWrite(ENA, speed);
}


//Correct the trajectory of the vehicle if it veers away from the path
//Refer to sensor diagram
void adjust_trajectory() {
    /*
    Serial.print(digitalRead(IR_LF_I));
    Serial.print("\t");
    Serial.print(digitalRead(IR_LF_A));
    Serial.print("\t");
    Serial.print(digitalRead(IR_LF_B));
    Serial.print("\t");
    Serial.print(digitalRead(IR_LF_C));
    Serial.print("\t");
    Serial.println(digitalRead(IR_LF_D));
    */
    /*
    Serial.print(digitalRead(IR_LF_1));
    Serial.print("\t");
    Serial.print(digitalRead(IR_LF_2));
    Serial.print("\t");
    Serial.print(digitalRead(IR_LF_3));
    Serial.print("\t");
    Serial.print(digitalRead(IR_LF_4));
    Serial.print("\t");
    Serial.println(digitalRead(IR_LF_5));
    */
    //During moving forwards
	if (direction == 'f') {
		//Adjust to the right lightly, if it goes to the left
        if (digitalRead(IR_LF_I) && digitalRead(IR_LF_A) && !digitalRead(IR_LF_B) && digitalRead(IR_LF_C) && digitalRead(IR_LF_D)) {
            //left();
            right();
            Serial.println("Adjust trajectory");
            delay(5);
		}

		//Manual adjustment
		if (digitalRead(IR_LF_I) && digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && !digitalRead(IR_LF_C) && digitalRead(IR_LF_D)) {
            left();
            Serial.println("turn left");
            delay(TIME_STEP);
		}
        //HARD RIGHT
        if (digitalRead(IR_LF_I) && digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && digitalRead(IR_LF_C) && !digitalRead(IR_LF_D)) {
            Serial.println("turn left");
            left();
            delay(2 * TIME_STEP);
            
        }
		//Adjust to the left, if it veers to the right
		if (digitalRead(IR_LF_I) && !digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && digitalRead(IR_LF_C) && digitalRead(IR_LF_D)) {
            right();
            Serial.println("turn right");
			delay(TIME_STEP);
        }
        //HARD LEFT
        if (!digitalRead(IR_LF_I) && digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && digitalRead(IR_LF_C) && digitalRead(IR_LF_D)) {
            right();
            Serial.println("turn right");
            delay(2*TIME_STEP);
        }
        forward();

	}
	else if (direction == 'b') {
		//During reverse
        //Adjust right
        if (digitalRead(IR_LF_1) && digitalRead(IR_LF_2) && !digitalRead(IR_LF_3) && digitalRead(IR_LF_4) && digitalRead(IR_LF_5)) {
            Serial.println("Adjust trajectory");
            left();
            delay(5);
        }
        
		if (digitalRead(IR_LF_1) && digitalRead(IR_LF_2) && digitalRead(IR_LF_3) && !digitalRead(IR_LF_4) && digitalRead(IR_LF_5)) {
            left();
            Serial.println("turn right");
            delay(1);
        }
        //HARD RIGHT
        if (digitalRead(IR_LF_1) && digitalRead(IR_LF_2) && digitalRead(IR_LF_3) && digitalRead(IR_LF_4) && !digitalRead(IR_LF_5)) {
            //Inverted controls
            left();
            Serial.println("turn right");
            delay(10);
        }
        //Adjust to the left, if it veers to the right
        if (digitalRead(IR_LF_1) && !digitalRead(IR_LF_2) && digitalRead(IR_LF_3) && digitalRead(IR_LF_4) && digitalRead(IR_LF_5)) {
            right();
            Serial.println("turn left");
            delay(5);
        }
        //HARD LEFT
        if (!digitalRead(IR_LF_1) && digitalRead(IR_LF_2) && digitalRead(IR_LF_3) && digitalRead(IR_LF_4) && digitalRead(IR_LF_5)) {
            right();
            Serial.println("turn left");
            delay(15);
        }
        reverse();
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
	digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}
void right() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
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

    //Forwards
    if (direction == 'f') {
        forward();
    }
    //Reverse
    else if (direction == 'b') {
        reverse();
    }

    //adjust the speed of the motors
    analogWrite(ENA, power_a);
    analogWrite(ENB, power_b - 50);
}

//A function that checks if an object is nearby depending on the direction:
void check_obstacle() {
    //3a)Stop moving until the obstacle has passed
        //Show the red light
        bool has_stopped_from_object = false;
        while (is_object_nearby()) {
            Serial.print("Object detected!\n");
            off_green_light();
            halt();
            on_red_light();
            delay(50 * TIME_STEP);
            has_stopped_from_object = true;
        }
        if (has_stopped_from_object) {
            off_red_light();
        }
        on_green_light();
}

//Returns true if the IR obstacle detectors finds an object in front or the rear
//Otherwise false
//Stopping distance is set to 15.0
bool is_object_nearby() {

    double dist;

    //Check the direction of the obstacle
    //Don't stop of an obstacle behind the car if it's moving forwards

    //Fetch the distance from the Ultrasonic sensors
    if (direction == 'f') {
        dist = distance_by_USOD(USOD_F_TRIG, USOD_F_ECHO);
    }
    else {
        dist = distance_by_USOD(USOD_R_TRIG, USOD_R_ECHO);
    }

    //Debugging output
    /*
    Serial.print("USOD: ");
    Serial.println(dist);
    */
    //Object detected
    if (0.0 < dist && dist < STOP_DIST) {
        return true;
    }
    //Check and return if an object is nearby the vehicle
    //BEWARE SOMETIMES SENSORS INVERT THE SIGNAL
    return false;
}
//Returns true if the egg is inside the vehicle
//otherwise false
bool is_egg_inside() {
    return !digitalRead(IR_OD);
}

//Returns the distance in cm from the Ultrasonic obstacle detector
double distance_by_USOD(int trig_pin, int echo_pin) {

    unsigned long TimeOut = 10000; //In microseconds
    
    //Send 8 pulses at 40Khz for approx 10ms
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);
    //Make Echo_pin high and  Trigger pin Low


    //Wait for the echo pin to become high
    unsigned long endTime = micros() + TimeOut;//ENSURE THIS IS a LONG
    while (micros() < endTime && !digitalRead(echo_pin));

    //Begin timing for the echo pin
    unsigned long timeBegin = micros();
    endTime = timeBegin + TimeOut;
    while (micros() < endTime && digitalRead(echo_pin));

    //Calculate the distance
    double dist = double (micros() - timeBegin) * 34000.0 * (0.000001) / 2.0;
    
    //Print the results
    //Serial.print(dist);
    //Serial.print("cm \n");

    //return the result
    return dist;
}

//Return the number of ticks of the left or right motor
void count_left() {
    left_encoder +=1;
}
void count_right() {
    right_encoder +=1;
}

//Adjust the speed of the motor by counting the number of ticks
void adjust_motor_speed() {
    // Sample number of encoder ticks
	left_num_ticks = left_encoder;
	right_num_ticks = right_encoder;

	//Move
	drive(left_power, right_power);
	
	// Print out current number of ticks
	
	Serial.print(left_num_ticks);
	Serial.print("\t");
	Serial.println(right_num_ticks);
	

	// Number of ticks counted since last time
	//Proportional
	diff_left = left_num_ticks - left_encoder_prev;
	diff_right = right_num_ticks - right_encoder_prev;

	// Store current tick counter for next time
	left_encoder_prev = left_num_ticks;
	right_encoder_prev = right_num_ticks;

	
	if (diff_left > diff_right) {
        //Left is fast, slow it down and speed up right
		left_power -= MOTOR_OFFSET;
		right_power += MOTOR_OFFSET;
	}
	
	
	if (diff_left < diff_right) {
        //right is fast, slow it down and speed up left
		left_power += MOTOR_OFFSET;
		right_power -= MOTOR_OFFSET;
	}
    //Move
	drive(left_power, right_power);

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
    #if V2
    // rearrange ordering
    // - keeps same amount of pauses therefore will skip the same amount of IR receivers
    if (has_received_ir_signal() && egg_is_inside) {
        //0) Open Gate
        open_gate();    // code is same except this line moved to start

        //1) Stop the car
        halt();
        delay(UNLOAD_EGG);

        //2)Show the red light
        off_green_light();
        on_red_light();

        // keep delay from before incase this is important
        delay(UNLOAD_EGG);

        //3)Shut the gate
        close_gate();

        //4) Move away from the IR signal
        off_red_light();
        direction = 'b';
        reverse();
        adjust_trajectory();
        delay(GET_OUT_OFF_IR_SIGNAL);
    }
    #else 
    if (has_received_ir_signal() && egg_is_inside) {
        //stop and unload the egg
        //Show the red light when stopped   
        
        //0) Stop the car
        halt();
        delay(UNLOAD_EGG);
        //1)Show the red light
        off_green_light();
        on_red_light();

        //2)Open the gate till the egg rolls out
        open_gate();
        delay(UNLOAD_EGG);

        //3)Shut the gate
        close_gate();

        //4) Move away from the IR signal
        off_red_light();
        direction = 'b';
        reverse();
        adjust_trajectory();
        delay(GET_OUT_OFF_IR_SIGNAL);
        
    }
    #endif
    else if (has_received_ir_signal() && !egg_is_inside) {
        //Wait for the egg to be loaded;
        //and show the red light when stopped
        Serial.println("Load the egg\n");

        //0)Stop the car 
        halt();
        //1)Show red light
        off_green_light();
        on_red_light();

        //2)Wait for the egg to be loaded
        while(!is_egg_inside());
        off_red_light();

        //3)Move away from IR signal
        direction = 'f';
        forward();
        adjust_trajectory();
        delay(GET_OUT_OFF_IR_SIGNAL);
        

    }
    return false;
}

//Returns true if one of the IR recievers detects a signal from the transmitters
bool has_received_ir_signal() {
    //Debugging
    /*
    Serial.println(!digitalRead(IR_REC_I));
    Serial.println(!digitalRead(IR_REC_II));
    */
    return !digitalRead(IR_REC_I);
}

//Open the gate
void open_gate() {
    unloading_motor.write(OPEN_GATE);
}
//Close the gate and turn off the motor
void close_gate() {
    unloading_motor.write(CLOSE_GATE);
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
//Sends a high signal to a NANO to play the sound
void reversing_truck_sound_on() {
    digitalWrite(REV_TRUCK_SND_PIN, HIGH);
}
//Sends a high signal to a NANO to stop the sound
void reversing_truck_sound_off() {
    digitalWrite(REV_TRUCK_SND_PIN, LOW);
}